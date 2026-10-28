#!/usr/bin/env python3
"""
DRON Stereo Depth & 3D Point Cloud Processor
Processes paired stereo video recordings (cam0.mp4 & cam1.mp4),
computes dense metric depth using Semi-Global Block Matching (SGBM),
generates a colorized depth video, a 3-pane comparison video,
and exports 3D point clouds (.ply) with RGB color.

Usage:
    python tools/visualization/process_stereo_video.py
    python tools/visualization/process_stereo_video.py --session C:/Users/param/Downloads/dron_recordings/session_20261027_174153
    python tools/visualization/process_stereo_video.py --export-ply
"""

import argparse
import glob
import json
import os
import sys
import time
import cv2
import numpy as np


def find_latest_session(base_dir):
    sessions = sorted(glob.glob(os.path.join(base_dir, "session_*")))
    if not sessions:
        return None
    return sessions[-1]


def export_ply(filename, points, colors):
    """Write an ASCII or binary PLY point cloud file."""
    assert len(points) == len(colors), "Points and colors count must match"
    num_points = len(points)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {num_points}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )
    with open(filename, "wb") as f:
        f.write(header.encode("ascii"))
        # Pack float x,y,z, uint8 r,g,b
        vertex_data = np.empty(num_points, dtype=[
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
            ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')
        ])
        vertex_data['x'] = points[:, 0]
        vertex_data['y'] = points[:, 1]
        vertex_data['z'] = points[:, 2]
        vertex_data['red'] = colors[:, 0]
        vertex_data['green'] = colors[:, 1]
        vertex_data['blue'] = colors[:, 2]
        f.write(vertex_data.tobytes())


def main():
    parser = argparse.ArgumentParser(description="Process stereo videos to depth video and 3D point cloud")
    parser.add_argument("--session", default=None, help="Path to session folder (defaults to latest in Downloads/dron_recordings)")
    parser.add_argument("--base-dir", default=os.path.expanduser("~/Downloads/dron_recordings"), help="Base recordings directory")
    parser.add_argument("--num-disparities", type=int, default=64, help="Must be divisible by 16 (default: 64)")
    parser.add_argument("--block-size", type=int, default=7, help="SGBM block size (default: 7)")
    parser.add_argument("--baseline", type=float, default=0.044, help="Camera baseline in meters (default: 0.044m = 44mm)")
    parser.add_argument("--focal-length", type=float, default=525.0, help="Focal length fx in pixels (default: 525.0)")
    parser.add_argument("--max-depth", type=float, default=5.0, help="Maximum depth cutoff in meters (default: 5.0m)")
    parser.add_argument("--min-depth", type=float, default=0.3, help="Minimum depth cutoff in meters (default: 0.3m)")
    parser.add_argument("--keyframe", type=int, default=50, help="Frame index to export as 3D point cloud (.ply)")
    args = parser.parse_args()

    session_dir = args.session
    if not session_dir:
        session_dir = find_latest_session(args.base_dir)
        if not session_dir:
            sys.exit(f"[ERROR] No session folders found in {args.base_dir}")

    cam0_path = os.path.join(session_dir, "cam0.mp4")
    cam1_path = os.path.join(session_dir, "cam1.mp4")

    if not os.path.exists(cam0_path) or not os.path.exists(cam1_path):
        sys.exit(f"[ERROR] Missing cam0.mp4 or cam1.mp4 in {session_dir}")

    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except Exception:
            pass

    print("=" * 65)
    print("[DRON] STEREO 3D RECONSTRUCTION ENGINE")
    print(f"  Processing Session: {os.path.basename(session_dir)}")
    print(f"  ▶ Baseline:     {args.baseline * 1000:.1f} mm")
    print(f"  ▶ Focal Length: {args.focal_length:.1f} px")
    print(f"  ▶ Depth Range:  {args.min_depth:.1f}m - {args.max_depth:.1f}m")
    print("=" * 65)

    cap0 = cv2.VideoCapture(cam0_path)
    cap1 = cv2.VideoCapture(cam1_path)

    total_frames0 = int(cap0.get(cv2.CAP_PROP_FRAME_COUNT))
    total_frames1 = int(cap1.get(cv2.CAP_PROP_FRAME_COUNT))
    total_frames = min(total_frames0, total_frames1)
    fps = cap0.get(cv2.CAP_PROP_FPS) or 25.0
    w = int(cap0.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap0.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cx, cy = w / 2.0, h / 2.0

    print(f"Loaded videos: {w}x{h} @ {fps:.1f} FPS ({total_frames} frames)")

    # Setup StereoSGBM Matcher
    p1 = 8 * 3 * (args.block_size ** 2)
    p2 = 32 * 3 * (args.block_size ** 2)
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=args.num_disparities,
        blockSize=args.block_size,
        P1=p1,
        P2=p2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    # Output Video Writers
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    depth_video_path = os.path.join(session_dir, "depth_video.mp4")
    triple_video_path = os.path.join(session_dir, "stereo_depth_triple.mp4")

    out_depth = cv2.VideoWriter(depth_video_path, fourcc, fps, (w, h))
    out_triple = cv2.VideoWriter(triple_video_path, fourcc, fps, (w * 3, h))

    best_keyframe_points = None
    best_keyframe_colors = None
    best_valid_count = 0

    frame_idx = 0
    start_time = time.time()
    valid_ratios = []

    try:
        while True:
            ret0, f0 = cap0.read()
            ret1, f1 = cap1.read()
            if not ret0 or not ret1:
                break

            # Convert to grayscale for stereo matcher
            gray0 = cv2.cvtColor(f0, cv2.COLOR_BGR2GRAY)
            gray1 = cv2.cvtColor(f1, cv2.COLOR_BGR2GRAY)

            # Compute Disparity (Left = f0, Right = f1)
            raw_disp = sgbm.compute(gray0, gray1).astype(np.float32) / 16.0

            # Valid disparity mask
            valid_mask = (raw_disp > 0.5) & (raw_disp < args.num_disparities)
            valid_count = np.count_nonzero(valid_mask)
            valid_ratios.append(valid_count / (w * h))

            # Compute Metric Depth: Z = (f * B) / disparity
            depth_m = np.zeros_like(raw_disp)
            depth_m[valid_mask] = (args.focal_length * args.baseline) / raw_disp[valid_mask]
            depth_valid = valid_mask & (depth_m >= args.min_depth) & (depth_m <= args.max_depth)

            # Normalize for colorization: 0m (red/warm) -> max_depth (blue/cool)
            norm_depth = np.zeros_like(depth_m, dtype=np.uint8)
            norm_depth[depth_valid] = np.clip(
                ((depth_m[depth_valid] - args.min_depth) / (args.max_depth - args.min_depth)) * 255.0,
                0, 255
            ).astype(np.uint8)

            color_depth = cv2.applyColorMap(norm_depth, cv2.COLORMAP_TURBO)
            color_depth[~depth_valid] = [15, 15, 20]  # dark background for unmeasured areas

            # Overlay info text
            avg_d = np.mean(depth_m[depth_valid]) if np.count_nonzero(depth_valid) > 0 else 0
            cv2.putText(color_depth, f"Frame {frame_idx} | Avg Depth: {avg_d:.2f}m", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(color_depth, f"Range: {args.min_depth:.1f}m - {args.max_depth:.1f}m", (15, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

            # Write Depth Video
            out_depth.write(color_depth)

            # Write Triple Comparison Video: [Left | Right | Depth]
            f0_labeled = f0.copy()
            f1_labeled = f1.copy()
            cv2.putText(f0_labeled, "Camera 0 (Left)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(f1_labeled, "Camera 1 (Right)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            triple_frame = np.hstack([f0_labeled, f1_labeled, color_depth])
            out_triple.write(triple_frame)

            # Save 3D Point Cloud for keyframe (or frame with highest valid matches)
            if valid_count > best_valid_count and (frame_idx == args.keyframe or best_keyframe_points is None or valid_count > best_valid_count * 1.2):
                best_valid_count = valid_count
                # Backproject 2D pixels to 3D metric coordinates
                v_coords, u_coords = np.where(depth_valid)
                z = depth_m[depth_valid]
                x = ((u_coords - cx) * z) / args.focal_length
                y = ((v_coords - cy) * z) / args.focal_length
                pts = np.stack([x, y, z], axis=1).astype(np.float32)
                # Sample RGB colors from left image
                rgb = f0[depth_valid][:, ::-1]  # BGR to RGB
                best_keyframe_points = pts
                best_keyframe_colors = rgb

            frame_idx += 1
            if frame_idx % 25 == 0 or frame_idx == total_frames:
                elapsed = time.time() - start_time
                fps_proc = frame_idx / elapsed if elapsed > 0 else 0
                pct = (frame_idx / total_frames) * 100
                sys.stdout.write(f"\rProcessing: {frame_idx}/{total_frames} frames ({pct:.0f}%) @ {fps_proc:.1f} FPS")
                sys.stdout.flush()

    finally:
        cap0.release()
        cap1.release()
        out_depth.release()
        out_triple.release()

    # Export 3D Point Cloud
    ply_path = os.path.join(session_dir, "lab_room_3d.ply")
    if best_keyframe_points is not None and len(best_keyframe_points) > 0:
        export_ply(ply_path, best_keyframe_points, best_keyframe_colors)
        ply_status = f"[OK] Exported {len(best_keyframe_points):,} 3D points to: {os.path.basename(ply_path)}"
    else:
        ply_status = "[WARN] No valid 3D points found for export"

    mean_valid = (np.mean(valid_ratios) * 100) if valid_ratios else 0
    print(f"\n\n" + "=" * 65)
    print(f"[SUCCESS] STEREO 3D RECONSTRUCTION COMPLETE!")
    print(f"Quality Metrics:")
    print(f"  - Average Stereo Match Density: {mean_valid:.1f}%")
    print(f"  - Total Frames Processed:       {frame_idx}")
    print(f"\nGenerated Outputs in: {session_dir}")
    print(f"  1. Colorized Depth Video:    {depth_video_path}")
    print(f"  2. Triple Comparison Video:  {triple_video_path}")
    print(f"  3. 3D Point Cloud (PLY):     {ply_path}")
    print(f"     {ply_status}")
    print("=" * 65)


if __name__ == "__main__":
    main()
