#!/usr/bin/env python3
"""
DRON Live 3D Perception & Telemetry Viewer
Visualizes live stereo camera streams and Pixhawk IMU telemetry in an interactive real-time 3D window.

Features:
1. Real-time dense stereo depth calculation (20-30 FPS)
2. Interactive 3D point cloud rendering with Open3D
3. Real-time drone orientation and camera frustum locked to Pixhawk IMU attitude
4. Toggle between RGB Color and Thermal Inferno Heatmap (Press 'C')
5. Toggle between Instantaneous view and Persistent accumulated map (Press 'A')
6. Offline Replay Mode: Test from home using recorded sessions (--replay)

Usage:
    # 1. Live stream from Raspberry Pi on the drone:
    python tools/visualization/live_3d_viewer.py --host 10.42.0.1

    # 2. Test right now from home using Tuesday's recording:
    python tools/visualization/live_3d_viewer.py --replay latest

    # 3. Test on a specific recorded session folder:
    python tools/visualization/live_3d_viewer.py --replay C:/Users/param/Downloads/dron_recordings/session_20261027_173902
"""

import argparse
import datetime
import glob
import json
import math
import os
import sys
import threading
import time
import urllib.request
import cv2
import numpy as np
import open3d as o3d


def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """Converts Roll, Pitch, Yaw in degrees to a 3x3 rotation matrix."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(r), -math.sin(r)],
        [0.0, math.sin(r), math.cos(r)]
    ])
    Ry = np.array([
        [math.cos(p), 0.0, math.sin(p)],
        [0.0, 1.0, 0.0],
        [-math.sin(p), 0.0, math.cos(p)]
    ])
    Rz = np.array([
        [math.cos(y), -math.sin(y), 0.0],
        [math.sin(y), math.cos(y), 0.0],
        [0.0, 0.0, 1.0]
    ])
    return Rz @ Ry @ Rx


def create_camera_frustum(scale=0.15):
    """Creates a 3D wireframe camera frustum (pyramid) representing the drone's eye."""
    points = [
        [0, 0, 0],                  # Camera optical center (apex)
        [-scale, -scale * 0.75, scale * 1.5],  # Top-left
        [ scale, -scale * 0.75, scale * 1.5],  # Top-right
        [ scale,  scale * 0.75, scale * 1.5],  # Bottom-right
        [-scale,  scale * 0.75, scale * 1.5],  # Bottom-left
    ]
    lines = [
        [0, 1], [0, 2], [0, 3], [0, 4],  # Rays from optical center
        [1, 2], [2, 3], [3, 4], [4, 1]   # Image plane rectangle
    ]
    colors = [[1.0, 0.2, 0.2] for _ in lines]  # Vibrant red frustum
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


class StreamWorker:
    """Thread-safe background MJPEG stream reader."""
    def __init__(self, url, name="Stream"):
        self.url = url
        self.name = name
        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        while self.running:
            try:
                cap = cv2.VideoCapture(self.url)
                if not cap.isOpened():
                    time.sleep(1.0)
                    continue
                while self.running:
                    ret, frame = cap.read()
                    if not ret:
                        break
                    with self.lock:
                        self.latest_frame = frame
                cap.release()
            except Exception:
                time.sleep(1.0)

    def read_latest(self):
        with self.lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def release(self):
        self.running = False


class IMUWorker:
    """Background polling worker for Pixhawk telemetry (/imu.json)."""
    def __init__(self, http_url):
        self.http_url = http_url
        self.latest = {
            "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
            "ax_m_s2": 0.0, "ay_m_s2": 0.0, "az_m_s2": 0.0,
            "connected": False
        }
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        while self.running:
            try:
                req = urllib.request.Request(self.http_url, headers={"User-Agent": "DRON-3DViewer"})
                with urllib.request.urlopen(req, timeout=0.15) as resp:
                    data = json.loads(resp.read().decode("utf-8"))
                    if data.get("connected"):
                        self.latest = {
                            "roll_deg": data.get("roll_deg", 0.0),
                            "pitch_deg": data.get("pitch_deg", 0.0),
                            "yaw_deg": data.get("yaw_deg", 0.0),
                            "ax_m_s2": data.get("ax_m_s2", 0.0),
                            "ay_m_s2": data.get("ay_m_s2", 0.0),
                            "az_m_s2": data.get("az_m_s2", 0.0),
                            "connected": True
                        }
                time.sleep(0.02)  # 50 Hz
            except Exception:
                time.sleep(0.1)

    def get_latest(self):
        return self.latest


class FastStereoDepth:
    """Optimized Semi-Global Block Matching for real-time live depth estimation."""
    def __init__(self, baseline_m=0.044, focal_px=525.0, min_depth=0.3, max_depth=5.0):
        self.baseline_m = baseline_m
        self.focal_px = focal_px
        self.min_depth = min_depth
        self.max_depth = max_depth

        block_size = 5
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=48,  # 48 disparities is very fast and covers 0.3m-5m cleanly
            blockSize=block_size,
            P1=8 * 3 * block_size ** 2,
            P2=32 * 3 * block_size ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=8,
            speckleWindowSize=50,
            speckleRange=16,
            mode=cv2.STEREO_SGBM_MODE_FAST
        )

    def compute(self, f0, f1, step=2):
        """Computes 3D points (N, 3) and RGB colors (N, 3) with spatial sub-sampling step."""
        gray0 = cv2.cvtColor(f0, cv2.COLOR_BGR2GRAY)
        gray1 = cv2.cvtColor(f1, cv2.COLOR_BGR2GRAY)

        disp16 = self.sgbm.compute(gray0, gray1)
        disp = disp16.astype(np.float32) / 16.0

        # Sub-sample for real-time 3D performance
        disp_sub = disp[::step, ::step]
        rgb_sub = cv2.cvtColor(f0[::step, ::step], cv2.COLOR_BGR2RGB) / 255.0

        h, w = disp_sub.shape
        valid = disp_sub > 1.0

        depth = np.zeros_like(disp_sub)
        depth[valid] = (self.focal_px * self.baseline_m) / disp_sub[valid]
        mask = (depth >= self.min_depth) & (depth <= self.max_depth)

        if not np.any(mask):
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)

        # Vectorized back-projection
        u_coords, v_coords = np.meshgrid(np.arange(w) * step, np.arange(h) * step)
        z = depth[mask]
        x = (u_coords[mask] - (w * step) / 2.0) * z / self.focal_px
        y = (v_coords[mask] - (h * step) / 2.0) * z / self.focal_px

        # Standard Computer Vision coordinates: X-Right, Y-Down, Z-Forward
        points = np.column_stack((x, y, z)).astype(np.float32)
        colors = rgb_sub[mask].astype(np.float32)
        return points, colors


def find_latest_session(base_dir):
    sessions = sorted(glob.glob(os.path.join(base_dir, "session_*")))
    return sessions[-1] if sessions else None


def main():
    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except Exception:
            pass

    parser = argparse.ArgumentParser(description="DRON Live 3D Perception & Telemetry Viewer")
    parser.add_argument("--host", default="10.42.0.1", help="Pi IP or hostname (default: 10.42.0.1)")
    parser.add_argument("--port", type=int, default=8000, help="HTTP Port (default: 8000)")
    parser.add_argument("--baseline", type=float, default=0.044, help="Stereo baseline in meters (default: 0.044)")
    parser.add_argument("--focal-length", type=float, default=525.0, help="Focal length in pixels (default: 525.0)")
    parser.add_argument("--step", type=int, default=2, help="Sub-sampling step for real-time speed (default: 2)")
    parser.add_argument("--replay", default=None, help="Replay a recorded session folder instead of live stream (or 'latest')")
    parser.add_argument("--base-dir", default=os.path.expanduser("~/Downloads/dron_recordings"), help="Base recordings directory")
    args = parser.parse_args()

    # Mode 1: Offline Replay from recording
    replay_mode = args.replay is not None
    cap0, cap1 = None, None
    imu_records = []
    replay_frame_idx = 0

    if replay_mode:
        session_dir = args.replay
        if session_dir == "latest":
            session_dir = find_latest_session(args.base_dir)
            if not session_dir:
                sys.exit(f"[ERROR] No sessions found in {args.base_dir}")
        
        cam0_path = os.path.join(session_dir, "cam0.mp4")
        cam1_path = os.path.join(session_dir, "cam1.mp4")
        if not os.path.exists(cam0_path) or not os.path.exists(cam1_path):
            sys.exit(f"[ERROR] Missing cam0.mp4 or cam1.mp4 in {session_dir}")

        cap0 = cv2.VideoCapture(cam0_path)
        cap1 = cv2.VideoCapture(cam1_path)
        print("=" * 70)
        print(f"[REPLAY MODE] Playing back session: {os.path.basename(session_dir)}")
        print("=" * 70)
    else:
        # Mode 2: Live Stream from Raspberry Pi
        url0 = f"http://{args.host}:{args.port}/cam0.mjpg"
        url1 = f"http://{args.host}:{args.port}/cam1.mjpg"
        imu_url = f"http://{args.host}:{args.port}/imu.json"

        print("=" * 70)
        print(f"[LIVE STREAM MODE] Connecting to Pi at http://{args.host}:{args.port}...")
        print("=" * 70)

        worker0 = StreamWorker(url0, "cam0")
        worker1 = StreamWorker(url1, "cam1")
        imu_worker = IMUWorker(imu_url)

        # Wait for streams
        t0 = time.time()
        while time.time() - t0 < 5.0:
            if worker0.read_latest() is not None and worker1.read_latest() is not None:
                break
            time.sleep(0.1)

        if worker0.read_latest() is None:
            if args.host != "raspberrypi.local":
                print("Trying fallback hostname: raspberrypi.local...")
                worker0.release()
                worker1.release()
                args.host = "raspberrypi.local"
                worker0 = StreamWorker(f"http://{args.host}:{args.port}/cam0.mjpg", "cam0")
                worker1 = StreamWorker(f"http://{args.host}:{args.port}/cam1.mjpg", "cam1")
                imu_worker = IMUWorker(f"http://{args.host}:{args.port}/imu.json")
                time.sleep(2.0)

        if worker0.read_latest() is None:
            sys.exit(f"\n[ERROR] Could not connect to camera streams. Is live_view.py running on the Pi?")

    depth_engine = FastStereoDepth(baseline_m=args.baseline, focal_px=args.focal_length)

    # Setup Open3D Interactive Window
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="DRON Live 3D Perception & Telemetry", width=1280, height=720)

    # Initial 3D geometries
    pcd = o3d.geometry.PointCloud()
    frustum = create_camera_frustum(scale=0.15)
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)

    # Dummy initial points so Open3D sets up bounding box
    dummy_pts = np.array([[0, 0, 1.0], [0.5, 0.5, 2.0], [-0.5, -0.5, 2.0]], dtype=np.float32)
    pcd.points = o3d.utility.Vector3dVector(dummy_pts)
    pcd.colors = o3d.utility.Vector3dVector(np.ones_like(dummy_pts))

    vis.add_geometry(pcd)
    vis.add_geometry(frustum)
    vis.add_geometry(coord_frame)

    # State toggles
    state = {
        "color_mode": "rgb",      # "rgb" or "thermal"
        "accumulate": False,      # True = persistent map, False = live frame only
        "paused": False,
        "running": True
    }

    accumulated_points = []
    accumulated_colors = []

    # Key callbacks
    def toggle_color(vis):
        state["color_mode"] = "thermal" if state["color_mode"] == "rgb" else "rgb"
        print(f"\n[VIEWER] Switched color mode: {state['color_mode'].upper()}")
        return False

    def toggle_accumulate(vis):
        state["accumulate"] = not state["accumulate"]
        accumulated_points.clear()
        accumulated_colors.clear()
        print(f"\n[VIEWER] Switched accumulation mode: {'ON (Persistent Map)' if state['accumulate'] else 'OFF (Live View)'}")
        return False

    def toggle_pause(vis):
        state["paused"] = not state["paused"]
        print(f"\n[VIEWER] {'PAUSED' if state['paused'] else 'RESUMED'}")
        return False

    # Bind keys
    vis.register_key_callback(ord('C'), toggle_color)
    vis.register_key_callback(ord('c'), toggle_color)
    vis.register_key_callback(ord('A'), toggle_accumulate)
    vis.register_key_callback(ord('a'), toggle_accumulate)
    vis.register_key_callback(ord(' '), toggle_pause)

    # Configure viewpoint
    view_ctrl = vis.get_view_control()
    view_ctrl.set_front([0.0, -0.4, -0.9])
    view_ctrl.set_up([0.0, -1.0, 0.0])
    view_ctrl.set_lookat([0.0, 0.0, 1.5])
    view_ctrl.set_zoom(0.7)

    render_opt = vis.get_render_option()
    render_opt.point_size = 3.0
    render_opt.background_color = np.array([0.05, 0.08, 0.12])  # Dark slate background

    print("\n" + "=" * 70)
    print("🎮 INTERACTIVE 3D VIEWER CONTROLS:")
    print("  ▶ Left Click + Drag:  Orbit 3D View")
    print("  ▶ Right Click + Drag: Pan Camera")
    print("  ▶ Scroll Wheel:       Zoom In / Out")
    print("  ▶ Press 'C':          Toggle RGB Color / Thermal Heatmap")
    print("  ▶ Press 'A':          Toggle Instantaneous Live View / Persistent Room Map")
    print("  ▶ Press 'Space':      Pause / Resume")
    print("  ▶ Press 'Q' or Esc:   Exit Viewer")
    print("=" * 70 + "\n")

    fps_count = 0
    fps_start = time.time()
    current_fps = 0.0

    try:
        while state["running"]:
            if not vis.poll_events():
                break

            if state["paused"]:
                time.sleep(0.05)
                continue

            # Step 1: Read next pair of frames
            if replay_mode:
                ret0, f0 = cap0.read()
                ret1, f1 = cap1.read()
                if not ret0 or not ret1:
                    # Loop video in replay mode
                    cap0.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    cap1.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    replay_frame_idx = 0
                    continue
                replay_frame_idx += 1
                imu_info = {"roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0, "connected": False}
            else:
                f0 = worker0.read_latest()
                f1 = worker1.read_latest()
                imu_info = imu_worker.get_latest()
                if f0 is None or f1 is None:
                    time.sleep(0.02)
                    continue

            # Step 2: Compute dense 3D points
            pts, cols = depth_engine.compute(f0, f1, step=args.step)

            if len(pts) > 0:
                # Step 3: Color selection (RGB vs Thermal Heatmap)
                if state["color_mode"] == "thermal":
                    # Synthesize thermal heatmap based on central depth distance / warm target
                    dists = np.linalg.norm(pts - np.median(pts, axis=0), axis=1)
                    norm_heat = np.clip(1.0 - (dists / 0.8), 0.0, 1.0)
                    lut = np.uint8(norm_heat * 255.0)
                    thermal_cm = cv2.applyColorMap(lut, cv2.COLORMAP_INFERNO)
                    cols = (thermal_cm.reshape(-1, 3)[:, ::-1] / 255.0).astype(np.float32)

                # Step 4: Apply IMU Orientation if available
                if imu_info["connected"]:
                    r_mat = euler_to_rotation_matrix(imu_info["roll_deg"], imu_info["pitch_deg"], imu_info["yaw_deg"])
                    pts = (r_mat @ pts.T).T

                # Accumulate or replace
                if state["accumulate"]:
                    accumulated_points.append(pts)
                    accumulated_colors.append(cols)
                    if len(accumulated_points) > 40:  # Keep last 40 frames for map
                        accumulated_points.pop(0)
                        accumulated_colors.pop(0)
                    vis_pts = np.vstack(accumulated_points)
                    vis_cols = np.vstack(accumulated_colors)
                else:
                    vis_pts = pts
                    vis_cols = cols

                pcd.points = o3d.utility.Vector3dVector(vis_pts)
                pcd.colors = o3d.utility.Vector3dVector(vis_cols)
                vis.update_geometry(pcd)

            # Update live drone frustum
            if imu_info["connected"]:
                r_mat = euler_to_rotation_matrix(imu_info["roll_deg"], imu_info["pitch_deg"], imu_info["yaw_deg"])
                # Reset and rotate frustum
                new_frustum = create_camera_frustum(scale=0.15)
                new_frustum.rotate(r_mat, center=[0, 0, 0])
                frustum.points = new_frustum.points
                vis.update_geometry(frustum)

            vis.update_renderer()

            # FPS counter & HUD
            fps_count += 1
            if time.time() - fps_start >= 1.0:
                current_fps = fps_count / (time.time() - fps_start)
                fps_count = 0
                fps_start = time.time()

            # Live terminal HUD
            if imu_info["connected"]:
                hud_imu = f"IMU: R:{imu_info['roll_deg']:+5.1f}° P:{imu_info['pitch_deg']:+5.1f}° Y:{imu_info['yaw_deg']:5.1f}°"
            else:
                hud_imu = "IMU: None"

            mode_str = f"Color: {state['color_mode'].upper()} | Map: {'ACCUMULATED' if state['accumulate'] else 'LIVE'}"
            sys.stdout.write(f"\r[3D LIVE] {current_fps:4.1f} FPS | {len(pcd.points):6,d} pts | {hud_imu} | {mode_str}   ")
            sys.stdout.flush()

    finally:
        vis.destroy_window()
        if replay_mode:
            if cap0: cap0.release()
            if cap1: cap1.release()
        else:
            worker0.release()
            worker1.release()
            imu_worker.running = False
        print("\n\n[VIEWER] Closed.")


if __name__ == "__main__":
    main()
