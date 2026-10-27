#!/usr/bin/env python3
"""
DRON Stream Recorder (Runs on your Laptop/PC)
Connects to the Pi's live stream, records N seconds of video from both cameras,
and saves timestamped .mp4 files.

Usage:
    python tools/visualization/record_stream.py --seconds 10
    python tools/visualization/record_stream.py --seconds 5 --host 10.42.0.1
    python tools/visualization/record_stream.py --seconds 15 --out-dir C:/Users/param/Downloads/dron_recordings
"""

import argparse
import datetime
import os
import sys
import threading
import time
import cv2
import numpy as np


class StreamWorker:
    """Continuously reads frames from an MJPEG HTTP stream to prevent buffer lag."""
    def __init__(self, url, name):
        self.url = url
        self.name = name
        self.frame = None
        self.running = True
        self.connected = False
        self.cap = cv2.VideoCapture(url)
        if self.cap.isOpened():
            self.connected = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _reader_loop(self):
        while self.running:
            if not self.cap.isOpened():
                time.sleep(0.1)
                try:
                    self.cap = cv2.VideoCapture(self.url)
                except Exception:
                    pass
                continue
            try:
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    self.frame = frame
                    self.connected = True
                else:
                    time.sleep(0.01)
            except Exception:
                break

    def read_latest(self):
        return self.frame

    def release(self):
        self.running = False
        if self.cap:
            self.cap.release()


def main():
    parser = argparse.ArgumentParser(description="Record live camera streams from Raspberry Pi")
    parser.add_argument("-s", "--seconds", type=float, default=10.0, help="Duration in seconds (default: 10.0)")
    parser.add_argument("--host", default="raspberrypi.local", help="Host IP or hostname (default: raspberrypi.local)")
    parser.add_argument("--port", type=int, default=8000, help="Port (default: 8000)")
    parser.add_argument("--fps", type=float, default=25.0, help="Recording FPS (default: 25.0)")
    parser.add_argument("-o", "--out-dir", default=os.path.expanduser("~/Downloads/dron_recordings"),
                        help="Output directory for saved videos")
    parser.add_argument("--side-by-side", action="store_true", default=True,
                        help="Also save a combined side-by-side stereo video (default: True)")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    # Stream URLs
    url0 = f"http://{args.host}:{args.port}/cam0.mjpg"
    url1 = f"http://{args.host}:{args.port}/cam1.mjpg"

    print(f"Connecting to streams at http://{args.host}:{args.port}...")
    worker0 = StreamWorker(url0, "cam0")
    worker1 = StreamWorker(url1, "cam1")

    # Wait for first frames
    print("Waiting for video feeds to initialize...")
    wait_start = time.time()
    while time.time() - wait_start < 5.0:
        f0 = worker0.read_latest()
        f1 = worker1.read_latest()
        if f0 is not None and f1 is not None:
            break
        time.sleep(0.1)

    f0 = worker0.read_latest()
    f1 = worker1.read_latest()

    if f0 is None or f1 is None:
        # If raspberrypi.local failed, try 10.42.0.1
        if args.host == "raspberrypi.local":
            print("Trying fallback IP: 10.42.0.1...")
            worker0.release()
            worker1.release()
            args.host = "10.42.0.1"
            url0 = f"http://{args.host}:{args.port}/cam0.mjpg"
            url1 = f"http://{args.host}:{args.port}/cam1.mjpg"
            worker0 = StreamWorker(url0, "cam0")
            worker1 = StreamWorker(url1, "cam1")
            time.sleep(1.5)
            f0 = worker0.read_latest()
            f1 = worker1.read_latest()

    if f0 is None or f1 is None:
        worker0.release()
        worker1.release()
        sys.exit(f"\n[ERROR] Could not connect to streams at http://{args.host}:{args.port}. Is live_view.py running on the Pi?")

    h, w = f0.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    # Dedicated folder for this recording session
    session_dir = os.path.join(args.out_dir, f"session_{timestamp}")
    os.makedirs(session_dir, exist_ok=True)

    file_cam0 = os.path.join(session_dir, "cam0.mp4")
    file_cam1 = os.path.join(session_dir, "cam1.mp4")
    file_stereo = os.path.join(session_dir, "stereo.mp4")

    out0 = cv2.VideoWriter(file_cam0, fourcc, args.fps, (w, h))
    out1 = cv2.VideoWriter(file_cam1, fourcc, args.fps, (w, h))
    out_stereo = cv2.VideoWriter(file_stereo, fourcc, args.fps, (w * 2, h)) if args.side_by_side else None

    print(f"\n" + "=" * 60)
    print(f"🎬 RECORDING STARTED: {args.seconds} seconds @ {args.fps} FPS")
    print(f"📁 Saving to: {args.out_dir}")
    print(f"=" * 60)

    start_time = time.time()
    frame_interval = 1.0 / args.fps
    saved_frames = 0

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed >= args.seconds:
                break

            loop_start = time.time()
            f0 = worker0.read_latest()
            f1 = worker1.read_latest()

            if f0 is not None and f1 is not None:
                out0.write(f0)
                out1.write(f1)
                if out_stereo:
                    sbs = np.hstack([f0, f1])
                    out_stereo.write(sbs)
                saved_frames += 1

            # Progress bar
            progress = min(1.0, elapsed / args.seconds)
            bar_len = 30
            filled = int(bar_len * progress)
            bar = "█" * filled + "░" * (bar_len - filled)
            sys.stdout.write(f"\rRecording: [{bar}] {elapsed:.1f}s / {args.seconds:.1f}s ({saved_frames} frames)")
            sys.stdout.flush()

            # Maintain frame pacing
            dt = time.time() - loop_start
            if dt < frame_interval:
                time.sleep(frame_interval - dt)

    finally:
        out0.release()
        out1.release()
        if out_stereo:
            out_stereo.release()
        worker0.release()
        worker1.release()

    # Save session metadata
    meta_path = os.path.join(session_dir, "metadata.json")
    try:
        import json
        with open(meta_path, "w") as mf:
            json.dump({
                "timestamp": timestamp,
                "duration_seconds": args.seconds,
                "fps": args.fps,
                "frame_count": saved_frames,
                "resolution": {"width": w, "height": h},
                "host": args.host
            }, mf, indent=2)
    except Exception:
        pass

    print(f"\n\n✅ RECORDING COMPLETE! ({saved_frames} frames saved)")
    print(f"📁 Session Folder: {session_dir}")
    print(f"  ▶ Camera 0:           {file_cam0}")
    print(f"  ▶ Camera 1:           {file_cam1}")
    if args.side_by_side:
        print(f"  ▶ Side-by-Side Stereo: {file_stereo}")
    print(f"  ▶ Metadata:           {meta_path}")


if __name__ == "__main__":
    main()
