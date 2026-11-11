#!/usr/bin/env python3
"""
DRON Stream & Telemetry Recorder (Runs on your Laptop/PC)
Connects to the Pi's live HTTP stream over Wi-Fi, records N seconds of:
  1. Synchronized Left Camera video (cam0.mp4)
  2. Synchronized Right Camera video (cam1.mp4)
  3. Side-by-side stereo depth video (stereo.mp4)
  4. High-rate Pixhawk IMU Telemetry (imu.csv: Roll, Pitch, Yaw, Quaternions, Angular Rates)
  5. Session metadata (metadata.json)

Usage:
    python tools/visualization/record_stream.py --seconds 10
    python tools/visualization/record_stream.py --seconds 15 --host 10.42.0.1
    python tools/visualization/record_stream.py --seconds 20 --imu-port COM3  # Direct USB telemetry fallback
"""

import argparse
import datetime
import json
import math
import os
import struct
import sys
import threading
import time
import urllib.request
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
            if not self.cap or not self.cap.isOpened():
                time.sleep(0.05)
                continue
            try:
                if not self.running:
                    break
                ret, frame = self.cap.read()
                if not self.running:
                    break
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
        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass


class IMUWorker:
    """Records high-rate Pixhawk IMU telemetry via HTTP json stream or local serial."""
    def __init__(self, http_url=None, serial_port=None, serial_baud=115200):
        self.http_url = http_url
        self.serial_port = serial_port
        self.serial_baud = serial_baud
        self.records = []
        self.latest = {
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": 0.0,
            "ax_m_s2": 0.0,
            "ay_m_s2": 0.0,
            "az_m_s2": 0.0,
            "gx_rad_s": 0.0,
            "gy_rad_s": 0.0,
            "gz_rad_s": 0.0,
            "connected": False
        }
        self.running = True
        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()

    def _worker_loop(self):
        # Priority 1: Direct Serial Port if provided
        if self.serial_port:
            try:
                import serial
                ser = serial.Serial(self.serial_port, self.serial_baud, timeout=0.1)
                while self.running:
                    b = ser.read(1)
                    if b == b'\xfe':  # MAVLink header
                        hdr = ser.read(5)
                        if len(hdr) == 5:
                            length, seq, sys_id, comp_id, msg_id = hdr
                            payload = ser.read(length)
                            checksum = ser.read(2)
                            if msg_id == 30 and len(payload) >= 28:
                                t_boot, r, p, y, rs, ps, ys = struct.unpack('<Iffffff', payload[:28])
                                now = time.time()
                                rec = {
                                    "timestamp_s": now,
                                    "time_boot_ms": t_boot,
                                    "roll_deg": math.degrees(r),
                                    "pitch_deg": math.degrees(p),
                                    "yaw_deg": math.degrees(y),
                                    "rollspeed_rad_s": rs,
                                    "pitchspeed_rad_s": ps,
                                    "yawspeed_rad_s": ys,
                                    "ax_m_s2": self.latest["ax_m_s2"],
                                    "ay_m_s2": self.latest["ay_m_s2"],
                                    "az_m_s2": self.latest["az_m_s2"],
                                    "gx_rad_s": self.latest["gx_rad_s"],
                                    "gy_rad_s": self.latest["gy_rad_s"],
                                    "gz_rad_s": self.latest["gz_rad_s"]
                                }
                                self.records.append(rec)
                                self.latest.update({
                                    "roll_deg": rec["roll_deg"],
                                    "pitch_deg": rec["pitch_deg"],
                                    "yaw_deg": rec["yaw_deg"],
                                    "connected": True
                                })
                            elif msg_id == 105 and len(payload) >= 32:
                                t_us, ax, ay, az, gx, gy, gz = struct.unpack('<Qffffff', payload[:32])
                                now = time.time()
                                rec = {
                                    "timestamp_s": now,
                                    "time_boot_ms": int(t_us // 1000),
                                    "roll_deg": self.latest["roll_deg"],
                                    "pitch_deg": self.latest["pitch_deg"],
                                    "yaw_deg": self.latest["yaw_deg"],
                                    "rollspeed_rad_s": gx,
                                    "pitchspeed_rad_s": gy,
                                    "yawspeed_rad_s": gz,
                                    "ax_m_s2": ax,
                                    "ay_m_s2": ay,
                                    "az_m_s2": az,
                                    "gx_rad_s": gx,
                                    "gy_rad_s": gy,
                                    "gz_rad_s": gz
                                }
                                self.records.append(rec)
                                self.latest.update({
                                    "ax_m_s2": ax,
                                    "ay_m_s2": ay,
                                    "az_m_s2": az,
                                    "gx_rad_s": gx,
                                    "gy_rad_s": gy,
                                    "gz_rad_s": gz,
                                    "connected": True
                                })
                ser.close()
                return
            except Exception as e:
                print(f"[IMU] Serial connection failed: {e}")

        # Priority 2: HTTP Polling from Pi's live_view server (/imu.json)
        if self.http_url:
            last_boot_ms = -1
            while self.running:
                try:
                    req = urllib.request.Request(self.http_url, headers={"User-Agent": "DRON-Recorder"})
                    with urllib.request.urlopen(req, timeout=0.1) as resp:
                        data = json.loads(resp.read().decode("utf-8"))
                        if data.get("connected"):
                            boot_ms = data.get("time_boot_ms", 0)
                            if boot_ms != last_boot_ms:
                                last_boot_ms = boot_ms
                                data["timestamp_s"] = time.time()
                                self.records.append(data)
                                self.latest = {
                                    "roll_deg": data.get("roll_deg", 0.0),
                                    "pitch_deg": data.get("pitch_deg", 0.0),
                                    "yaw_deg": data.get("yaw_deg", 0.0),
                                    "ax_m_s2": data.get("ax_m_s2", 0.0),
                                    "ay_m_s2": data.get("ay_m_s2", 0.0),
                                    "az_m_s2": data.get("az_m_s2", 0.0),
                                    "gx_rad_s": data.get("gx_rad_s", 0.0),
                                    "gy_rad_s": data.get("gy_rad_s", 0.0),
                                    "gz_rad_s": data.get("gz_rad_s", 0.0),
                                    "connected": True
                                }
                    time.sleep(0.01)  # Poll at up to 100 Hz
                except Exception:
                    time.sleep(0.05)

    def get_latest(self):
        return self.latest

    def save_csv(self, filepath):
        self.running = False
        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=0.5)

        with open(filepath, "w") as f:
            f.write("timestamp_s,time_boot_ms,roll_deg,pitch_deg,yaw_deg,rollspeed_rad_s,pitchspeed_rad_s,yawspeed_rad_s,ax_m_s2,ay_m_s2,az_m_s2,gx_rad_s,gy_rad_s,gz_rad_s\n")
            for r in self.records:
                f.write(
                    f"{r.get('timestamp_s', 0):.4f},"
                    f"{r.get('time_boot_ms', 0)},"
                    f"{r.get('roll_deg', 0.0):.2f},"
                    f"{r.get('pitch_deg', 0.0):.2f},"
                    f"{r.get('yaw_deg', 0.0):.2f},"
                    f"{r.get('rollspeed_rad_s', 0.0):.4f},"
                    f"{r.get('pitchspeed_rad_s', 0.0):.4f},"
                    f"{r.get('yawspeed_rad_s', 0.0):.4f},"
                    f"{r.get('ax_m_s2', 0.0):.4f},"
                    f"{r.get('ay_m_s2', 0.0):.4f},"
                    f"{r.get('az_m_s2', 0.0):.4f},"
                    f"{r.get('gx_rad_s', 0.0):.4f},"
                    f"{r.get('gy_rad_s', 0.0):.4f},"
                    f"{r.get('gz_rad_s', 0.0):.4f}\n"
                )
        return len(self.records)


def main():
    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except Exception:
            pass

    parser = argparse.ArgumentParser(description="Record live camera streams & IMU from Raspberry Pi")
    parser.add_argument("-s", "--seconds", type=float, default=10.0, help="Duration in seconds (default: 10.0)")
    parser.add_argument("-m", "--manual", action="store_true", default=False,
                        help="Manual start/stop mode: Press ENTER to start, and ENTER or Ctrl+C to stop")
    parser.add_argument("--host", default="raspberrypi.local", help="Host IP or hostname (default: raspberrypi.local)")
    parser.add_argument("--port", type=int, default=8000, help="HTTP Port (default: 8000)")
    parser.add_argument("--fps", type=float, default=25.0, help="Recording FPS (default: 25.0)")
    parser.add_argument("-o", "--out-dir", default=os.path.expanduser("~/Downloads/dron_recordings"),
                        help="Output directory for saved sessions")
    parser.add_argument("--side-by-side", action="store_true", default=True,
                        help="Also save a combined side-by-side stereo video (default: True)")
    parser.add_argument("--imu-port", default=None,
                        help="Optional direct serial port for Pixhawk on laptop (e.g. COM3 or /dev/ttyACM1)")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    # Stream URLs
    url0 = f"http://{args.host}:{args.port}/cam0.mjpg"
    url1 = f"http://{args.host}:{args.port}/cam1.mjpg"
    imu_url = f"http://{args.host}:{args.port}/imu.json"

    print(f"Connecting to streams at http://{args.host}:{args.port}...")
    worker0 = StreamWorker(url0, "cam0")
    worker1 = StreamWorker(url1, "cam1")
    imu_worker = IMUWorker(http_url=imu_url, serial_port=args.imu_port)

    # Wait for first frames
    print("Waiting for camera feeds to initialize...")
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
        if args.host == "raspberrypi.local":
            print("Trying fallback IP: 10.42.0.1...")
            worker0.release()
            worker1.release()
            args.host = "10.42.0.1"
            url0 = f"http://{args.host}:{args.port}/cam0.mjpg"
            url1 = f"http://{args.host}:{args.port}/cam1.mjpg"
            imu_url = f"http://{args.host}:{args.port}/imu.json"
            worker0 = StreamWorker(url0, "cam0")
            worker1 = StreamWorker(url1, "cam1")
            imu_worker = IMUWorker(http_url=imu_url, serial_port=args.imu_port)
            time.sleep(1.5)
            f0 = worker0.read_latest()
            f1 = worker1.read_latest()

    if f0 is None or f1 is None:
        worker0.release()
        worker1.release()
        imu_worker.running = False
        sys.exit(f"\n[ERROR] Could not connect to camera streams at http://{args.host}:{args.port}. Is live_view.py running on the Pi?")

    h, w = f0.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    session_dir = os.path.join(args.out_dir, f"session_{timestamp}")
    os.makedirs(session_dir, exist_ok=True)

    file_cam0 = os.path.join(session_dir, "cam0.mp4")
    file_cam1 = os.path.join(session_dir, "cam1.mp4")
    file_stereo = os.path.join(session_dir, "stereo.mp4")
    file_imu = os.path.join(session_dir, "imu.csv")
    file_frame_timestamps = os.path.join(session_dir, "frame_timestamps.csv")

    out0 = cv2.VideoWriter(file_cam0, fourcc, args.fps, (w, h))
    out1 = cv2.VideoWriter(file_cam1, fourcc, args.fps, (w, h))
    out_stereo = cv2.VideoWriter(file_stereo, fourcc, args.fps, (w * 2, h)) if args.side_by_side else None

    if args.manual:
        print("\n" + "=" * 70)
        print("🟢 CAMERA STREAMS & PIXHAWK IMU CONNECTED!")
        print("👉 Press [ENTER] to START recording...")
        print("=" * 70)
        try:
            input()
        except (KeyboardInterrupt, EOFError):
            worker0.release()
            worker1.release()
            imu_worker.running = False
            sys.exit(0)

    print(f"\n" + "=" * 70)
    if args.manual:
        print(f"🎬 RECORDING STARTED (MANUAL MODE @ {args.fps} FPS + Pixhawk IMU)")
        print("👉 Press [ENTER] or Ctrl+C at any time to STOP recording and save.")
    else:
        print(f"🎬 RECORDING STARTED: {args.seconds}s @ {args.fps} FPS + Pixhawk IMU")
    print(f"📁 Saving to: {session_dir}")
    print(f"=" * 70)

    stop_event = threading.Event()
    if args.manual:
        def wait_for_stop():
            try:
                input()
            except Exception:
                pass
            stop_event.set()
        threading.Thread(target=wait_for_stop, daemon=True).start()

    start_time = time.time()
    last_frame_time = start_time
    frame_interval = 1.0 / args.fps
    saved_frames = 0
    frame_log = []

    try:
        while True:
            now = time.time()
            elapsed = now - start_time
            if args.manual:
                if stop_event.is_set():
                    break
            else:
                if elapsed >= args.seconds:
                    break

            loop_start = now
            f0 = worker0.read_latest()
            f1 = worker1.read_latest()

            if f0 is not None and f1 is not None:
                out0.write(f0)
                out1.write(f1)
                if out_stereo:
                    sbs = np.hstack([f0, f1])
                    out_stereo.write(sbs)

                imu_info = imu_worker.get_latest()
                dt_frame = now - last_frame_time if saved_frames > 0 else 0.0
                last_frame_time = now

                frame_log.append({
                    "frame_index": saved_frames,
                    "timestamp_s": now,
                    "elapsed_s": elapsed,
                    "dt_s": dt_frame,
                    "time_boot_ms": imu_info.get("time_boot_ms", 0),
                    "roll_deg": imu_info.get("roll_deg", 0.0),
                    "pitch_deg": imu_info.get("pitch_deg", 0.0),
                    "yaw_deg": imu_info.get("yaw_deg", 0.0),
                    "ax_m_s2": imu_info.get("ax_m_s2", 0.0),
                    "ay_m_s2": imu_info.get("ay_m_s2", 0.0),
                    "az_m_s2": imu_info.get("az_m_s2", 0.0)
                })
                saved_frames += 1

            # Progress bar with live IMU attitude HUD
            imu_info = imu_worker.get_latest()
            if imu_info["connected"]:
                hud = f"IMU: R:{imu_info['roll_deg']:+5.1f}° P:{imu_info['pitch_deg']:+5.1f}° Y:{imu_info['yaw_deg']:5.1f}° | Az:{imu_info.get('az_m_s2', 0.0):+4.1f}m/s²"
            else:
                hud = "IMU: Waiting..."

            if args.manual:
                hud_line = f"\r[REC 🔴] {elapsed:5.1f}s | {saved_frames:3d} frames | {hud} | Press ENTER to STOP"
            else:
                progress = min(1.0, elapsed / args.seconds)
                bar_len = 20
                filled = int(bar_len * progress)
                bar = "█" * filled + "░" * (bar_len - filled)
                hud_line = f"\r[{bar}] {elapsed:4.1f}s / {args.seconds:.1f}s | {saved_frames:3d} frames | {hud}"

            sys.stdout.write(hud_line)
            sys.stdout.flush()

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
        imu_count = imu_worker.save_csv(file_imu)

        # Save per-frame timestamps
        with open(file_frame_timestamps, "w") as ftf:
            ftf.write("frame_index,timestamp_s,elapsed_s,dt_s,time_boot_ms,roll_deg,pitch_deg,yaw_deg,ax_m_s2,ay_m_s2,az_m_s2\n")
            for fl in frame_log:
                ftf.write(
                    f"{fl['frame_index']},"
                    f"{fl['timestamp_s']:.6f},"
                    f"{fl['elapsed_s']:.4f},"
                    f"{fl['dt_s']:.4f},"
                    f"{fl['time_boot_ms']},"
                    f"{fl['roll_deg']:.2f},"
                    f"{fl['pitch_deg']:.2f},"
                    f"{fl['yaw_deg']:.2f},"
                    f"{fl.get('ax_m_s2', 0.0):.3f},"
                    f"{fl.get('ay_m_s2', 0.0):.3f},"
                    f"{fl.get('az_m_s2', 0.0):.3f}\n"
                )

    end_time = time.time()

    # Save session metadata
    meta_path = os.path.join(session_dir, "metadata.json")
    try:
        with open(meta_path, "w") as mf:
            json.dump({
                "timestamp": timestamp,
                "start_epoch_time": start_time,
                "end_epoch_time": end_time,
                "duration_requested_s": args.seconds,
                "duration_actual_s": round(end_time - start_time, 3),
                "fps_target": args.fps,
                "fps_actual": round(saved_frames / max(1.0, end_time - start_time), 2),
                "frame_count": saved_frames,
                "resolution": {"width": w, "height": h},
                "host": args.host,
                "imu_samples_logged": imu_count,
                "imu_rate_hz": round(imu_count / max(1.0, end_time - start_time), 1),
                "frame_timestamps_file": "frame_timestamps.csv",
                "imu_telemetry_file": "imu.csv"
            }, mf, indent=2)
    except Exception:
        pass

    print(f"\n\n✅ RECORDING COMPLETE!")
    print(f"📁 Session Folder: {session_dir}")
    print(f"  ▶ Camera 0:             {file_cam0}")
    print(f"  ▶ Camera 1:             {file_cam1}")
    if args.side_by_side:
        print(f"  ▶ Side-by-Side Stereo:   {file_stereo}")
    print(f"  ▶ Per-Frame Timestamps: {file_frame_timestamps} ({len(frame_log)} frames)")
    print(f"  ▶ Pixhawk IMU CSV:      {file_imu} ({imu_count} samples)")
    print(f"  ▶ Session Metadata:     {meta_path}")


if __name__ == "__main__":
    main()
