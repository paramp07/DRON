#!/usr/bin/env python3
"""
DRON Multi-Camera + Thermal + IMU Live View Server
Runs on the Raspberry Pi 5. Decouples capture loops from HTTP streaming.

Serves:
  - http://<ip>:8000/            (Live HUD Dashboard)
  - http://<ip>:8000/cam0.mjpg   (Left Camera MJPEG)
  - http://<ip>:8000/cam1.mjpg   (Right Camera MJPEG)
  - http://<ip>:8000/thermal.mjpg(MI48 Thermal MJPEG)
  - http://<ip>:8000/imu.json    (Latest IMU Telemetry in JSON)

Usage on Pi:
  python3 live_view.py                          # Stereo + Thermal + IMU
  python3 live_view.py --no-thermal             # Stereo + IMU
  python3 live_view.py --fps 25 --size 640x480
"""

import argparse
import json
import logging
import math
import os
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

logging.basicConfig(level=logging.WARNING)

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit("OpenCV or NumPy missing. Install with: sudo apt install -y python3-opencv python3-numpy")


PAGE = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>DRON Perception & Telemetry Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { margin: 0; background: #0f172a; color: #f8fafc; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    header { background: #1e293b; padding: 12px 20px; display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid #334155; }
    h1 { font-size: 18px; margin: 0; font-weight: 600; letter-spacing: 0.5px; }
    .badge { background: #0284c7; padding: 4px 10px; border-radius: 9999px; font-size: 12px; font-weight: 500; }
    .hud { background: #1e293b; margin: 16px 20px 0; padding: 14px 20px; border-radius: 8px; border: 1px solid #334155; display: flex; gap: 30px; align-items: center; }
    .hud-item { display: flex; flex-direction: column; }
    .hud-label { font-size: 11px; text-transform: uppercase; color: #94a3b8; font-weight: 600; letter-spacing: 0.5px; }
    .hud-val { font-size: 20px; font-weight: 700; font-family: monospace; color: #38bdf8; margin-top: 2px; }
    .grid { display: flex; flex-wrap: wrap; gap: 16px; padding: 20px; }
    .card { flex: 1 1 340px; background: #1e293b; border-radius: 8px; overflow: hidden; border: 1px solid #334155; }
    .card h3 { margin: 0; padding: 10px 14px; font-size: 13px; background: #334155; color: #e2e8f0; font-weight: 500; }
    .card img { width: 100%; display: block; background: #000; min-height: 240px; object-fit: contain; }
  </style>
  <script>
    setInterval(async () => {
      try {
        const res = await fetch('/imu.json');
        const data = await res.json();
        if (data.connected) {
          document.getElementById('r-val').innerText = data.roll_deg.toFixed(1) + '°';
          document.getElementById('p-val').innerText = data.pitch_deg.toFixed(1) + '°';
          document.getElementById('y-val').innerText = data.yaw_deg.toFixed(1) + '°';
          if (data.ax_m_s2 !== undefined) {
            document.getElementById('acc-val').innerText = data.ax_m_s2.toFixed(2) + ', ' + data.ay_m_s2.toFixed(2) + ', ' + data.az_m_s2.toFixed(2) + ' m/s²';
          }
          document.getElementById('imu-status').innerText = 'PIXHAWK CONNECTED (' + data.source + ')';
          document.getElementById('imu-status').style.background = '#10b981';
        } else {
          document.getElementById('imu-status').innerText = 'IMU DISCONNECTED';
          document.getElementById('imu-status').style.background = '#ef4444';
        }
      } catch (e) {}
    }, 100);
  </script>
</head>
<body>
  <header>
    <h1>🛸 DRON Autonomous Perception Suite</h1>
    <span id="imu-status" class="badge">PROBING IMU...</span>
  </header>
  <div class="hud">
    <div class="hud-item"><span class="hud-label">Roll (Bank)</span><span id="r-val" class="hud-val">0.0°</span></div>
    <div class="hud-item"><span class="hud-label">Pitch (Elevation)</span><span id="p-val" class="hud-val">0.0°</span></div>
    <div class="hud-item"><span class="hud-label">Yaw (Heading)</span><span id="y-val" class="hud-val">0.0°</span></div>
    <div class="hud-item"><span class="hud-label">Accel (X/Y/Z)</span><span id="acc-val" class="hud-val">0.0, 0.0, 9.8 m/s²</span></div>
  </div>
  <div class="grid">__PANES__</div>
</body>
</html>
"""


class LatestFrame:
    """Thread-safe single-frame buffer for video streams."""
    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = 0

    def put(self, jpeg):
        with self.cond:
            self.jpeg = jpeg
            self.seq += 1
            self.cond.notify_all()

    def wait_next(self, last_seq, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout)
            return self.jpeg, self.seq


class IMUState:
    """Thread-safe store for Pixhawk IMU telemetry."""
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            "connected": False,
            "source": "None",
            "time_boot_ms": 0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": 0.0,
            "rollspeed_rad_s": 0.0,
            "pitchspeed_rad_s": 0.0,
            "yawspeed_rad_s": 0.0,
            "ax_m_s2": 0.0,
            "ay_m_s2": 0.0,
            "az_m_s2": 0.0,
            "gx_rad_s": 0.0,
            "gy_rad_s": 0.0,
            "gz_rad_s": 0.0,
            "timestamp_s": 0.0,
            "sample_count": 0
        }

    def update_attitude(self, boot_ms, roll, pitch, yaw, r_spd, p_spd, y_spd, source):
        with self.lock:
            self.data["connected"] = True
            self.data["source"] = source
            self.data["time_boot_ms"] = boot_ms
            self.data["roll_deg"] = math.degrees(roll)
            self.data["pitch_deg"] = math.degrees(pitch)
            self.data["yaw_deg"] = math.degrees(yaw)
            self.data["rollspeed_rad_s"] = r_spd
            self.data["pitchspeed_rad_s"] = p_spd
            self.data["yawspeed_rad_s"] = y_spd
            self.data["timestamp_s"] = time.time()
            self.data["sample_count"] += 1

    def update_highres_imu(self, time_us, ax, ay, az, gx, gy, gz, source):
        with self.lock:
            self.data["connected"] = True
            self.data["source"] = source
            self.data["time_boot_ms"] = int(time_us // 1000)
            self.data["ax_m_s2"] = ax
            self.data["ay_m_s2"] = ay
            self.data["az_m_s2"] = az
            self.data["gx_rad_s"] = gx
            self.data["gy_rad_s"] = gy
            self.data["gz_rad_s"] = gz
            self.data["timestamp_s"] = time.time()
            self.data["sample_count"] += 1

    def update(self, boot_ms, roll, pitch, yaw, r_spd, p_spd, y_spd, source):
        self.update_attitude(boot_ms, roll, pitch, yaw, r_spd, p_spd, y_spd, source)

    def get_dict(self):
        with self.lock:
            return dict(self.data)


def imu_reader_loop(imu_state, stop_event):
    """Background thread reading MAVLink attitude & IMU telemetry from Pixhawk."""
    candidate_ports = [
        "/dev/ttyACM1",
        "/dev/ttyACM0",
        "/dev/ttyAMA0",
        "/dev/serial0"
    ]
    candidate_bauds = [115200, 921600, 57600]

    try:
        import serial
    except ImportError:
        print("[IMU] Warning: pyserial not installed. Skipping IMU thread.")
        return

    ser = None
    active_port = None
    active_baud = None

    while not stop_event.is_set():
        if ser is None:
            for port in candidate_ports:
                if not os.path.exists(port):
                    continue
                for baud in candidate_bauds:
                    try:
                        test_ser = serial.Serial(port, baud, timeout=0.5)
                        t0 = time.time()
                        while time.time() - t0 < 0.8:
                            b = test_ser.read(1)
                            if b == b'\xfe':
                                ser = test_ser
                                active_port = port
                                active_baud = baud
                                print(f"[IMU] Successfully locked to Pixhawk on {port} @ {baud} baud!")
                                break
                        if ser is not None:
                            break
                        test_ser.close()
                    except Exception:
                        pass
                if ser is not None:
                    break
            if ser is None:
                time.sleep(2.0)
                continue

        # Continuous read loop
        try:
            byte = ser.read(1)
            if byte == b'\xfe':  # MAVLink v1 framing byte
                hdr = ser.read(5)
                if len(hdr) == 5:
                    length, seq, sys_id, comp_id, msg_id = hdr
                    payload = ser.read(length)
                    checksum = ser.read(2)
                    if msg_id == 30 and len(payload) >= 28:  # ATTITUDE
                        t_boot, roll, pitch, yaw, r_spd, p_spd, y_spd = struct.unpack('<Iffffff', payload[:28])
                        imu_state.update_attitude(t_boot, roll, pitch, yaw, r_spd, p_spd, y_spd, active_port)
                    elif msg_id == 105 and len(payload) >= 32:  # HIGHRES_IMU (time_usec + ax,ay,az,gx,gy,gz)
                        t_us, ax, ay, az, gx, gy, gz = struct.unpack('<Qffffff', payload[:32])
                        imu_state.update_highres_imu(t_us, ax, ay, az, gx, gy, gz, active_port)
        except Exception:
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(1.0)


def camera_loop(cam_num, size, fps, out_stream, quality, stop_event):
    """Decoupled Picamera2 capture loop."""
    try:
        from picamera2 import Picamera2
        cam = Picamera2(cam_num)
        config = cam.create_video_configuration(main={"size": size, "format": "RGB888"})
        cam.configure(config)
        cam.start()
        print(f"[Camera {cam_num}] Started ({size[0]}x{size[1]} @ {fps}fps)")
    except Exception as e:
        print(f"[Camera {cam_num}] Failed to initialize: {e}")
        return

    interval = 1.0 / max(1, fps)
    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]

    while not stop_event.is_set():
        t0 = time.time()
        try:
            rgb = cam.capture_array("main")
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            ok, jpeg = cv2.imencode(".jpg", bgr, encode_params)
            if ok:
                out_stream.put(jpeg.tobytes())
        except Exception as e:
            time.sleep(0.05)
            continue

        dt = time.time() - t0
        if dt < interval:
            time.sleep(interval - dt)

    try:
        cam.stop()
        cam.close()
    except Exception:
        pass


def thermal_loop(port, out_stream, stop_event):
    """Decoupled SenXor MI48 radiometric readout loop."""
    try:
        sys.path.extend([
            "/home/dron/Desktop/pysenxor-master/build/lib",
            "/home/dron/Desktop/pysenxor-master/example",
            "/home/dron/Desktop/pysenxor-master",
            "/home/dron"
        ])
        from senxor.interfaces import USB_Interface
        from senxor.mi48 import MI48

        usb = USB_Interface(port)
        mi48 = MI48([usb, usb])
        mi48.set_fps(9)
        if hasattr(mi48, "disable_low_netd"):
            mi48.disable_low_netd()
        mi48.start(stream=True, with_header=True)
        print("[Thermal] Started MI48 on USB Virtual COM @ 9Hz")
    except Exception as e:
        print(f"[Thermal] MI48 init failed: {e}")
        return

    while not stop_event.is_set():
        try:
            data, _ = mi48.read()
            if data is None:
                time.sleep(0.01)
                continue
            frame = np.asarray(data, dtype=np.float32).reshape((62, 80))
            f_norm = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            colored = cv2.applyColorMap(f_norm, cv2.COLORMAP_INFERNO)
            colored_up = cv2.resize(colored, (320, 248), interpolation=cv2.INTER_CUBIC)
            ok, jpeg = cv2.imencode(".jpg", colored_up)
            if ok:
                out_stream.put(jpeg.tobytes())
        except Exception:
            time.sleep(0.02)

    try:
        mi48.stop()
    except Exception:
        pass


def make_handler(streams, labels, imu_state):
    class DualStackHandler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # Silence spammy HTTP request logging

        def do_GET(self):
            path = self.path.split("?")[0].rstrip("/")

            if path in ("", "/index.html"):
                panes = "".join(
                    f'<div class="card"><h3>{labels[k]}</h3><img src="/{k}.mjpg"></div>'
                    for k in streams
                )
                html = PAGE.replace("__PANES__", panes).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(html)))
                self.end_headers()
                self.wfile.write(html)
                return

            if path == "/imu.json":
                data = json.dumps(imu_state.get_dict()).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(data)
                return

            name = path.lstrip("/")
            if name.endswith(".mjpg"):
                name = name[:-5]

            if name not in streams:
                self.send_error(404, "Stream Not Found")
                return

            stream = streams[name]
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=--frame")
            self.end_headers()

            seq = -1
            while True:
                jpeg, seq = stream.wait_next(seq, timeout=2.0)
                if jpeg is None:
                    continue
                try:
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(jpeg)).encode()
                        + b"\r\n\r\n"
                        + jpeg
                        + b"\r\n"
                    )
                except (BrokenPipeError, ConnectionResetError):
                    break

    return DualStackHandler


class DualStackServer(ThreadingHTTPServer):
    address_family = socket.AF_INET6

    def server_bind(self):
        try:
            self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        except (AttributeError, OSError):
            pass
        super().server_bind()


def main():
    parser = argparse.ArgumentParser(description="DRON Live Streaming & Telemetry Server")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--size", default="640x480")
    parser.add_argument("--fps", type=int, default=25)
    parser.add_argument("--quality", type=int, default=80)
    parser.add_argument("--no-thermal", action="store_true")
    parser.add_argument("--thermal-port", default="/dev/ttyACM0")
    args = parser.parse_args()

    size = tuple(int(v) for v in args.size.lower().split("x"))
    stop_event = threading.Event()
    streams = {}
    labels = {}
    threads = []
    imu_state = IMUState()

    # Launch IMU thread
    imu_thread = threading.Thread(target=imu_reader_loop, args=(imu_state, stop_event), daemon=True)
    imu_thread.start()

    # Launch Stereo Cameras
    try:
        from picamera2 import Picamera2
        cams = Picamera2.global_camera_info()
        for info in cams[:2]:
            i = int(info.get("Num", 0))
            streams[f"cam{i}"] = LatestFrame()
            labels[f"cam{i}"] = f"Camera {i} ({info.get('Model', 'IMX219')})"
            threads.append(threading.Thread(
                target=camera_loop,
                args=(i, size, args.fps, streams[f"cam{i}"], args.quality, stop_event),
                daemon=True
            ))
    except Exception as e:
        print(f"[Cameras] Warning: {e}")

    # Launch Thermal
    if not args.no_thermal:
        streams["thermal"] = LatestFrame()
        labels["thermal"] = "Thermal MI48 (Celsius)"
        threads.append(threading.Thread(
            target=thermal_loop,
            args=(args.thermal_port, streams["thermal"], stop_event),
            daemon=True
        ))

    for t in threads:
        t.start()

    server = DualStackServer(("::", args.port), make_handler(streams, labels, imu_state))
    print(f"\n🚀 DRON Server Live: Open http://raspberrypi.local:{args.port} or http://10.42.0.1:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.server_close()


if __name__ == "__main__":
    main()
