# DRON Lab Hardware Integration & Sensor Verification Report

**Date:** 27 October 2026  
**Subteam:** 3D Visualization & Perception  
**Hardware Rig:** Drone 01 (Raspberry Pi 5 + Dual Sony IMX219 + Meridian Innovation MI48)

---

## 1. Executive Summary

During today's lab session, the visualization team achieved critical hardware bringup milestones on the physical drone platform:
1. **Hardware & Sensor Identification:** Inspected and documented the exact hardware platform: Raspberry Pi 5 Model B Rev 1.0, dual Sony IMX219 (Pi Camera v2) sensors on native CSI buses, and the Meridian Innovation MI48 radiometric thermal camera.
2. **Platform Recovery:** Successfully recovered a locked, read-only Linux root filesystem caused by 100% storage saturation, reclaiming over **7.0 GB of disk space**.
3. **Thermal Sensor Validation:** Successfully read real-time radiometric temperature arrays from the MI48 over USB serial, confirming raw Celsius values ($13.45^\circ\text{C}$ to $28.95^\circ\text{C}$).
4. **Live Low-Latency Streaming:** Implemented and launched an HTTP MJPEG dual-stack streaming server (`live_view.py`) on the Pi, streaming both stereo cameras simultaneously at 25 FPS to a laptop web browser.
5. **Automated Dataset Recording:** Developed a laptop-side recording pipeline (`record_stream.py`) that captures synchronized, timestamped stereo video sessions (`cam0.mp4`, `cam1.mp4`, `stereo.mp4`, `metadata.json`) directly to the laptop.

---

## 2. Power Architecture & Battery Operational Safety

### LiPo Flight Battery vs. Bench Power
* The drone's onboard compute (Raspberry Pi 5 + dual cameras + MI48) draws **10 to 15 Watts** under active capture.
* When powered via a 4S/6S LiPo battery through the drone's power distribution board (PDB) and 5V BEC:
  * **Voltage Cutoff:** An idle drone sitting on a lab bench with active cameras will drain a battery in 15–20 minutes. If the cell voltage drops below $3.3\text{ V/cell}$, the BEC shuts down or browns out the Pi, causing instant Wi-Fi loss and corrupting the ext4 filesystem.
  * Always monitor battery voltage with a digital LiPo checker buzzer during bench testing.

### ⚠️ Critical Warning on USB-C Laptop Connection
* **NEVER plug a laptop's USB-C cable directly into the Raspberry Pi while the drone flight battery is connected.**
  * Doing so connects two independent 5V power sources in parallel (the drone's switching BEC and the laptop's USB regulator), creating a ground loop / backfeed that can permanently destroy the laptop's USB controller or the Pi's power management IC (PMIC).
  * USB-C from a laptop should only be used on a bare Pi on the bench with the drone battery completely unplugged.

---

## 3. Architecture Separation: Raspberry Pi vs. PX4 Flight Controller

A key clarification established today:
* **PX4 Flight Controller (Pixhawk):** Controls flight dynamics, motors, gyroscopes, accelerometer, GPS, and offboard velocity/position setpoints. It runs an STM32 microcontroller and communicates via low-bandwidth MAVLink or `MicroXRCEAgent`. **It has no physical connection to video cameras or thermal sensors.**
* **Raspberry Pi 5 (Companion Computer):** Hosts the high-bandwidth perception stack. The dual CSI ribbon cables and the MI48 USB cable are wired directly to the Pi. All video capture, disparity, and 3D visualization data originate from the Pi.

```
       ┌───────────────────────────────┐
       │     PX4 Flight Controller     │ (Motors, IMU, GPS, Flight Telemetry)
       └──────────────┬────────────────┘
                      │ Micro-XRCE-DDS / Serial (Telemetry only)
       ┌──────────────▼────────────────┐
       │   Raspberry Pi 5 (Companion)  │ (Linux Debian 12 Bookworm)
       ├───────────────────────────────┤
       │  ├─ CAM0 (Left):  IMX219 CSI  │
       │  ├─ CAM1 (Right): IMX219 CSI  │
       │  └─ Thermal:      MI48 USB    │
       └──────────────┬────────────────┘
                      │ Wi-Fi Access Point (SSID: "dron")
       ┌──────────────▼────────────────┐
       │   Ground Laptop (Windows)     │ (Live View, Recorders, Unity Visualizer)
       └───────────────────────────────┘
```

---

## 4. Network Setup & Remote Access

1. **Drone Wi-Fi Access Point:**
   * **SSID:** `dron`
   * **Behavior:** When connecting, Windows indicates *"No internet, secured"*. This is **expected and normal**; the drone functions as an autonomous local robotics network, not an internet router.
2. **IP Addresses:**
   * **Raspberry Pi Gateway IP:** `10.42.0.1` (or link-local IPv6)
   * **Laptop IP:** Assigned dynamically via DHCP (e.g. `10.42.0.152`)
3. **SSH Commands:**
   ```powershell
   # Primary connection method (mDNS):
   ssh dron@raspberrypi.local

   # Direct IP fallback:
   ssh dron@10.42.0.1
   ```

---

## 5. Storage Recovery & System Maintenance

### Root Cause of Image Capture Failures
During initial capture tests, `rpicam-still` reported `ERROR: *** failed to open file ***` and system logs reported `(30: Read-only file system)`. 
* Running `df -h /` revealed the 32GB Micro-SD card had reached **100% capacity (only 104 MB free)**.
* In response to storage exhaustion, the Linux kernel remounted `/dev/mmcblk0p2` in read-only mode to prevent filesystem corruption.
* A storage audit (`du -h -d 2 /home/dron`) identified **14 GB of reclaimable files**:
  * `~/.local/share/Trash`: 7.0 GB of discarded files.
  * `~/Desktop/imgs`: 6.7 GB of legacy image dumps.

### Resolution Commands Executed:
```bash
# 1. Remount filesystem in read-write mode:
sudo mount -o remount,rw /
systemctl daemon-reload

# 2. Purge desktop trash bin:
rm -rf /home/dron/.local/share/Trash/*

# 3. Clean APT package archives:
sudo apt clean

# 4. Verify recovered capacity (reclaimed 7.1 GB free, 74% utilized):
df -h /
```

---

## 6. Sensor Verification Tests & Commands

### A. Stereo Camera Verification
* **Driver:** `libcamera` / `libpisp` v1.0.7 with RP1 peripheral controller.
* **Bus Routing:**
  * Camera 0: `/base/axi/pcie@120000/rp1/i2c@88000/imx219@10`
  * Camera 1: `/base/axi/pcie@120000/rp1/i2c@80000/imx219@10`
* **Test Commands Executed:**
  ```bash
  # Check camera detection:
  rpicam-hello --list-cameras

  # Test capture single frames:
  rpicam-still -t 1000 -n -o /home/dron/test_cam0.jpg --camera 0
  rpicam-still -t 1000 -n -o /home/dron/test_cam1.jpg --camera 1
  ```
* **Results:** Both sensors captured uncompressed frames (`1.5 MB` and `1.6 MB`) at resolutions up to $3280 \times 2464$.

### B. Thermal Camera Verification
* **Sensor:** Meridian Innovation MI48 on `/dev/ttyACM0` (`ID 0416:b020 Winbond USB Virtual COM`).
* **SDK:** `senxor` (located on Pi at `/home/dron/Desktop/pysenxor-master/`).
* **Crucial Driver Configuration:** The dual-interface API requires passing `MI48([usb, usb])` so the SDK uses interface 0 for registers and interface 1 for radiometric streaming.
* **Verification Script:**
  ```bash
  export PYTHONPATH="/home/dron/Desktop/pysenxor-master/build/lib:/home/dron/Desktop/pysenxor-master/example:$PYTHONPATH"
  python3 -c "
  import serial
  from senxor.mi48 import MI48
  from senxor.interfaces import USB_Interface
  ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.5)
  usb = USB_Interface(ser)
  mi48 = MI48([usb, usb])
  mi48.set_fps(9)
  mi48.start(stream=True, with_header=True)
  data, header = mi48.read()
  mi48.stop()
  print('Resolution:', data.shape, 'Min:', data.min(), 'C, Max:', data.max(), 'C')
  "
  ```
* **Results:** Successfully acquired a $62 \times 80$ array (4,960 pixels) reading room temperature ($22.44^\circ\text{C}$ average, $13.45^\circ\text{C}$ to $28.95^\circ\text{C}$).

---

## 7. Live Stream Server (`live_view.py`)

A custom multi-threaded Python streaming server was written and deployed to `/home/dron/live_view.py`.

### Features:
* **Dual-Stack Socket Binding:** Supports both IPv4 (`10.42.0.1`) and IPv6 (`raspberrypi.local`), preventing `ERR_CONNECTION_REFUSED` in modern browsers.
* **Graceful Degradation:** The `--no-thermal` flag allows running high-framerate stereo feeds without USB serial blocking.
* **Independent Capture Threads:** Runs Camera 0 and Camera 1 on decoupled `Picamera2` threads.

### Execution on Pi:
```bash
# Run stereo cameras at 25 FPS:
python3 /home/dron/live_view.py --no-thermal --fps 25
```
* **Laptop Browser Access:** Open `http://raspberrypi.local:8000` or `http://10.42.0.1:8000`.

---

## 8. Laptop-Side Recording Pipeline (`record_stream.py`)

To eliminate the need for manual `scp` commands after every flight, a laptop-side recorder was created at:
[`tools/visualization/record_stream.py`](file:///C:/Users/param/OneDrive/Documents/Code/Projects/DRON/tools/visualization/record_stream.py)

### Features:
* **Direct Network Ingestion:** Connects directly to the Pi's MJPEG streams over Wi-Fi.
* **Organized Directory Structure:** Every recording automatically creates a timestamped folder:
  ```text
  Downloads/dron_recordings/
  └── session_20261027_173720/
      ├── cam0.mp4            (Left camera video)
      ├── cam1.mp4            (Right camera video)
      ├── stereo.mp4          (Side-by-side synchronized stereo)
      └── metadata.json       (FPS, duration, resolution, frame counts)
  ```
* **Non-Blocking Reader:** Background thread consumes frames continuously to ensure zero frame buildup or buffer latency.

### How to Execute from Laptop (PowerShell):
```powershell
# Record 10 seconds of 25 FPS stereo video:
python "C:\Users\param\OneDrive\Documents\Code\Projects\DRON\tools\visualization\record_stream.py" -s 10

# Record 30 seconds:
python "C:\Users\param\OneDrive\Documents\Code\Projects\DRON\tools\visualization\record_stream.py" -s 30
```
* **Today's Test Run:** Successfully recorded **248 synchronized stereo frames** across multiple sessions with verified `.mp4` playback on Windows.

---

## 9. Next Steps & Technical Roadmap

### 1. Autonomy Team Coordination (Flight Controller IMU Data)
* **Objective:** Extract synchronized IMU telemetry (angular velocity, linear acceleration, estimated orientation) from the PX4 flight controller.
* **Integration Strategy:**
  * PX4 publishes `vehicle_odometry` and `sensor_combined` over the Micro-XRCE-DDS client on the drone.
  * Connect a ROS 2 node on the ground station to subscribe to PX4 odometry topics.
  * **Role in Visualization:** While pure Stereo Visual SLAM does not strictly require an IMU to build a metric map, fusing PX4's IMU data:
    1. Prevents tracking loss during fast yaw maneuvers.
    2. Maintains a true gravity vector so the floor plane remains level.
    3. Bridges perception gaps when pointing at low-texture walls.

### 2. Algorithmic Depth & Reconstruction Experiments (On Recorded Videos)
Using the video datasets recorded today, conduct offline benchmarking on the ground workstation:
* **Classical Stereo Disparity (SGBM + WLS):**
  * Run OpenCV `StereoSGBM` with weighted least squares (WLS) filtering on extracted image pairs.
  * Evaluate metric accuracy against known lab dimensions.
* **RTAB-Map Stereo Visual Odometry:**
  * Feed `stereo_YYYYMMDD_HHMMSS.mp4` into the RTAB-Map stereo odometry pipeline to test trajectory stability.
* **3D Gaussian Splatting & Neural Surface Reconstruction:**
  * Sample keyframes from `cam0.mp4` and `cam1.mp4` with COLMAP / Glomap pose estimation.
  * Train a 3D Gaussian Splatting model (e.g. `nerfstudio` / `gsplat`) to evaluate photorealistic scene reconstruction from drone stereo passes.
* **Thermal Texture Projection:**
  * Overlay MI48 radiometric temperature grids onto the reconstructed 3D point cloud using the un-rectification matrix ($R_1^T$) and rigid extrinsics ($R_{th \leftarrow L}, T_{th \leftarrow L}$).

### 3. Electrical & Mechanical Flight Stack Packaging
* **BEC Current Budgeting:** Ensure the 5V BEC powering the Pi 5 from the LiPo can supply at least **5.0A continuous** to prevent brownout reboots when the dual cameras, thermal sensor, and Wi-Fi radio peak simultaneously.
* **Thermal Dissipation:** Verify that the Pi 5 active cooler fan operates properly inside the drone canopy during flight to avoid thermal throttling under heavy CPU load.
