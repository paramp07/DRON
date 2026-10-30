# DRON Visualization Subteam: Kimera-Aligned Lab Action Plan

## 1. Executive Summary & Architecture Shift

Based on our benchmarking of open-source aerial robotics systems, the DRON Visualization Subteam is aligning its development roadmap with the architecture of **MIT Kimera** (*MIT SPARK Lab*). 

### Why the Kimera Architecture?
1. **Inherently Eliminates Walking Shake in 3D**: Kimera-VIO tightly couples stereo camera disparity with high-rate (200 Hz) IMU measurements from the flight controller. Camera motion, hand tremors, and footstep bobbing are tracked directly on the SE(3) manifold, producing a rock-solid, drift-free 3D world trajectory. **Pre-stabilizing video in 2D is not needed and is intentionally avoided.**
2. **Solves the Overlapping Point Bug**: By estimating 6-DOF camera poses across time, depth points are accumulated continuously into a global world coordinate frame instead of dumping every frame at `(0, 0, 0)`.
3. **Converts Floating Dots into Solid Meshes**: Rather than sparse, flickering point clouds, Kimera fuses depth frames into a Truncated Signed Distance Field (TSDF) voxel grid, creating watertight 3D walls, floors, and surfaces.
4. **Native Radiometric Projection (Kimera-Semantics)**: The MI48 $62 \times 80$ thermal array is raycast directly onto the 3D voxel mesh surface, painting raw Celsius temperatures onto physical structural geometry.

```
                           [ DRON Physical Hardware ]
        Dual Sony IMX219 (CSI)         Pixhawk 6C IMU         Meridian MI48 Thermal
             (25 FPS, Pi 5)             (200 Hz, PX4)             (9 FPS, USB)
                   │                          │                         │
                   └────────────┬─────────────┘                         │
                                ▼                                       │
                     ┌──────────────────────┐                           │
                     │    1. Kimera-VIO     │                           │
                     │  (Stereo + IMU Pose) │                           │
                     └──────────┬───────────┘                           │
                                ▼                                       │
                     ┌──────────────────────┐                           │
                     │    2. Kimera-RPGO    │                           │
                     │ (Loop Closure Graph) │                           │
                     └──────────┬───────────┘                           │
                                ▼                                       │
                      Optimized Drone Poses                             │
                                │                                       │
                                ▼                                       ▼
                     ┌──────────────────────────────────────────────────────┐
                     │          3. Kimera-Semantics (TSDF Fusion)           │
                     │       (Voxel Mesh + Radiometric Raycasting)          │
                     └──────────────────────┬───────────────────────────────┘
                                            ▼
                        ┌───────────────────────────────────────┐
                        │    4. Unified Metric-Thermal Model    │
                        │ (Streamed to Unity at 60 FPS for GUI) │
                        └───────────────────────────────────────┘
```

---

## 2. Lab Meeting Goals & Stations

### 🎯 Goal 1: IMU-Stereo Telemetry Handshake (Kimera-VIO Input)
* **Objective**: Simultaneously log stereo video frames (25 FPS) and Pixhawk flight telemetry (200 Hz) with synchronized microsecond timestamps.
* **Tasks**:
  1. Have the Autonomy team launch `MicroXRCEAgent` over serial/UDP on the Pi:
     ```bash
     MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
     ```
  2. Confirm on ground laptop that PX4 telemetry is actively streaming over ROS 2:
     ```bash
     ros2 topic echo /fmu/out/vehicle_attitude
     ros2 topic echo /fmu/out/sensor_combined
     ```
  3. Verify system clock synchronization between Raspberry Pi 5 (`dron@10.42.0.1`) and laptop ground station.
  4. Lock Pi camera exposure to $2\text{ ms}$ ($1/500\text{s}$) in `live_view.py` to prevent walking motion blur while maintaining high contrast.

---

### 🎯 Goal 2: Rig Extrinsics & Geometric Calibration Matrix
* **Objective**: Measure and calibrate the rigid transformation tree ($T_{\text{IMU}}^{\text{cam}}$, $T_{\text{cam}}^{\text{thermal}}$).
* **Tasks**:
  1. **Measure Physical Sensor Baselines** with digital calipers:
     * Stereo baseline $B = \|T_{\text{cam0} \to \text{cam1}}\|$ (nominal: $44.0\text{ mm}$).
     * Thermal-to-Left-Camera translation vector $[T_x, T_y, T_z]$ in millimeters.
     * Camera-rig-to-Pixhawk-center translation vector $[T_x, T_y, T_z]$.
  2. **Checkerboard Recording**:
     * Hold an $8 \times 6$ checkerboard at distances of 0.5m, 1.0m, and 1.5m.
     * Record a 20-second sweep visible to Cam 0, Cam 1, and the MI48 thermal camera simultaneously.
  3. Store calibrated extrinsics into `config/visualization/rigs/drone_01/extrinsics.yaml`.

---

### 🎯 Goal 3: Controlled "Thermal Target" 360° Walkthrough
* **Objective**: Collect a benchmark multimodal dataset containing stereo images, thermal arrays, and IMU data centered on a warm object.
* **Tasks**:
  1. **Hot Target Placement**: Place an active laptop power brick (~$45^\circ\text{C}$) or hot mug on a table in the center of the lab.
  2. **Controlled Trajectory**: Walk the drone in a smooth 360° circle around the table at an approximate radius of 1.5–2.0 meters.
  3. **Data Recording**:
     * `cam0.mp4` & `cam1.mp4` (25 FPS, synchronized)
     * `thermal.csv` / raw array dumps ($62 \times 80$ float32 in Celsius at ~9 FPS)
     * `imu_telemetry.csv` (PX4 vehicle attitude quaternion + angular velocities at 200 Hz)

---

### 🎯 Goal 4: Immediate In-Lab Sanity Check
* **Objective**: Confirm data integrity before leaving the lab space.
* **Tasks**:
  1. Inspect recorded frames for frame drops or out-of-order timestamps.
  2. Run single-frame projection test using `tools/visualization/thermal_projector.py` to verify that the hot target appears red/yellow on the depth surface.

---

## 3. Subteam Coordination Agenda (10-Minute Standup)

Bring these specific questions to the cross-subteam standup:

| Subteam | Agenda Topic | Goal |
|---|---|---|
| **Autonomy Subteam** | `MicroXRCEAgent` Topic Agreement | Confirm that `/fmu/out/vehicle_attitude` and `/fmu/out/sensor_combined` will remain active during manual/carrying tests (even when disarmed). |
| **Autonomy Subteam** | Clock Synchronization | Agree on whether NTP or ROS 2 time will serve as the master clock between the Pi 5 and the ground station. |
| **Hardware / Airframe** | Rigidity & Damping | Verify that the 3D-printed dual-camera bracket has zero physical flex relative to the drone frame, and discuss rubber grommets for motor vibration isolation. |
| **Faculty / Team Leads** | Architecture Sign-off | Formally present the Kimera metric-semantic pipeline as the visualization subteam's core framework. |

---

## 4. Hardware & Power Safety Rules (Non-Negotiable)

1. ⚠️ **NEVER connect laptop USB-C power to the Pi 5 while the LiPo drone battery is plugged in.** This forms an unfused 5V ground loop between the drone BEC and your laptop motherboard.
2. ⚠️ **Keep propellers OFF the drone** during all indoor bench tests, carrying walkthroughs, and sensor calibration runs.
3. ⚠️ **Monitor LiPo cell voltage**; do not let 4S cells drop below 3.5V per cell during prolonged sensor streaming.

---

## 5. Post-Lab Deliverables (Ground Processing)

Once the lab dataset is secured, ground station software will execute:
1. **Pose Trajectory Extraction**: Generate continuous 6-DOF camera trajectory from stereo + IMU.
2. **TSDF Voxel Fusion**: Accumulate consecutive stereo depth frames into a continuous 3D surface mesh.
3. **Radiometric Texture Projection**: Paint the MI48 temperatures onto the mesh vertices.
4. **Unity Integration**: Feed the resulting `ThermalChunk` data directly into `ThermalCloudRenderer.cs`.
