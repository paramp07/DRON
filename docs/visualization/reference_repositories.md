# Open-Source 3D Reconstruction & SLAM Reference Repositories

This document curates the primary open-source frameworks, algorithms, and toolkits benchmarked for the **DRON** autonomous drone platform (Raspberry Pi 5, dual Sony IMX219 stereo cameras, Meridian Innovation MI48 radiometric thermal sensor, and Pixhawk 6C flight controller).

---

## 1. Core Architecture & Volumetric Meshing (The Primary Blueprint)

### 🌟 **[MIT-SPARK/Kimera](https://github.com/MIT-SPARK/Kimera)** *(MIT SPARK Lab)*
* **Focus**: Real-time metric-semantic SLAM system designed for aerial robots.
* **Architecture Role for DRON**:
  * Serves as the primary system architecture for the visualization subteam.
  * Solves the single-point dumping issue by transforming stereo depth into a globally tracked SE(3) world trajectory.
  * **Kimera-Semantics** provides the exact mathematical framework needed to raycast and project the MI48 ($62 \times 80$) radiometric thermal matrix onto 3D voxel surfaces.
* **Sub-modules**:
  * **[Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO)**: Visual-Inertial Odometry frontend/backend using GTSAM factor graphs.
  * **[Kimera-RPGO](https://github.com/MIT-SPARK/Kimera-RPGO)**: Robust Pose Graph Optimization with outlier rejection.
  * **[Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics)**: Volumetric TSDF voxel fusion and semantic/thermal surface mesh generation.

---

### 🌟 **[ethz-asl/voxblox](https://github.com/ethz-asl/voxblox)** *(ETH Zurich Autonomous Systems Lab)*
* **Focus**: Continuous Truncated Signed Distance Field (TSDF) voxel mapping library optimized for Micro Aerial Vehicles (MAVs).
* **Architecture Role for DRON**:
  * Converts sparse, noisy stereo point clouds into smooth, watertight 3D triangle meshes.
  * Eliminates sensor flicker across frames through continuous ray-bundle depth integration.
  * Exports clean `.ply` / `.obj` meshes for 60 FPS rendering in the Unity Ground Control Interface.

---

## 2. Drone / UAV Visual-Inertial SLAM Frameworks

### 🛸 **[HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)** *(HKUST Aerial Robotics)*
* **Focus**: Optimization-based multi-sensor visual-inertial state estimator for autonomous quadrotors.
* **Architecture Role for DRON**:
  * Battle-tested under aggressive flight dynamics, rapid pitch/roll changes, and motor vibrations.
  * Tightly couples stereo camera streams with the Pixhawk's 200 Hz IMU telemetry.
  * Highly optimized C++ implementation suitable for companion computers like the Raspberry Pi 5.

---

### 🛸 **[laxnpander/OpenREALM](https://github.com/laxnpander/OpenREALM)**
* **Focus**: Real-time aerial mapping and 3D reconstruction framework specifically built for UAVs.
* **Architecture Role for DRON**:
  * Tailored for airborne camera perspectives (forward- and downward-facing stereo rigs).
  * Generates 2.5D digital elevation models (DEM) and dense 3D point clouds on the fly.
  * Includes integrated loop closure to eliminate spatial drift during room laps.

---

### 🛸 **[uzh-rpg/rpg_svo_pro_open](https://github.com/uzh-rpg/rpg_svo_pro_open)** *(University of Zurich RPG)*
* **Focus**: Semi-direct Visual Odometry (SVO 2.0).
* **Architecture Role for DRON**:
  * High-speed direct visual odometry operating at 100+ FPS on embedded hardware.
  * Bypasses costly feature extraction by operating directly on pixel photometric intensities.

---

## 3. Dense 3D Point Cloud Engines & Python Processing

### 🔧 **[introlab/rtabmap](https://github.com/introlab/rtabmap)** & **[rtabmap_ros](https://github.com/introlab/rtabmap_ros)**
* **Focus**: Real-Time Appearance-Based Mapping — the industry-standard visual SLAM library for ROS and ROS 2.
* **Architecture Role for DRON**:
  * Native stereo camera frontend (`rtabmap_odom`).
  * Features long-term memory management (transfers distant keyframes to disk to prevent RAM exhaustion on ground station).
  * Direct ROS 2 bindings for stereo depth-to-point-cloud streaming.

---

### 🔧 **[isl-org/Open3D](https://github.com/isl-org/Open3D)**
* **Focus**: Fast C++ and Python library for 3D data manipulation.
* **Architecture Role for DRON**:
  * Rapid post-processing and filtering of `.ply` point cloud files on the laptop ground station:
    * **Statistical Outlier Removal (SOR)**: Removes floating stereo noise artifacts.
    * **Voxel Downsampling**: Reduces point density for real-time rendering performance.
    * **Iterative Closest Point (ICP)**: Registers and aligns multi-run point clouds.

---

### 🔧 **[umass-amrl/stereo_dense_reconstruction](https://github.com/umass-amrl/stereo_dense_reconstruction)**
* **Focus**: Minimal, high-speed ROS node converting synchronized stereo camera pairs into dense `sensor_msgs/PointCloud2`.
* **Architecture Role for DRON**:
  * Clean, minimal reference code for real-time OpenCV StereoSGBM matching with zero academic bloat.

---

## 4. Multi-Sensor Rig & IMU Calibration

### 📐 **[ethz-asl/kalibr](https://github.com/ethz-asl/kalibr)** *(ETH Zurich)*
* **Focus**: The gold-standard multi-camera and IMU spatial/temporal calibration toolbox.
* **Architecture Role for DRON**:
  * Computes sub-millimeter camera intrinsics ($f_x, f_y, c_x, c_y$) and distortion parameters.
  * Determines the spatial transformation between the stereo camera baseline and the Pixhawk IMU center of gravity ($T_{\text{cam}}^{\text{IMU}}$).
  * Measures temporal clock delays between camera exposures and IMU measurement packets.

---

## 5. Multimodal Thermal-Visual 3D Research

### 🔥 **[uzh-rpg](https://github.com/uzh-rpg)** *(UZH Robotics & Perception Group)*
* **Focus**: Thermal-inertial SLAM and multi-spectral 3D mapping research on quadrotors for search-and-rescue.
* **Architecture Role for DRON**:
  * Reference mathematical formulations for projecting low-resolution radiometric thermal arrays (like the MI48) onto high-resolution visible stereo 3D structures.

---

## 6. Project Benchmark Mapping Matrix

| Problem / Capability | Recommended Repository | Application to DRON Pipeline |
|---|---|---|
| **Core Architecture & Thermal Projection** | **[MIT-SPARK/Kimera](https://github.com/MIT-SPARK/Kimera)** | Blueprint for VIO + TSDF Mesh + Thermal raycasting |
| **Solid Voxel Mesh Reconstruction** | **[ethz-asl/voxblox](https://github.com/ethz-asl/voxblox)** | Converts noisy depth into watertight walls & floors |
| **Aerial Stereo + IMU State Estimation** | **[HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)** | Tightly coupled EKF/optimization for drone flight dynamics |
| **Offline Point Cloud Cleaning (SOR)** | **[isl-org/Open3D](https://github.com/isl-org/Open3D)** | Cleans `.ply` point clouds and computes ICP registration |
| **Ground Station SLAM & Memory Mgmt** | **[introlab/rtabmap](https://github.com/introlab/rtabmap)** | Accumulates multi-session 3D room point clouds |
| **Multi-Camera & IMU Calibration** | **[ethz-asl/kalibr](https://github.com/ethz-asl/kalibr)** | Calibrates baseline $B$ and $T_{\text{cam}}^{\text{IMU}}$ matrix |
