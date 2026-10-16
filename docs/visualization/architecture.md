# DRON 3D Thermal Visualization Architecture

## 1. Overview
The DRON 3D thermal mapping system reconstructs a dense 3D representation of indoor/structural fire environments from stereo cameras and overlays radiometric thermal measurements onto the geometry.

## 2. Distributed Responsibilities
- **Drone Edge (Raspberry Pi 5)**:
  - Synchronized image capture (`Picamera2`)
  - USB serial radiometric readout from Meridian Innovation MI48 (`senxor`)
  - ROS 2 publishers with monotonic timestamps and optical frame IDs
- **Ground Control Computer**:
  - Stereo rectification (`stereo_image_proc` / OpenCV SGBM)
  - Odometry and Pose Graph SLAM (`RTAB-Map`)
  - 3D-to-thermal distortion-aware projection with z-buffer occlusion checking
  - Keyframe-local observation storage (`ObservationStore`)
- **Unity Ground Station Visualizer**:
  - GPU point cloud rendering via structured `GraphicsBuffer` and procedural shaders
  - Interactive temperature windowing and colormaps (Turbo, Ironbow, Inferno)
  - Memory-budgeted chunk store with atomic SLAM graph corrections

## 3. Data Flow Diagram
```
[Pi 5: Dual Cameras]  --> /drone_01/stereo/left/image_raw  --> [Ground: Stereo Depth & SLAM]
                      --> /drone_01/stereo/right/image_raw               |
[Pi 5: MI48 Thermal]  --> /drone_01/thermal/image_temperature            v
                                                              [Ground: Thermal Projection & Store]
                                                                         |
                                                              /drone_01/mapping/keyframe_chunk
                                                                         |
                                                                         v
                                                               [Unity 3D Visualizer]
```
