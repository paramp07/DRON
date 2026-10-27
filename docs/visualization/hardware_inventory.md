# Hardware Inventory & Platform Specifications

| Component | Model / Specification | Interface | Notes |
|---|---|---|---|
| Drone Compute | Raspberry Pi 5 (8GB RAM) | PCIe / USB 3.0 / CSI | Runs Pi OS 64-bit + ROS 2 Humble |
| Stereo Cameras | Dual Sony IMX219 / Pi Camera Module v2 | Dual MIPI CSI (CAM0 & CAM1) | Verified on RP1 buses; 640x480 to 3280x2464 @ 30fps |
| Thermal Sensor | Meridian Innovation MI48 | USB Virtual COM (/dev/ttyACM0) | Verified 80x62 radiometric array via senxor (9 Hz) |
| Flight Controller | Pixhawk 6C / PX4 Autopilot | USB-C / Telem | Micro-XRCE-DDS bridge to ROS 2 |
| Ground Station | Laptop with NVIDIA GPU (16GB RAM) | 5GHz Wi-Fi / Ethernet | Runs ROS 2, RTAB-Map, and Unity Visualizer |
