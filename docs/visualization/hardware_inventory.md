# Hardware Inventory & Platform Specifications

| Component | Model / Specification | Interface | Notes |
|---|---|---|---|
| Drone Compute | Raspberry Pi 5 (8GB RAM) | PCIe / USB 3.0 / CSI | Runs Pi OS 64-bit + ROS 2 Humble |
| Stereo Cameras | Dual Sony IMX708 / Pi Camera Module 3 | Dual MIPI CSI (CAM0 & CAM1) | Hardware synchronized via Picamera2 |
| Thermal Sensor | Meridian Innovation MI48A2 / MI48A3 | USB / UART Serial | 80x62 resolution, 9 Hz radiometric output |
| Flight Controller | Pixhawk 6C / PX4 Autopilot | USB-C / Telem | Micro-XRCE-DDS bridge to ROS 2 |
| Ground Station | Laptop with NVIDIA GPU (16GB RAM) | 5GHz Wi-Fi / Ethernet | Runs ROS 2, RTAB-Map, and Unity Visualizer |
