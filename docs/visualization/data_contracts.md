# Data Contracts & Message Specifications

## 1. Topics
| Topic | Type | Semantics |
|---|---|---|
| `stereo/left/image_raw` | `sensor_msgs/Image` | Left rectified RGB/Mono8 image with hardware timestamp |
| `stereo/left/camera_info` | `sensor_msgs/CameraInfo` | Intrinsic matrix, distortion, image dimensions |
| `thermal/image_temperature` | `sensor_msgs/Image` | `32FC1` (Float32 in Celsius), NaN for invalid pixels |
| `thermal/camera_info` | `sensor_msgs/CameraInfo` | Thermal intrinsics, optical frame ID |
| `mapping/keyframe_chunk` | `dron_interfaces/ThermalChunk` | Keyframe-local geometry + radiometric temperatures |
| `mapping/pose_graph` | `dron_interfaces/PoseGraphUpdate` | Atomic SLAM loop-closure pose updates |
| `mapping/status` | `dron_interfaces/MappingStatus` | Tracking status, point count, latency, time skew |

## 2. PointCloud2 Binary Layout
Field layout inside `ThermalChunk.points`:
- `x`: FLOAT32 (offset 0)
- `y`: FLOAT32 (offset 4)
- `z`: FLOAT32 (offset 8)
- `temperature_c`: FLOAT32 (offset 12)
- `confidence`: FLOAT32 (offset 16)
- `valid`: UINT8 (offset 20)
- `observation_dt_s`: FLOAT32 (offset 21)
Total `point_step`: 25 to 28 bytes with padding.
