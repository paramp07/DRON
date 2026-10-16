# Coordinate Frame Conventions

## 1. ROS Standard Coordinate Frames
- **World / Map Frame (`map`)**: FLU (X-Forward, Y-Left, Z-Up).
- **Body Frame (`base_link`)**: FLU (X-Forward, Y-Left, Z-Up), centered on drone center of gravity.
- **Camera Optical Frames (`*_optical_frame`)**: Standard optical frame (X-Right, Y-Down, Z-Forward).

## 2. Unity Coordinate Frames
- **Unity World Space**: Left-handed, Y-up (X-Right, Y-Up, Z-Forward).

## 3. Boundary Transform Conversions
```csharp
// ROS FLU -> Unity
Vector3 unityPos = new Vector3(-rosPos.y, rosPos.z, rosPos.x);

// Optical -> Unity
Vector3 unityOptical = new Vector3(opticalPos.x, -opticalPos.y, opticalPos.z);
```
All coordinate transformations must pass through `CoordinateConverter.cs` (C#) and `tests/test_coordinate_converter.py` (Python).
