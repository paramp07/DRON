# Multimodal Calibration Protocol

## 1. Hardware Requirements
- **Heated Checkerboard / Perforated Board**: Standard printed paper targets are invisible to long-wave infrared (LWIR) cameras. Use a perforated metal/acrylic target with an electric heating pad or warm lamp behind it to provide high contrast in both RGB and thermal spectra simultaneously.

## 2. Calibration Procedure
1. **Stereo RGB Calibration**:
   - Collect 30-50 synchronized stereo pairs covering diverse orientations and depths.
   - Run `tools/visualization/calibrate_stereo.py` to solve $K_L, K_R, D_L, D_R, R, T$.
   - Enforce vertical epipolar residual error < 0.5 px median.
2. **Thermal Intrinsic Calibration**:
   - Collect 20-30 heated target poses.
   - Run `tools/visualization/calibrate_thermal.py` to solve $K_{th}, D_{th}$.
3. **Thermal-to-RGB Extrinsic Calibration**:
   - Using common board detections with fixed intrinsics, solve rigid transformation $[R_{th \leftarrow L} | T_{th \leftarrow L}]$.
   - Verify that baseline offset matches physical CAD measurements within 5mm.

## 3. Storage
Save all calibration parameters into `config/visualization/rigs/<rig_id>/` and link in `manifest.yaml`.
