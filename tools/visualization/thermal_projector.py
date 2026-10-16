"""
thermal_projector.py
Handles rigid-body transformation from left camera frame into thermal frame,
lens distortion projection, z-buffer occlusion check, and radiometric bilinear sampling.
"""

import numpy as np
import cv2
from typing import Tuple, Dict, Any
from .visibility_buffer import VisibilityBuffer

class ThermalProjector:
    def __init__(
        self,
        k_thermal: np.ndarray,
        d_thermal: np.ndarray,
        r_thermal_left: np.ndarray,
        t_thermal_left: np.ndarray,
        thermal_shape: Tuple[int, int] = (62, 80),
        depth_tolerance_m: float = 0.15
    ):
        self.k_thermal = np.asarray(k_thermal, dtype=np.float64)
        self.d_thermal = np.asarray(d_thermal, dtype=np.float64).ravel()
        self.r_thermal_left = np.asarray(r_thermal_left, dtype=np.float64)
        self.t_thermal_left = np.asarray(t_thermal_left, dtype=np.float64).reshape(3, 1)
        self.height, self.width = thermal_shape
        self.visibility_buffer = VisibilityBuffer(self.width, self.height, depth_tolerance_m)

    def project(
        self,
        points_rectified_left: np.ndarray,
        thermal_matrix: np.ndarray,
        r1: np.ndarray,
        dt_s: float = 0.0
    ) -> Dict[str, np.ndarray]:
        """
        Projects 3D points into thermal image and samples temperatures.
        :param points_rectified_left: (N, 3) 3D points in rectified left coordinates.
        :param thermal_matrix: (H, W) array of temperatures in Celsius.
        :param r1: (3, 3) stereo rectification rotation matrix.
        :param dt_s: Time difference between stereo and thermal acquisition.
        :return: Dict containing 'points_left', 'temperatures_c', 'confidence', 'valid_mask'
        """
        n_points = points_rectified_left.shape[0]
        if n_points == 0:
            return {
                "points_left": np.zeros((0, 3), dtype=np.float32),
                "temperatures_c": np.zeros(0, dtype=np.float32),
                "confidence": np.zeros(0, dtype=np.float32),
                "valid_mask": np.zeros(0, dtype=bool)
            }

        # 1. Un-rectify: P_left = R1.T @ P_rectified
        # In NumPy row-vector format: P_left = P_rectified @ R1
        pts_left = (points_rectified_left @ r1).astype(np.float32)

        # 2. Rigid transformation to Thermal Frame: P_th = R_th @ P_left + T_th
        pts_th = (self.r_thermal_left @ pts_left.T + self.t_thermal_left).T.astype(np.float32)

        z_th = pts_th[:, 2]
        valid_z = (z_th > 0.05) & np.isfinite(pts_th).all(axis=1)

        valid_indices = np.where(valid_z)[0]
        if len(valid_indices) == 0:
            return {
                "points_left": pts_left,
                "temperatures_c": np.full(n_points, -999.0, dtype=np.float32),
                "confidence": np.zeros(n_points, dtype=np.float32),
                "valid_mask": np.zeros(n_points, dtype=bool)
            }

        # 3. Perspective Projection with distortion
        pts_to_project = pts_th[valid_indices]
        rvec = np.zeros((3, 1), dtype=np.float64)
        tvec = np.zeros((3, 1), dtype=np.float64)

        img_pts, _ = cv2.projectPoints(
            pts_to_project, rvec, tvec, self.k_thermal, self.d_thermal
        )
        img_pts = img_pts.reshape(-1, 2)
        u = img_pts[:, 0]
        v = img_pts[:, 1]

        # 4. Populate Visibility Z-buffer
        self.visibility_buffer.reset()
        for idx_local, orig_idx in enumerate(valid_indices):
            ui = int(np.round(u[idx_local]))
            vi = int(np.round(v[idx_local]))
            self.visibility_buffer.insert_depth(ui, vi, z_th[orig_idx])

        # 5. Boundary & Occlusion filtering + Bilinear sampling
        in_bounds = (u >= 0.0) & (u <= self.width - 1.001) & (v >= 0.0) & (v <= self.height - 1.001)
        
        temperatures = np.full(n_points, -999.0, dtype=np.float32)
        confidence = np.zeros(n_points, dtype=np.float32)
        valid_mask = np.zeros(n_points, dtype=bool)

        for idx_local, orig_idx in enumerate(valid_indices):
            if not in_bounds[idx_local]:
                continue

            ui = int(np.round(u[idx_local]))
            vi = int(np.round(v[idx_local]))

            if not self.visibility_buffer.is_visible(ui, vi, z_th[orig_idx]):
                continue # Occluded

            # Bilinear interpolation
            u_val = u[idx_local]
            v_val = v[idx_local]

            u0 = int(np.floor(u_val))
            v0 = int(np.floor(v_val))
            u1 = min(u0 + 1, self.width - 1)
            v1 = min(v0 + 1, self.height - 1)

            du = u_val - u0
            dv = v_val - v0

            t00 = thermal_matrix[v0, u0]
            t01 = thermal_matrix[v0, u1]
            t10 = thermal_matrix[v1, u0]
            t11 = thermal_matrix[v1, u1]

            sampled = (1.0 - du) * (1.0 - dv) * t00 + \
                      du * (1.0 - dv) * t01 + \
                      (1.0 - du) * dv * t10 + \
                      du * dv * t11

            temperatures[orig_idx] = sampled
            confidence[orig_idx] = max(0.0, 1.0 - (abs(dt_s) / 0.1))
            valid_mask[orig_idx] = True

        return {
            "points_left": pts_left,
            "temperatures_c": temperatures,
            "confidence": confidence,
            "valid_mask": valid_mask
        }
