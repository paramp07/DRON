import pytest
import numpy as np
import cv2
from tools.visualization.thermal_projector import ThermalProjector

@pytest.fixture
def test_projector():
    # Intrinsic matrix for 80x62 thermal camera (fx=fy=96.5, cx=39.5, cy=30.5)
    k_th = np.array([
        [96.5, 0.0, 39.5],
        [0.0, 96.5, 30.5],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    d_th = np.zeros(5, dtype=np.float64)
    r_th_l = np.eye(3, dtype=np.float64)
    t_th_l = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    return ThermalProjector(k_th, d_th, r_th_l, t_th_l, thermal_shape=(62, 80), depth_tolerance_m=0.15)

def test_on_axis_point_projection(test_projector):
    # Point directly on optical axis at Z = 2.0m: should project to (cx, cy) = (39.5, 30.5)
    pts_rect = np.array([[0.0, 0.0, 2.0]], dtype=np.float32)
    thermal_mat = np.full((62, 80), 25.0, dtype=np.float32)
    thermal_mat[30:32, 39:41] = 88.0 # Hotspot at principal point
    r1 = np.eye(3, dtype=np.float64)

    res = test_projector.project(pts_rect, thermal_mat, r1)

    assert res["valid_mask"][0] == True
    assert 80.0 <= res["temperatures_c"][0] <= 88.0

def test_unrectification_rotation(test_projector):
    # Non-zero R1 rotation (e.g. 5 degree yaw rotation)
    angle = np.radians(5.0)
    r1 = np.array([
        [np.cos(angle), 0.0, np.sin(angle)],
        [0.0, 1.0, 0.0],
        [-np.sin(angle), 0.0, np.cos(angle)]
    ], dtype=np.float64)

    pts_rect = np.array([[0.0, 0.0, 2.0]], dtype=np.float32)
    thermal_mat = np.full((62, 80), 30.0, dtype=np.float32)

    res = test_projector.project(pts_rect, thermal_mat, r1)
    
    # Point in left camera frame should have been rotated by R1.T
    expected_x = pts_rect @ r1
    np.testing.assert_allclose(res["points_left"][0], expected_x[0], atol=1e-4)

def test_invalid_nan_negative_depth(test_projector):
    pts_invalid = np.array([
        [np.nan, 0.0, 2.0],
        [0.0, np.inf, 2.0],
        [0.0, 0.0, -1.0],  # Behind camera
        [100.0, 100.0, 2.0] # Far outside sensor FOV
    ], dtype=np.float32)
    thermal_mat = np.full((62, 80), 20.0, dtype=np.float32)
    r1 = np.eye(3, dtype=np.float64)

    res = test_projector.project(pts_invalid, thermal_mat, r1)

    assert np.all(res["valid_mask"] == False)
    assert np.all(res["temperatures_c"] == -999.0)

def test_occlusion_prevents_background_bleeding(test_projector):
    # Two points on same ray:
    # 1. Foreground point at Z = 1.0m (Hot: 85°C)
    # 2. Background point at Z = 3.0m (Cool: 20°C)
    pts = np.array([
        [0.0, 0.0, 1.0], # Foreground
        [0.0, 0.0, 3.0]  # Background behind it
    ], dtype=np.float32)

    thermal_mat = np.full((62, 80), 20.0, dtype=np.float32)
    thermal_mat[30, 40] = 85.0
    r1 = np.eye(3, dtype=np.float64)

    res = test_projector.project(pts, thermal_mat, r1)

    # Foreground point must be valid and receive temperature
    assert res["valid_mask"][0] == True
    # Background point MUST be rejected as occluded!
    assert res["valid_mask"][1] == False
