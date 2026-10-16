import pytest
import struct
import numpy as np

def pack_thermal_pointcloud2(points, temps, confs):
    """
    Simulates ROS 2 PointCloud2 serialization in Python.
    Format: x(f32), y(f32), z(f32), temp(f32), conf(f32) -> 20 bytes per point.
    """
    n = points.shape[0]
    buffer = bytearray(n * 20)
    for i in range(n):
        struct.pack_into(
            "<fffff", buffer, i * 20,
            points[i, 0], points[i, 1], points[i, 2],
            temps[i], confs[i]
        )
    return bytes(buffer)

def unpack_thermal_pointcloud2(buffer):
    n = len(buffer) // 20
    pts = np.zeros((n, 3), dtype=np.float32)
    temps = np.zeros(n, dtype=np.float32)
    confs = np.zeros(n, dtype=np.float32)

    for i in range(n):
        x, y, z, t, c = struct.unpack_from("<fffff", buffer, i * 20)
        pts[i] = [x, y, z]
        temps[i] = t
        confs[i] = c
    return pts, temps, confs

def test_pointcloud2_serialization_roundtrip():
    points = np.array([[1.0, 2.0, 3.0], [-0.5, 0.0, 2.5]], dtype=np.float32)
    temps = np.array([45.5, 88.0], dtype=np.float32)
    confs = np.array([0.95, 0.99], dtype=np.float32)

    raw_bytes = pack_thermal_pointcloud2(points, temps, confs)
    recovered_pts, recovered_temps, recovered_confs = unpack_thermal_pointcloud2(raw_bytes)

    np.testing.assert_allclose(recovered_pts, points, atol=1e-5)
    np.testing.assert_allclose(recovered_temps, temps, atol=1e-5)
    np.testing.assert_allclose(recovered_confs, confs, atol=1e-5)
