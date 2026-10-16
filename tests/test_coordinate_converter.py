import pytest
import numpy as np

def ros_flu_to_unity(ros_pos):
    # Unity.X = -ROS.Y, Unity.Y = ROS.Z, Unity.Z = ROS.X
    return np.array([-ros_pos[1], ros_pos[2], ros_pos[0]], dtype=np.float32)

def unity_to_ros_flu(unity_pos):
    return np.array([unity_pos[2], -unity_pos[0], unity_pos[1]], dtype=np.float32)

def ros_optical_to_unity(optical_pos):
    # Optical: X right, Y down, Z forward -> Unity: X right, Y up, Z forward
    return np.array([optical_pos[0], -optical_pos[1], optical_pos[2]], dtype=np.float32)

def test_ros_flu_to_unity_roundtrip():
    original = np.array([1.5, -2.3, 4.0], dtype=np.float32)
    unity = ros_flu_to_unity(original)
    
    assert unity[0] == pytest.approx(2.3)
    assert unity[1] == pytest.approx(4.0)
    assert unity[2] == pytest.approx(1.5)

    recovered = unity_to_ros_flu(unity)
    np.testing.assert_allclose(recovered, original, atol=1e-5)

def test_optical_axis_inversion():
    optical = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    unity = ros_optical_to_unity(optical)

    assert unity[0] == pytest.approx(1.0)
    assert unity[1] == pytest.approx(-2.0)
    assert unity[2] == pytest.approx(3.0)
