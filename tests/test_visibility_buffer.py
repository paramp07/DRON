import pytest
import numpy as np
from tools.visualization.visibility_buffer import VisibilityBuffer

def test_visibility_buffer_basic():
    vb = VisibilityBuffer(width=80, height=62, depth_tolerance_m=0.15)
    
    # Insert foreground point at (u=40, v=30) with depth 1.2m
    vb.insert_depth(40, 30, 1.2)

    # Point at same pixel with depth 1.25m should be visible (within 0.15m tolerance)
    assert vb.is_visible(40, 30, 1.25) == True

    # Point at same pixel with depth 2.5m should be OCCLUDED (behind foreground)
    assert vb.is_visible(40, 30, 2.5) == False

def test_visibility_buffer_closer_overwrites():
    vb = VisibilityBuffer(width=80, height=62, depth_tolerance_m=0.10)
    
    # Insert background first
    vb.insert_depth(25, 25, 3.0)
    # Then insert foreground
    vb.insert_depth(25, 25, 1.0)

    # Foreground should now be closest
    assert vb.is_visible(25, 25, 1.05) == True
    # Background should now be occluded
    assert vb.is_visible(25, 25, 3.0) == False

def test_visibility_buffer_boundary():
    vb = VisibilityBuffer(width=80, height=62)
    # Out of bounds should return False
    assert vb.is_visible(-1, 0, 1.0) == False
    assert vb.is_visible(80, 0, 1.0) == False
    assert vb.is_visible(0, 62, 1.0) == False
