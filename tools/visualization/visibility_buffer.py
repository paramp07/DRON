"""
visibility_buffer.py
Z-buffer in the thermal camera focal plane to resolve occlusions and prevent
foreground heat from bleeding through onto background geometry.
"""

import numpy as np

class VisibilityBuffer:
    def __init__(self, width: int = 80, height: int = 62, depth_tolerance_m: float = 0.15):
        self.width = width
        self.height = height
        self.depth_tolerance_m = depth_tolerance_m
        self.buffer = np.full((height, width), np.inf, dtype=np.float32)

    def reset(self):
        self.buffer.fill(np.inf)

    def insert_depth(self, u: int, v: int, z_thermal: float):
        if 0 <= u < self.width and 0 <= v < self.height and z_thermal > 0:
            if z_thermal < self.buffer[v, u]:
                self.buffer[v, u] = z_thermal

    def is_visible(self, u: int, v: int, z_thermal: float) -> bool:
        if 0 <= u < self.width and 0 <= v < self.height and z_thermal > 0:
            closest_z = self.buffer[v, u]
            if np.isinf(closest_z):
                return False
            return z_thermal <= (closest_z + self.depth_tolerance_m)
        return False
