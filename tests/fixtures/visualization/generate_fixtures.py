#!/usr/bin/env python3
"""
generate_fixtures.py
Generates deterministic synthetic fixtures for testing the DRON 3D thermal mapping pipeline.
- Fixture 1: Synthetic plane at depth Z = 2.0m with known rectangular temperature pattern.
- Fixture 2: Occlusion scene with foreground box (Z=1.0m, 80°C) in front of a background wall (Z=3.0m, 20°C).
"""

import os
import numpy as np

def generate_fixtures(output_base: str = "tests/fixtures/visualization"):
    plane_dir = os.path.join(output_base, "synthetic_plane")
    occl_dir = os.path.join(output_base, "occlusion_scene")
    os.makedirs(plane_dir, exist_ok=True)
    os.makedirs(occl_dir, exist_ok=True)

    # 1. Synthetic Plane at Z = 2.0m
    # 100x100 grid spanning [-1, 1] in X and [-0.75, 0.75] in Y
    xx, yy = np.meshgrid(np.linspace(-1.0, 1.0, 100), np.linspace(-0.75, 0.75, 75))
    zz = np.full_like(xx, 2.0)
    pts_plane = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1).astype(np.float32)

    # Thermal image (62x80): Ambient 22°C with a 65°C center block
    thermal_plane = np.full((62, 80), 22.0, dtype=np.float32)
    thermal_plane[20:42, 25:55] = 65.0

    np.save(os.path.join(plane_dir, "points_rectified_left.npy"), pts_plane)
    np.save(os.path.join(plane_dir, "thermal_matrix.npy"), thermal_plane)

    # 2. Occlusion Scene
    # Foreground box: Z = 1.0m, covering [-0.2, 0.2] in X, [-0.2, 0.2] in Y
    xf, yf = np.meshgrid(np.linspace(-0.2, 0.2, 30), np.linspace(-0.2, 0.2, 30))
    zf = np.full_like(xf, 1.0)
    pts_fg = np.stack([xf.ravel(), yf.ravel(), zf.ravel()], axis=1)

    # Background wall: Z = 3.0m, covering [-1.5, 1.5] in X, [-1.0, 1.0] in Y (including directly behind the box!)
    xb, yb = np.meshgrid(np.linspace(-1.5, 1.5, 60), np.linspace(-1.0, 1.0, 40))
    zb = np.full_like(xb, 3.0)
    pts_bg = np.stack([xb.ravel(), yb.ravel(), zb.ravel()], axis=1)

    pts_scene = np.vstack([pts_fg, pts_bg]).astype(np.float32)

    # Thermal image: Foreground box is HOT (85°C), background is COOL (20°C)
    thermal_occl = np.full((62, 80), 20.0, dtype=np.float32)
    # The box at Z=1.0m projects to roughly the center 24x24 pixels of thermal sensor
    thermal_occl[20:44, 28:52] = 85.0

    np.save(os.path.join(occl_dir, "points_rectified_left.npy"), pts_scene)
    np.save(os.path.join(occl_dir, "thermal_matrix.npy"), thermal_occl)

    print("Generated test fixtures successfully in:", output_base)

if __name__ == "__main__":
    generate_fixtures()
