#!/usr/bin/env python3
"""
export_mesh.py
Reconstructs 3D mesh from optimized SLAM poses and depth keyframes,
projects radiometric thermal measurements onto mesh vertices,
and saves standard geometry (.ply) + numerical thermal sidecar (.npz) + JSON manifest.
"""

import os
import json
import numpy as np
from typing import List, Dict, Any

def export_thermal_model(
    vertices: np.ndarray,          # (N, 3) float32
    triangles: np.ndarray,         # (M, 3) int32
    temperatures_c: np.ndarray,    # (N,) float32 Celsius
    confidence: np.ndarray,        # (N,) float32 [0, 1]
    output_dir: str,
    model_name: str = "dron_thermal_mesh",
    session_id: str = "session_001"
):
    os.makedirs(output_dir, exist_ok=True)
    
    ply_path = os.path.join(output_dir, f"{model_name}.ply")
    npz_path = os.path.join(output_dir, f"{model_name}_thermal.npz")
    json_path = os.path.join(output_dir, f"{model_name}_manifest.json")

    # 1. Export Standard Binary PLY
    n_v = vertices.shape[0]
    n_f = triangles.shape[0] if triangles is not None else 0

    header = (
        "ply\n"
        "format ascii 1.0\n"
        f"element vertex {n_v}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property float temperature_c\n"
        "property float confidence\n"
    )
    if n_f > 0:
        header += (
            f"element face {n_f}\n"
            "property list uchar int vertex_indices\n"
        )
    header += "end_header\n"

    with open(ply_path, "w", encoding="utf-8") as f:
        f.write(header)
        for i in range(n_v):
            f.write(f"{vertices[i,0]:.4f} {vertices[i,1]:.4f} {vertices[i,2]:.4f} {temperatures_c[i]:.2f} {confidence[i]:.3f}\n")
        if n_f > 0:
            for j in range(n_f):
                f.write(f"3 {triangles[j,0]} {triangles[j,1]} {triangles[j,2]}\n")

    # 2. Export Numerical Radiometric Sidecar (NPZ)
    np.savez_compressed(
        npz_path,
        vertices=vertices,
        temperatures_c=temperatures_c,
        confidence=confidence,
        triangles=triangles if triangles is not None else np.zeros((0,3), dtype=np.int32)
    )

    # 3. Export Companion JSON Manifest
    manifest = {
        "model_name": model_name,
        "session_id": session_id,
        "vertex_count": int(n_v),
        "face_count": int(n_f),
        "units": {
            "spatial": "meters",
            "temperature": "celsius"
        },
        "stats": {
            "min_temp_c": float(np.min(temperatures_c[temperatures_c > -500])) if np.any(temperatures_c > -500) else None,
            "max_temp_c": float(np.max(temperatures_c)) if len(temperatures_c) > 0 else None,
            "mean_temp_c": float(np.mean(temperatures_c[temperatures_c > -500])) if np.any(temperatures_c > -500) else None
        },
        "files": {
            "geometry": os.path.basename(ply_path),
            "sidecar": os.path.basename(npz_path)
        }
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

    print(f"Exported thermal 3D model to:\n  PLY: {ply_path}\n  NPZ: {npz_path}\n  JSON: {json_path}")
    return ply_path, npz_path, json_path

if __name__ == "__main__":
    # Test export with a synthetic cube
    verts = np.array([
        [-1, -1, 2], [1, -1, 2], [1, 1, 2], [-1, 1, 2]
    ], dtype=np.float32)
    faces = np.array([
        [0, 1, 2], [0, 2, 3]
    ], dtype=np.int32)
    temps = np.array([24.5, 82.0, 85.3, 25.1], dtype=np.float32)
    confs = np.array([0.95, 0.98, 0.97, 0.92], dtype=np.float32)

    export_thermal_model(verts, faces, temps, confs, "output/test_model")
