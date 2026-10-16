"""
observation_store.py
Keyframe-local observation storage. Decouples local sensor measurements from global SLAM poses.
"""

from typing import Dict, List, Optional, Any
import numpy as np

class KeyframeChunk:
    def __init__(
        self,
        chunk_id: int,
        session_id: str,
        drone_id: str,
        timestamp_s: float,
        points: np.ndarray,          # (N, 3) in Left camera frame
        temperatures_c: np.ndarray,  # (N,) float Celsius
        confidence: np.ndarray,      # (N,) float
        valid_mask: np.ndarray,      # (N,) bool
        pose_world_keyframe: np.ndarray = None # 4x4 matrix
    ):
        self.chunk_id = chunk_id
        self.session_id = session_id
        self.drone_id = drone_id
        self.timestamp_s = timestamp_s
        self.points = points
        self.temperatures_c = temperatures_c
        self.confidence = confidence
        self.valid_mask = valid_mask
        self.pose_world_keyframe = pose_world_keyframe if pose_world_keyframe is not None else np.eye(4, dtype=np.float64)
        self.revision = 1

class ObservationStore:
    def __init__(self):
        self.chunks: Dict[int, KeyframeChunk] = {}
        self.current_revision = 0

    def add_chunk(self, chunk: KeyframeChunk):
        self.chunks[chunk.chunk_id] = chunk

    def update_poses(self, chunk_ids: List[int], poses: List[np.ndarray], revision: int):
        if revision <= self.current_revision:
            return # Ignore stale revisions
        self.current_revision = revision
        for cid, pose in zip(chunk_ids, poses):
            if cid in self.chunks:
                self.chunks[cid].pose_world_keyframe = pose.copy()
                self.chunks[cid].revision = revision

    def get_world_point_cloud(self, min_confidence: float = 0.1) -> Dict[str, np.ndarray]:
        """
        Reconstructs the global point cloud by transforming all keyframe chunks by their optimized poses.
        """
        all_pts = []
        all_temps = []
        all_confs = []

        for chunk in self.chunks.values():
            mask = chunk.valid_mask & (chunk.confidence >= min_confidence)
            if not np.any(mask):
                continue

            pts_local = chunk.points[mask] # (M, 3)
            # Transform to world: P_w = R_w_c @ P_c + T_w_c
            R = chunk.pose_world_keyframe[:3, :3]
            T = chunk.pose_world_keyframe[:3, 3]

            pts_world = (R @ pts_local.T).T + T
            all_pts.append(pts_world)
            all_temps.append(chunk.temperatures_c[mask])
            all_confs.append(chunk.confidence[mask])

        if len(all_pts) == 0:
            return {
                "points": np.zeros((0, 3), dtype=np.float32),
                "temperatures": np.zeros(0, dtype=np.float32),
                "confidence": np.zeros(0, dtype=np.float32)
            }

        return {
            "points": np.vstack(all_pts).astype(np.float32),
            "temperatures": np.concatenate(all_temps).astype(np.float32),
            "confidence": np.concatenate(all_confs).astype(np.float32)
        }
