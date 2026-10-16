"""
timestamp_adapter.py
Utilities for clock synchronization, timestamp skew measurement,
and motion error budget enforcement between stereo and thermal frames.
"""

from typing import Tuple

class TimestampValidator:
    def __init__(self, max_skew_s: float = 0.05, angular_rate_threshold: float = 0.5):
        """
        :param max_skew_s: Maximum permissible timestamp difference (seconds) between paired frames.
        :param angular_rate_threshold: Max yaw rate (rad/s) beyond which skew causes geometric distortion.
        """
        self.max_skew_s = max_skew_s
        self.angular_rate_threshold = angular_rate_threshold

    def evaluate_pair(self, stereo_stamp_s: float, thermal_stamp_s: float, angular_velocity_rad_s: float = 0.0) -> Tuple[bool, float, str]:
        """
        Evaluates whether a stereo and thermal frame can be validly projected.
        Returns: (is_valid, skew_s, reason_message)
        """
        skew = abs(stereo_stamp_s - thermal_stamp_s)
        
        if skew > self.max_skew_s:
            return False, skew, f"Timestamp skew {skew*1000:.1f}ms exceeds tolerance {self.max_skew_s*1000:.1f}ms"
            
        if abs(angular_velocity_rad_s) > self.angular_rate_threshold and skew > 0.02:
            return False, skew, f"High angular velocity {angular_velocity_rad_s:.2f} rad/s under non-zero skew {skew*1000:.1f}ms"

        return True, skew, "OK"
