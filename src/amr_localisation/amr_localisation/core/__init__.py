"""ROS-free localisation algorithms.

Same rule as ``amr_perception.core``: nothing here imports rclpy, so the scan
matcher can be measured against ground truth in the test suite without standing
up a simulator.
"""

from .scan_matcher import MatchResult, match_scans, scan_to_points
from .synthetic2d import simulate_2d_scan, warehouse_floorplan

__all__ = [
    "MatchResult",
    "match_scans",
    "scan_to_points",
    "simulate_2d_scan",
    "warehouse_floorplan",
]
