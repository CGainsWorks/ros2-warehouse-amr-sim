"""Algorithm cores for the warehouse perception pipeline.

Nothing in this subpackage imports ``rclpy`` or any other ROS package. That is a
hard rule, not a convenience: it is what lets the whole segmentation stack be
unit tested, profiled and regression-checked on a laptop with nothing but numpy
and scipy installed, and it is what makes the CI job in ``.github/workflows``
able to run the real algorithms rather than a mock of them.

The ROS-facing wrappers live in ``amr_perception.nodes`` and are deliberately
thin: they convert messages, look up transforms, and call into here.
"""

from .clustering import Cluster, cluster_label_array, cluster_points
from .ground_removal import GroundPlane, fit_ground_plane, remove_ground
from .occupancy_3d import VoxelOccupancy, build_occupancy
from .voxel import voxel_downsample
from .zones import (
    Zone,
    ZoneOccupancyTracker,
    ZoneState,
    evaluate_zone,
    evaluate_zones,
    load_zones,
    points_in_polygon,
)

__all__ = [
    "Cluster",
    "GroundPlane",
    "Zone",
    "ZoneOccupancyTracker",
    "ZoneState",
    "VoxelOccupancy",
    "build_occupancy",
    "cluster_label_array",
    "cluster_points",
    "evaluate_zone",
    "evaluate_zones",
    "fit_ground_plane",
    "load_zones",
    "points_in_polygon",
    "remove_ground",
    "voxel_downsample",
]
