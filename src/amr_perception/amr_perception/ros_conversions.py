"""Message <-> numpy conversion for the perception nodes.

Isolated in one module so that ``amr_perception.core`` stays ROS-free and so
that the one genuinely version-sensitive piece of code in the package — reading
a ``PointCloud2`` — has a single home. ``sensor_msgs_py.point_cloud2.read_points``
returns a structured array on Humble but returned a generator of tuples on
earlier distributions and in some backports; :func:`pointcloud2_to_xyz` handles
both rather than failing at runtime on whichever one is installed.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

# A stable, high-contrast palette for the cluster markers. Cluster ids are
# reassigned every frame, so colour does not identify a persistent object; it
# only makes adjacent clusters distinguishable in RViz.
_PALETTE = [
    (0.90, 0.33, 0.25),
    (0.25, 0.65, 0.90),
    (0.45, 0.80, 0.35),
    (0.95, 0.75, 0.20),
    (0.70, 0.45, 0.85),
    (0.30, 0.80, 0.75),
]


def pointcloud2_to_xyz(msg: PointCloud2, skip_nans: bool = True) -> np.ndarray:
    """Extract an (N, 3) float64 array of finite xyz from a ``PointCloud2``.

    Non-finite points are dropped as well as NaNs. gz-sim's gpu_lidar emits a
    point for every beam, and a beam with no return comes out at infinity
    rather than NaN, which ``skip_nans`` does not catch; one infinite point is
    enough to blow up the voxel grid's bounds.
    """
    raw = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=skip_nans)

    arr = np.asarray(list(raw)) if not isinstance(raw, np.ndarray) else raw

    if arr.dtype.names is not None:  # structured array
        if len(arr) == 0:
            return np.empty((0, 3), dtype=np.float64)
        xyz = np.stack(
            [arr["x"].astype(np.float64), arr["y"].astype(np.float64), arr["z"].astype(np.float64)],
            axis=1,
        )
    else:
        if arr.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        xyz = np.asarray(arr, dtype=np.float64).reshape(-1, 3)
    return xyz[np.isfinite(xyz).all(axis=1)]


def xyz_to_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Build an XYZ float32 ``PointCloud2``."""
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp
    data = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    return point_cloud2.create_cloud_xyz32(header, data)


def _colour(index: int, alpha: float = 0.7) -> ColorRGBA:
    r, g, b = _PALETTE[index % len(_PALETTE)]
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(alpha))


def clusters_to_markers(
    clusters: Sequence,
    frame_id: str,
    stamp,
    namespace: str = "clusters",
    lifetime_sec: float = 0.3,
) -> MarkerArray:
    """Bounding-box markers for a list of clusters.

    A DELETEALL is emitted first. Without it, a frame that detects fewer
    clusters than the previous one leaves the surplus boxes on screen until
    their lifetime expires, which reads as a phantom obstacle. The lifetime is
    a second line of defence for the case where the node dies outright.
    """
    from builtin_interfaces.msg import Duration

    array = MarkerArray()

    clear = Marker()
    clear.header.frame_id = frame_id
    clear.header.stamp = stamp
    clear.ns = namespace
    clear.action = Marker.DELETEALL
    array.markers.append(clear)

    lifetime = Duration()
    lifetime.sec = int(lifetime_sec)
    lifetime.nanosec = int((lifetime_sec % 1.0) * 1e9)

    for cluster in clusters:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = int(cluster.id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        centre = (cluster.bbox_min + cluster.bbox_max) / 2.0
        dims = np.maximum(cluster.dimensions, 0.02)  # a flat box renders as nothing

        marker.pose.position = Point(x=float(centre[0]), y=float(centre[1]), z=float(centre[2]))
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(x=float(dims[0]), y=float(dims[1]), z=float(dims[2]))
        marker.color = _colour(cluster.id)
        marker.lifetime = lifetime
        array.markers.append(marker)

    return array


def zones_to_markers(
    zones: Sequence,
    occupied: dict,
    frame_id: str,
    stamp,
    namespace: str = "zones",
) -> MarkerArray:
    """Outline each zone polygon, red when occupied and green when clear."""
    array = MarkerArray()

    clear_marker = Marker()
    clear_marker.header.frame_id = frame_id
    clear_marker.header.stamp = stamp
    clear_marker.ns = namespace
    clear_marker.action = Marker.DELETEALL
    array.markers.append(clear_marker)

    for index, zone in enumerate(zones):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = index
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05

        is_occupied = bool(occupied.get(zone.name, False))
        marker.color = (
            ColorRGBA(r=0.90, g=0.20, b=0.20, a=0.9)
            if is_occupied
            else ColorRGBA(r=0.20, g=0.80, b=0.30, a=0.6)
        )

        closed = np.vstack([zone.polygon, zone.polygon[:1]])
        for x, y in closed:
            marker.points.append(Point(x=float(x), y=float(y), z=float(zone.z_min)))
        array.markers.append(marker)

    return array
