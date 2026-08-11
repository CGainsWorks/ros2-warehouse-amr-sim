"""Work-zone occupancy publisher.

Zones are fixed in the ``map`` frame — they describe pick faces and a charger
bay, which do not move. The segmentation node publishes in ``base_footprint``,
so this node transforms the obstacle cloud into ``map`` before testing it
against the zone polygons.

That transform is the weak link, and it is treated as one. It depends on AMCL's
``map -> odom`` correction, which is exactly the thing that degrades when
localisation is struggling. If the transform is missing, stale beyond
``transform_timeout_sec``, or the localisation covariance has blown past
``max_position_variance``, the node does **not** publish "clear". It republishes
the last state with ``transform_valid`` false and lets the consumer decide. A
zone publisher that quietly reports clear when it has lost track of where it is
is the single most dangerous thing this pipeline could do.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from visualization_msgs.msg import MarkerArray

from amr_common.transforms import transform_points
from amr_msgs.msg import ZoneOccupancy, ZoneOccupancyArray
from amr_perception.core.clustering import cluster_points
from amr_perception.core.zones import ZoneOccupancyTracker, evaluate_zones, load_zones
from amr_perception.ros_conversions import pointcloud2_to_xyz, zones_to_markers

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

# Zone states are latched for late subscribers: a supervisor that starts after
# the robot must not have to wait a scan period to learn which zones are busy.
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _describe(text: str) -> ParameterDescriptor:
    return ParameterDescriptor(description=text)


class ZoneOccupancyNode(Node):
    def __init__(self) -> None:
        super().__init__("zone_occupancy_node")

        self.declare_parameter("zone_config", "", _describe("Absolute path to the zone definition YAML."))
        self.declare_parameter("map_frame", "map", _describe("Frame the zone polygons are defined in."))
        self.declare_parameter("transform_timeout_sec", 0.10, _describe("Transform wait, seconds."))
        self.declare_parameter("enter_frames", 2, _describe("Consecutive occupied scans needed to latch occupied."))
        self.declare_parameter("exit_frames", 5, _describe("Consecutive clear scans needed to release occupied."))
        self.declare_parameter("cluster_tolerance", 0.35, _describe("Clustering radius, metres."))
        self.declare_parameter("cluster_min_points", 25, _describe("Minimum raw returns per cluster."))
        self.declare_parameter("cluster_voxel_size", 0.12, _describe("Downsample resolution, metres."))
        self.declare_parameter(
            "max_position_variance",
            0.25,
            _describe(
                "Largest AMCL x or y variance, in m^2, for which zone states are "
                "trusted. 0.25 m^2 is a 0.5 m standard deviation, about the width "
                "of the margin the zone polygons are drawn with. Beyond that the "
                "zones cannot be placed accurately enough to be worth publishing "
                "as valid. Set to 0 to disable the check."
            ),
        )
        self.declare_parameter("publish_markers", True, _describe("Publish zone outlines for RViz."))

        config_path = str(self.get_parameter("zone_config").value)
        if not config_path:
            raise RuntimeError("zone_config parameter is required and must be an absolute path")
        self._zones = load_zones(config_path)
        self.get_logger().info(
            "loaded %d zones from %s: %s"
            % (len(self._zones), config_path, ", ".join(z.name for z in self._zones))
        )

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._tracker = ZoneOccupancyTracker(
            enter_frames=int(self.get_parameter("enter_frames").value),
            exit_frames=int(self.get_parameter("exit_frames").value),
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._state_pub = self.create_publisher(ZoneOccupancyArray, "~/zones", STATE_QOS)
        self._marker_pub = self.create_publisher(MarkerArray, "~/zone_markers", 5)

        self._sub = self.create_subscription(PointCloud2, "~/obstacles", self._on_cloud, SENSOR_QOS)
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10
        )

        self._position_variance = 0.0
        self._last_message: ZoneOccupancyArray = None

    # ------------------------------------------------------------------
    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        # Covariance is row-major 6x6 over (x, y, z, roll, pitch, yaw); indices
        # 0 and 7 are the x and y variances.
        cov = msg.pose.covariance
        self._position_variance = max(float(cov[0]), float(cov[7]))

    def _localisation_ok(self) -> bool:
        limit = float(self.get_parameter("max_position_variance").value)
        if limit <= 0.0:
            return True
        return self._position_variance <= limit

    def _on_cloud(self, msg: PointCloud2) -> None:
        timeout = float(self.get_parameter("transform_timeout_sec").value)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, msg.header.frame_id, msg.header.stamp, timeout=Duration(seconds=timeout)
            )
        except (LookupException, ExtrapolationException, ConnectivityException) as exc:
            self.get_logger().warn(
                "no %s -> %s transform, republishing last state as invalid: %s"
                % (msg.header.frame_id, self._map_frame, exc),
                throttle_duration_sec=2.0,
            )
            self._republish_invalid(msg.header.stamp)
            return

        if not self._localisation_ok():
            self.get_logger().warn(
                "localisation variance %.3f m^2 exceeds limit; zone states not trusted"
                % self._position_variance,
                throttle_duration_sec=2.0,
            )
            self._republish_invalid(msg.header.stamp)
            return

        points = pointcloud2_to_xyz(msg)
        if len(points) == 0:
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        points = transform_points(points, np.array([t.x, t.y, t.z]), (q.x, q.y, q.z, q.w))

        clusters = cluster_points(
            points,
            tolerance=float(self.get_parameter("cluster_tolerance").value),
            min_points=int(self.get_parameter("cluster_min_points").value),
            voxel_size=float(self.get_parameter("cluster_voxel_size").value),
        )
        raw_states = evaluate_zones(self._zones, points, clusters)
        latched = self._tracker.update(raw_states)

        array = ZoneOccupancyArray()
        array.header.stamp = msg.header.stamp
        array.header.frame_id = self._map_frame
        array.transform_valid = True

        for state in raw_states:
            zone_msg = ZoneOccupancy()
            zone_msg.name = state.name
            zone_msg.occupied = bool(latched.get(state.name, False))
            zone_msg.occupied_raw = bool(state.occupied)
            zone_msg.point_count = int(state.point_count)
            zone_msg.nearest_cluster_distance = (
                float("nan")
                if not math.isfinite(state.nearest_cluster_distance)
                else float(state.nearest_cluster_distance)
            )
            zone_msg.cluster_ids = [int(i) for i in state.cluster_ids]
            array.zones.append(zone_msg)

        self._last_message = array
        self._state_pub.publish(array)

        if self.get_parameter("publish_markers").value:
            self._marker_pub.publish(
                zones_to_markers(self._zones, latched, self._map_frame, msg.header.stamp)
            )

    def _republish_invalid(self, stamp) -> None:
        """Re-emit the last known state, flagged invalid.

        Deliberately keeps ``occupied`` at its last value rather than clearing
        it. Losing the transform is not evidence that a zone became empty.
        """
        array = ZoneOccupancyArray()
        array.header.stamp = stamp
        array.header.frame_id = self._map_frame
        array.transform_valid = False

        if self._last_message is not None:
            array.zones = self._last_message.zones
        else:
            for zone in self._zones:
                zone_msg = ZoneOccupancy()
                zone_msg.name = zone.name
                # Never having localised is not the same as being clear.
                zone_msg.occupied = True
                zone_msg.occupied_raw = False
                zone_msg.point_count = 0
                zone_msg.nearest_cluster_distance = float("nan")
                array.zones.append(zone_msg)

        self._state_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZoneOccupancyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
