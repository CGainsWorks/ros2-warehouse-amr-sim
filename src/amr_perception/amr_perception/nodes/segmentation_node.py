"""Ground removal and Euclidean clustering on the 3D LiDAR cloud.

The node itself is deliberately thin: convert, transform, call into
``amr_perception.core``, publish. All of the interesting behaviour and all of
the tests live in the core.

Two operational details are worth reading before changing anything here.

**The input is a best-effort, depth-1 subscription.** A segmentation node that
queues scans is worse than one that drops them: by the time a 300 ms-old scan is
processed the robot has moved 45 cm at cruise, and the resulting obstacle is
plotted where the robot used to be. Dropping to the newest scan bounds the error
at one scan period no matter how far behind the node falls.

**Output stamps are copied from the input cloud, never set to "now".** This is
what makes the latency figure in ``DetectedObjectArray.processing_ms``
meaningful and what lets the costmap age the observation correctly.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException
from visualization_msgs.msg import MarkerArray

from amr_common.transforms import transform_points
from amr_msgs.msg import DetectedObject, DetectedObjectArray
from amr_perception.core.clustering import cluster_points
from amr_perception.core.ground_removal import remove_ground
from amr_perception.ros_conversions import (
    clusters_to_markers,
    pointcloud2_to_xyz,
    xyz_to_pointcloud2,
)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


def _describe(text: str) -> ParameterDescriptor:
    return ParameterDescriptor(description=text)


class SegmentationNode(Node):
    def __init__(self) -> None:
        super().__init__("segmentation_node")

        self.declare_parameter(
            "target_frame",
            "base_footprint",
            _describe(
                "Frame the cloud is transformed into before segmentation. Must be "
                "a frame whose z=0 is the floor, because the ground-plane seed "
                "band is expressed in it."
            ),
        )
        self.declare_parameter(
            "transform_timeout_sec",
            0.10,
            _describe(
                "How long to wait for the sensor-to-target transform. One scan "
                "period. Waiting longer just guarantees the result is stale."
            ),
        )
        self.declare_parameter("ground_distance_threshold", 0.06, _describe("RANSAC inlier band, metres."))
        self.declare_parameter("ground_max_tilt_deg", 10.0, _describe("Reject planes further than this from vertical."))
        self.declare_parameter("ground_max_iterations", 300, _describe("RANSAC iteration cap."))
        self.declare_parameter("ground_score_subsample", 4000, _describe("Points used to score each hypothesis."))
        self.declare_parameter("max_obstacle_height", 3.0, _describe("Discard returns above this height, metres."))
        self.declare_parameter("cluster_tolerance", 0.35, _describe("Euclidean clustering radius, metres."))
        self.declare_parameter("cluster_min_points", 25, _describe("Minimum raw returns per reported cluster."))
        self.declare_parameter("cluster_max_points", 200000, _describe("Maximum raw returns per reported cluster."))
        self.declare_parameter("cluster_voxel_size", 0.12, _describe("Downsample resolution for the radius graph."))
        self.declare_parameter("cluster_min_height", 0.10, _describe("Discard clusters shorter than this, metres."))
        self.declare_parameter("cluster_range_adaptive", True, _describe("Grow the clustering radius with range."))
        self.declare_parameter("cluster_ring_spacing_rad", 0.0349, _describe("Angular gap between LiDAR rings."))
        self.declare_parameter("cluster_range_gain", 1.3, _describe("Headroom on the predicted ring spacing."))
        self.declare_parameter("cluster_max_tolerance", 0.80, _describe("Ceiling on the adaptive radius, metres."))
        self.declare_parameter(
            "sensor_origin_xyz",
            [-0.15, 0.0, 0.90],
            _describe(
                "3D LiDAR position in target_frame, from the URDF. Range for the "
                "adaptive clustering radius is measured from here, so a wrong "
                "value understates range and under-merges distant objects."
            ),
        )
        self.declare_parameter(
            "publish_debug_clouds",
            True,
            _describe(
                "Publish the split ground and obstacle clouds. Roughly doubles "
                "the node's bandwidth; useful in RViz, wasteful on a real robot."
            ),
        )
        self.declare_parameter("publish_markers", True, _describe("Publish cluster bounding boxes for RViz."))

        self._target_frame = self.get_parameter("target_frame").value
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._objects_pub = self.create_publisher(DetectedObjectArray, "~/objects", 10)
        self._ground_pub = self.create_publisher(PointCloud2, "~/ground", SENSOR_QOS)
        self._obstacles_pub = self.create_publisher(PointCloud2, "~/obstacles", SENSOR_QOS)
        self._marker_pub = self.create_publisher(MarkerArray, "~/markers", 5)

        self._sub = self.create_subscription(PointCloud2, "~/input", self._on_cloud, SENSOR_QOS)

        self._scan_count = 0
        self._dropped_transform = 0
        self._latency_ms: list = []
        self.create_timer(10.0, self._report_health)

        self.get_logger().info(
            "segmentation_node up: target_frame=%s, listening on %s"
            % (self._target_frame, self._sub.topic_name)
        )

    # ------------------------------------------------------------------
    def _lookup(self, source_frame: str, stamp):
        timeout = float(self.get_parameter("transform_timeout_sec").value)
        try:
            return self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=timeout),
            )
        except (LookupException, ExtrapolationException, ConnectivityException) as exc:
            self._dropped_transform += 1
            # Throttled: a missing transform during startup would otherwise
            # produce one log line per scan.
            self.get_logger().warn(
                "dropping scan, no %s -> %s transform: %s" % (source_frame, self._target_frame, exc),
                throttle_duration_sec=2.0,
            )
            return None

    def _on_cloud(self, msg: PointCloud2) -> None:
        transform = self._lookup(msg.header.frame_id, msg.header.stamp)
        if transform is None:
            return

        started = time.perf_counter()

        points = pointcloud2_to_xyz(msg)
        if len(points) == 0:
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        points = transform_points(points, np.array([t.x, t.y, t.z]), (q.x, q.y, q.z, q.w))

        ground_mask, object_mask, plane = remove_ground(
            points,
            distance_threshold=float(self.get_parameter("ground_distance_threshold").value),
            max_height=float(self.get_parameter("max_obstacle_height").value),
            max_tilt_deg=float(self.get_parameter("ground_max_tilt_deg").value),
            max_iterations=int(self.get_parameter("ground_max_iterations").value),
            score_subsample=int(self.get_parameter("ground_score_subsample").value) or None,
        )
        if plane is None:
            self.get_logger().warn(
                "ground plane fit failed; treating the whole cloud as obstacles",
                throttle_duration_sec=5.0,
            )

        obstacle_points = points[object_mask]
        clusters = cluster_points(
            obstacle_points,
            tolerance=float(self.get_parameter("cluster_tolerance").value),
            min_points=int(self.get_parameter("cluster_min_points").value),
            max_points=int(self.get_parameter("cluster_max_points").value),
            voxel_size=float(self.get_parameter("cluster_voxel_size").value),
            min_height=float(self.get_parameter("cluster_min_height").value),
            range_adaptive=bool(self.get_parameter("cluster_range_adaptive").value),
            ring_spacing_rad=float(self.get_parameter("cluster_ring_spacing_rad").value),
            range_gain=float(self.get_parameter("cluster_range_gain").value),
            max_tolerance=float(self.get_parameter("cluster_max_tolerance").value),
            sensor_origin=tuple(float(v) for v in self.get_parameter("sensor_origin_xyz").value),
        )

        elapsed_ms = (time.perf_counter() - started) * 1e3
        self._latency_ms.append(elapsed_ms)
        self._scan_count += 1

        self._publish(msg, clusters, plane, points[ground_mask], obstacle_points, elapsed_ms)

    def _publish(self, msg, clusters, plane, ground_points, obstacle_points, elapsed_ms) -> None:
        stamp = msg.header.stamp
        frame = self._target_frame

        array = DetectedObjectArray()
        array.header.stamp = stamp
        array.header.frame_id = frame
        array.processing_ms = float(elapsed_ms)
        array.ground_plane = (
            [0.0, 0.0, 0.0, 0.0]
            if plane is None
            else [float(plane.normal[0]), float(plane.normal[1]), float(plane.normal[2]), float(plane.d)]
        )

        for cluster in clusters:
            obj = DetectedObject()
            obj.id = int(cluster.id)
            obj.point_count = int(cluster.size)
            obj.centroid.x, obj.centroid.y, obj.centroid.z = [float(v) for v in cluster.centroid]
            obj.bbox_min.x, obj.bbox_min.y, obj.bbox_min.z = [float(v) for v in cluster.bbox_min]
            obj.bbox_max.x, obj.bbox_max.y, obj.bbox_max.z = [float(v) for v in cluster.bbox_max]
            dims = cluster.dimensions
            obj.dimensions.x, obj.dimensions.y, obj.dimensions.z = [float(v) for v in dims]
            array.objects.append(obj)

        self._objects_pub.publish(array)

        if self.get_parameter("publish_debug_clouds").value:
            self._ground_pub.publish(xyz_to_pointcloud2(ground_points, frame, stamp))
            self._obstacles_pub.publish(xyz_to_pointcloud2(obstacle_points, frame, stamp))

        if self.get_parameter("publish_markers").value:
            self._marker_pub.publish(clusters_to_markers(clusters, frame, stamp))

    def _report_health(self) -> None:
        """Periodic latency summary.

        Percentiles rather than a mean: the mean hides the tail, and the tail is
        what actually causes a dropped scan.
        """
        if not self._latency_ms:
            if self._dropped_transform:
                self.get_logger().warn(
                    "no scans processed in the last 10 s (%d dropped for missing transforms)"
                    % self._dropped_transform
                )
            return
        samples = np.asarray(self._latency_ms)
        self.get_logger().info(
            "%d scans: latency median %.1f ms, p95 %.1f ms, max %.1f ms (%d dropped for transforms)"
            % (
                len(samples),
                float(np.median(samples)),
                float(np.percentile(samples, 95)),
                float(samples.max()),
                self._dropped_transform,
            )
        )
        self._latency_ms.clear()
        self._dropped_transform = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SegmentationNode()
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
