"""LiDAR odometry: publishes a scan-matched pose for the EKF to fuse.

This node produces ``odom1`` for ``robot_localization``. It does **not** publish
a transform — the EKF owns ``odom -> base_link``, and a second publisher on the
same edge is the classic way to get a TF tree that oscillates between two
answers.

Two design points:

**Scan-to-keyframe, not scan-to-scan.** Matching each scan against its immediate
predecessor accumulates the per-match error on every single frame: at 10 Hz that
is 10 error samples per second compounding, and the result drifts visibly within
a minute. Holding a reference scan until the robot has actually moved a
meaningful distance means error accumulates per *keyframe* instead, roughly an
order of magnitude less often at cruise, and the residual within a keyframe does
not accumulate at all. The cost is that the match gets harder as overlap falls,
which is what bounds the keyframe distance.

**Fitness gates publication.** A featureless stretch of aisle, or a scan taken
while a forklift fills the field of view, produces a confident and wrong match.
Publishing it poisons the EKF, and unlike a dropped message the EKF has no way
to know it was bad. When the match does not clear ``min_fitness`` the node stays
quiet and lets wheel odometry and the IMU carry that interval.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from amr_common.transforms import quat_from_yaw, wrap_angle
from amr_localisation.core.scan_matcher import scan_to_points
from amr_localisation.core.scan_odometry import ScanOdometry

SCAN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _describe(text: str) -> ParameterDescriptor:
    return ParameterDescriptor(description=text)


class ScanOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_odometry_node")

        self.declare_parameter("odom_frame", "odom", _describe("Parent frame of the published odometry."))
        self.declare_parameter("base_frame", "base_link", _describe("Child frame of the published odometry."))
        self.declare_parameter(
            "keyframe_distance",
            0.35,
            _describe(
                "Translation before a new reference scan is taken, metres. Beyond "
                "roughly 0.5 m the overlap between the current scan and the "
                "keyframe falls far enough that match quality degrades in a 2.4 m "
                "aisle; below ~0.2 m the drift advantage over scan-to-scan is lost."
            ),
        )
        self.declare_parameter(
            "keyframe_rotation",
            0.35,
            _describe(
                "Rotation before a new reference scan is taken, radians (20 deg). "
                "Rotation destroys overlap far faster than translation does, so "
                "this triggers well before the distance threshold during a spin."
            ),
        )
        self.declare_parameter(
            "min_fitness",
            0.60,
            _describe(
                "Inlier fraction below which the match is discarded. Must match "
                "the calibrated default in ScanOdometry: correct matches have a "
                "5th percentile of 0.78 with a tail to 0.38, incorrect ones a "
                "median of 0.47, and 0.60 sits between them. This parameter "
                "overrides the core default, so a stale value here silently "
                "un-does the calibration."
            ),
        )
        self.declare_parameter(
            "max_keyframe_age",
            50,
            _describe(
                "Force a new keyframe after this many matches regardless of "
                "measured motion. 50 is 5 s at 10 Hz. A backstop, not the normal "
                "path: distance and rotation should trigger keyframes. It exists "
                "because any fault that suppresses the measured motion would "
                "otherwise freeze the reference forever, which is exactly how a "
                "missing wheel-odometry seed pinned the published pose to the "
                "origin for an entire run."
            ),
        )
        self.declare_parameter("max_correspondence_distance", 0.50, _describe("ICP association cutoff, metres."))
        self.declare_parameter("max_iterations", 30, _describe("ICP iteration cap."))
        self.declare_parameter(
            "trim_ratio",
            0.95,
            _describe(
                "Fraction of correspondences kept per iteration. Must stay high: "
                "trimming discards the racking uprights, which are the only "
                "points constraining motion along a corridor of parallel walls."
            ),
        )
        self.declare_parameter("range_min", 0.10, _describe("Discard returns closer than this, metres."))
        self.declare_parameter("range_max", 20.0, _describe("Discard returns further than this, metres."))
        self.declare_parameter(
            "base_position_stddev",
            0.02,
            _describe(
                "Position standard deviation reported at perfect fitness, metres. "
                "Measured, not guessed: tests/test_scan_matcher.py reports the "
                "per-match error distribution this is taken from."
            ),
        )
        self.declare_parameter(
            "base_yaw_stddev", 0.01, _describe("Yaw standard deviation at perfect fitness, radians.")
        )
        self.declare_parameter(
            "use_odom_guess",
            True,
            _describe(
                "Seed ICP from wheel odometry. Strongly recommended: ICP's basin "
                "of convergence is about half the racking bay pitch, and an "
                "unseeded match during a fast spin can land a whole bay out."
            ),
        )

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)

        # All keyframe and rejection policy lives in the core so that the
        # offline benchmark measures this exact code path.
        self._odometry = ScanOdometry(
            keyframe_distance=float(self.get_parameter("keyframe_distance").value),
            keyframe_rotation=float(self.get_parameter("keyframe_rotation").value),
            min_fitness=float(self.get_parameter("min_fitness").value),
            max_keyframe_age=int(self.get_parameter("max_keyframe_age").value),
            max_correspondence_distance=float(
                self.get_parameter("max_correspondence_distance").value
            ),
            max_iterations=int(self.get_parameter("max_iterations").value),
            trim_ratio=float(self.get_parameter("trim_ratio").value),
        )

        self._pub = self.create_publisher(Odometry, "~/scan_odom", 10)
        self.create_subscription(LaserScan, "~/scan", self._on_scan, SCAN_QOS)
        self.create_subscription(Odometry, "~/odom", self._on_odom, 20)

        self._last_stamp = None
        self._wheel_pose = None
        self._reported_matched = 0
        self._reported_rejected = 0
        self.create_timer(10.0, self._report_health)

    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._wheel_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _on_scan(self, msg: LaserScan) -> None:
        points = scan_to_points(
            np.asarray(msg.ranges, dtype=np.float64),
            float(msg.angle_min),
            float(msg.angle_increment),
            range_min=max(float(msg.range_min), float(self.get_parameter("range_min").value)),
            range_max=min(float(msg.range_max), float(self.get_parameter("range_max").value)),
        )

        guess_source = self._wheel_pose if self.get_parameter("use_odom_guess").value else None
        update = self._odometry.update(points, odom_pose=guess_source)
        dt = self._elapsed(msg.header.stamp)
        self._last_stamp = msg.header.stamp

        if update is None:
            # Either the very first scan, which only establishes a keyframe, or
            # a rejected match. Publishing nothing is the point: the EKF carries
            # the interval on wheel odometry and the IMU rather than being fed a
            # confident wrong answer it has no way to detect.
            self.get_logger().warn(
                "no scan-odometry update this cycle; leaving the interval to wheel odometry and the IMU",
                throttle_duration_sec=2.0,
            )
            return

        self._publish(msg.header.stamp, update.previous_pose, dt, update.match)

    def _elapsed(self, stamp) -> float:
        now = stamp.sec + stamp.nanosec * 1e-9
        if self._last_stamp is None:
            return 0.0
        previous = self._last_stamp.sec + self._last_stamp.nanosec * 1e-9
        return max(0.0, now - previous)

    def _publish(self, stamp, previous_pose, dt: float, result) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame

        x, y, yaw = self._odometry.pose
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = quat_from_yaw(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        if dt > 1e-6:
            # Body-frame velocity: rotate the world-frame delta back into base.
            dx = x - previous_pose[0]
            dy = y - previous_pose[1]
            c, s = math.cos(previous_pose[2]), math.sin(previous_pose[2])
            msg.twist.twist.linear.x = float((c * dx + s * dy) / dt)
            msg.twist.twist.linear.y = float((-s * dx + c * dy) / dt)
            msg.twist.twist.angular.z = float(wrap_angle(yaw - previous_pose[2]) / dt)

        # Inflate the reported uncertainty as fitness falls. A match that only
        # just cleared the gate is genuinely worse than a clean one, and telling
        # the EKF so is the whole point of having a covariance field.
        scale = 1.0 / max(result.fitness, 0.1)
        pos_var = (float(self.get_parameter("base_position_stddev").value) * scale) ** 2
        yaw_var = (float(self.get_parameter("base_yaw_stddev").value) * scale) ** 2

        cov = [0.0] * 36
        cov[0] = pos_var  # x
        cov[7] = pos_var  # y
        cov[14] = 1e6  # z, not observed
        cov[21] = 1e6  # roll, not observed
        cov[28] = 1e6  # pitch, not observed
        cov[35] = yaw_var
        msg.pose.covariance = cov

        twist_cov = [0.0] * 36
        twist_cov[0] = pos_var / max(dt, 0.05) ** 2
        twist_cov[7] = pos_var / max(dt, 0.05) ** 2
        twist_cov[14] = 1e6
        twist_cov[21] = 1e6
        twist_cov[28] = 1e6
        twist_cov[35] = yaw_var / max(dt, 0.05) ** 2
        msg.twist.covariance = twist_cov

        self._pub.publish(msg)

    def _report_health(self) -> None:
        matched = self._odometry.matched - self._reported_matched
        rejected = self._odometry.rejected - self._reported_rejected
        total = matched + rejected
        if total == 0:
            self.get_logger().warn("no scans received in the last 10 s")
            return
        self.get_logger().info(
            "scan odometry: %d matched, %d rejected (%.1f%% reject rate)"
            % (matched, rejected, 100.0 * rejected / total)
        )
        self._reported_matched = self._odometry.matched
        self._reported_rejected = self._odometry.rejected


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanOdometryNode()
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
