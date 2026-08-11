"""Record estimated and ground-truth poses to CSV for offline evaluation.

Writes one CSV per source in the format ``analysis/localisation_error.py``
expects. Sources are recorded independently, at whatever rate each publishes,
and time-aligned offline rather than here — resampling in the logger would
throw away information and bake one alignment choice into the raw data.

THE GROUND-TRUTH RULE

``/ground_truth/pose`` comes from Gazebo's p3d plugin and is the simulator's
exact answer. This node is the only thing in the stack permitted to subscribe to
it. Nothing in localisation, navigation or perception may, and
``--assert-no-cheating`` checks that at runtime by counting publishers and
subscribers on the topic: if anything other than this logger is listening, the
run is not a valid evaluation and the node says so loudly rather than producing
numbers that look excellent for the wrong reason.
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, TextIO

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from amr_common.transforms import yaw_from_quat

BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
)

HEADER = ["t", "x", "y", "yaw", "cov_xx", "cov_yy", "cov_yawyaw"]


class _CsvSink:
    """One open CSV file plus its writer."""

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._handle: TextIO = open(path, "w", encoding="utf-8", newline="")
        self._writer = csv.writer(self._handle)
        self._writer.writerow(HEADER)
        self.rows = 0

    def write(self, t: float, x: float, y: float, yaw: float, cxx: float, cyy: float, cyaw: float) -> None:
        self._writer.writerow(
            ["%.6f" % t, "%.6f" % x, "%.6f" % y, "%.6f" % yaw, "%.9f" % cxx, "%.9f" % cyy, "%.9f" % cyaw]
        )
        self.rows += 1

    def close(self) -> None:
        # Flushed on every close rather than relying on interpreter shutdown: a
        # run killed with Ctrl-C is the normal way these end, and a truncated
        # final line makes the CSV unparseable.
        self._handle.flush()
        self._handle.close()


class PoseLogger(Node):
    def __init__(self) -> None:
        super().__init__("pose_logger")

        self.declare_parameter(
            "output_dir", "results", ParameterDescriptor(description="Directory for the CSV logs.")
        )
        self.declare_parameter(
            "assert_no_cheating",
            True,
            ParameterDescriptor(
                description=(
                    "Periodically verify that nothing except this node subscribes "
                    "to the ground-truth topic. Disabling it makes the evaluation "
                    "untrustworthy and should only be done when deliberately "
                    "replaying a bag."
                )
            ),
        )
        self.declare_parameter("ground_truth_topic", "/ground_truth/pose")

        output_dir = Path(str(self.get_parameter("output_dir").value))
        self._gt_topic = str(self.get_parameter("ground_truth_topic").value)

        self._sinks: Dict[str, _CsvSink] = {
            "ground_truth": _CsvSink(output_dir / "ground_truth.csv"),
            "amcl": _CsvSink(output_dir / "amcl_pose.csv"),
            "ekf": _CsvSink(output_dir / "ekf_odom.csv"),
            "wheel": _CsvSink(output_dir / "wheel_odom.csv"),
        }

        self.create_subscription(Odometry, self._gt_topic, self._on_ground_truth, BEST_EFFORT)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_ekf, 20)
        self.create_subscription(Odometry, "/odom", self._on_wheel, 20)

        if self.get_parameter("assert_no_cheating").value:
            self.create_timer(5.0, self._check_no_cheating)
        self.create_timer(10.0, self._report)

        self.get_logger().info("pose_logger writing to %s" % output_dir.resolve())

    # ------------------------------------------------------------------
    @staticmethod
    def _stamp(header) -> float:
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def _on_ground_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._sinks["ground_truth"].write(
            self._stamp(msg.header), p.position.x, p.position.y,
            yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
            0.0, 0.0, 0.0,
        )

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        cov = msg.pose.covariance
        self._sinks["amcl"].write(
            self._stamp(msg.header), p.position.x, p.position.y,
            yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
            float(cov[0]), float(cov[7]), float(cov[35]),
        )

    def _on_ekf(self, msg: Odometry) -> None:
        p = msg.pose.pose
        cov = msg.pose.covariance
        self._sinks["ekf"].write(
            self._stamp(msg.header), p.position.x, p.position.y,
            yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
            float(cov[0]), float(cov[7]), float(cov[35]),
        )

    def _on_wheel(self, msg: Odometry) -> None:
        p = msg.pose.pose
        cov = msg.pose.covariance
        self._sinks["wheel"].write(
            self._stamp(msg.header), p.position.x, p.position.y,
            yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
            float(cov[0]), float(cov[7]), float(cov[35]),
        )

    # ------------------------------------------------------------------
    def _check_no_cheating(self) -> None:
        """Fail loudly if anything but this node reads ground truth."""
        try:
            subscribers = self.get_subscriptions_info_by_topic(self._gt_topic)
        except Exception:  # pragma: no cover - depends on rmw support
            return

        offenders = [s.node_name for s in subscribers if s.node_name != self.get_name()]
        if offenders:
            self.get_logger().error(
                "GROUND TRUTH LEAK: %s subscribe to %s. This run is not a valid "
                "localisation evaluation — the stack can see the answer."
                % (", ".join(sorted(set(offenders))), self._gt_topic)
            )

    def _report(self) -> None:
        counts = ", ".join("%s=%d" % (name, sink.rows) for name, sink in sorted(self._sinks.items()))
        self.get_logger().info("rows logged: %s" % counts)
        if self._sinks["ground_truth"].rows == 0:
            self.get_logger().warn(
                "no ground truth received on %s; check the p3d plugin is in the URDF" % self._gt_topic
            )

    def close(self) -> None:
        for sink in self._sinks.values():
            sink.close()
        self.get_logger().info("closed %d CSV logs" % len(self._sinks))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
