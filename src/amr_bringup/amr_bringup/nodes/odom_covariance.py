"""Stamp a covariance onto the bridged wheel odometry and republish it.

gz-sim's DiffDrive publishes ``gz.msgs.Odometry``, which has no covariance
field, so ros_gz_bridge delivers a ``nav_msgs/Odometry`` whose covariance is
all zeros. robot_localization clamps a zero variance to 1e-9, which means the
EKF would treat the wheels as perfect and ignore everything else. Gazebo
Classic's diff drive plugin advertised a covariance itself; this node restores
the same values, laid out the same way:

* x, y and yaw from parameters (``config/odom_covariance.yaml``);
* z, roll and pitch at 1e6, "not measured", as the Classic plugin did;
* the same diagonal on the twist.

The layout logic is in :func:`odometry_covariance`, which has no ROS imports so
the unit tests can check it.
"""

from __future__ import annotations

from typing import List

UNMEASURED = 1.0e6


def odometry_covariance(cov_x: float, cov_y: float, cov_yaw: float) -> List[float]:
    """Row-major 6x6 covariance with the given x, y and yaw variances."""
    for name, value in (("cov_x", cov_x), ("cov_y", cov_y), ("cov_yaw", cov_yaw)):
        if not value > 0.0:
            raise ValueError("%s must be positive, got %r" % (name, value))
    matrix = [0.0] * 36
    for index, value in enumerate((cov_x, cov_y, UNMEASURED, UNMEASURED, UNMEASURED, cov_yaw)):
        matrix[index * 7] = float(value)
    return matrix


def main(args=None) -> None:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    class OdomCovariance(Node):
        def __init__(self) -> None:
            super().__init__("odom_covariance")
            self.declare_parameter("input_topic", "/odom_raw")
            self.declare_parameter("output_topic", "/odom")
            self.declare_parameter("covariance_x", 0.001)
            self.declare_parameter("covariance_y", 0.001)
            self.declare_parameter("covariance_yaw", 0.01)
            self._covariance = odometry_covariance(
                float(self.get_parameter("covariance_x").value),
                float(self.get_parameter("covariance_y").value),
                float(self.get_parameter("covariance_yaw").value),
            )
            self._pub = self.create_publisher(
                Odometry, str(self.get_parameter("output_topic").value), 20
            )
            self.create_subscription(
                Odometry, str(self.get_parameter("input_topic").value), self._on_odom, 20
            )

        def _on_odom(self, msg: Odometry) -> None:
            msg.pose.covariance = self._covariance
            msg.twist.covariance = self._covariance
            self._pub.publish(msg)

    rclpy.init(args=args)
    node = OdomCovariance()
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
