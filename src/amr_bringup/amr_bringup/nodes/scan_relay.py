#!/usr/bin/env python3
"""Republish the 2D LiDAR, with the ability to go quiet on command.

This exists for one induced failure: blinding the LiDAR mid-aisle. A Gazebo
sensor cannot be stopped and restarted from inside a run — making it stop
publishing means unloading its plugin, which is not recoverable — so the scan
is passed through a relay that can simply stop forwarding.

The relay is **not** in the path on a normal run. `simulation.launch.py` leaves
the LiDAR publishing straight to `scan`, and only `run_faults.sh` moves it to
`scan_raw` and interposes this node. That matters: the localisation figures in
the docs were measured without a relay, and a Python hop at 10 Hz in front of
AMCL is not something to add to every run for the sake of one fault.

The previous implementation of this fault was `pkill -STOP -f gzserver`, which
suspends the entire simulator. Under `use_sim_time` that stops the clock, so
every node in the stack stops with it and nothing observes a dropout at all. It
tested nothing.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def sensor_qos(depth: int = 10) -> QoSProfile:
    """Best-effort, matching what the Gazebo sensor plugin publishes with.

    A reliable subscriber never matches a best-effort publisher, and the
    symptom is a topic that lists fine and delivers nothing.
    """
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class ScanRelay(Node):
    def __init__(self) -> None:
        super().__init__("scan_relay")

        self.declare_parameter("input_topic", "scan_raw")
        self.declare_parameter("output_topic", "scan")
        self.declare_parameter("control_topic", "scan_relay/blind")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        control_topic = str(self.get_parameter("control_topic").value)

        self._blind = False
        self._forwarded = 0
        self._dropped = 0

        self._pub = self.create_publisher(LaserScan, output_topic, sensor_qos())
        self.create_subscription(LaserScan, input_topic, self._on_scan, sensor_qos())
        # Reliable and transient-local: the injector publishes this once, and a
        # dropped command would leave the robot blind for the rest of the run.
        self.create_subscription(
            Bool,
            control_topic,
            self._on_control,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.get_logger().info("scan_relay: %s -> %s (control %s)"
                               % (input_topic, output_topic, control_topic))

    def _on_control(self, msg: Bool) -> None:
        if msg.data == self._blind:
            return
        self._blind = bool(msg.data)
        self.get_logger().warn("scan_relay %s" % ("BLIND" if self._blind else "restored"))

    def _on_scan(self, msg: LaserScan) -> None:
        if self._blind:
            self._dropped += 1
            return
        self._forwarded += 1
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("scan_relay: forwarded %d, dropped %d"
                               % (node._forwarded, node._dropped))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
