"""Drive a fixed velocity profile with the navigation stack out of the loop.

WHY THIS EXISTS

Evaluating localisation during a Nav2 mission is circular. A localisation error
makes the costmap wrong, Nav2 then fails to progress and runs recovery spins,
the spins produce wheel slip, the slip corrupts odometry, and the corrupted
odometry makes localisation worse. Every run confounds the quality of the
estimator with the behaviour of the controller, and no parameter can be tuned
against a number that moves for reasons unrelated to the parameter.

Measured evidence that the loop is real: wheel-odometry yaw tracked ground truth
closely for the first 66 s of a mission run (-0.639 rad against -0.655), then
diverged to +3.385 against -0.654 once the recovery spins began. The 36%
over-reporting was not a property of the sensor, it was a consequence of the
failure it appeared to explain.

This node breaks the loop. It publishes a scripted velocity profile on a timer
and nothing reads any estimate, so the trajectory is identical run to run
regardless of how badly localisation performs. That makes the resulting ATE a
measurement of the estimator alone.

The profile deliberately mixes straight runs, in-place rotations and arcs.
In-place rotation is the case that matters most here: it is where wheel
odometry is most exposed, where scan matching has the least translational
signal, and where AMCL's motion model is worked hardest.

It also has to be physically feasible in a 2.8 m aisle, because open loop means
exactly that — nothing avoids obstacles. See the note on DEFAULT_PROFILE.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

# (label, linear m/s, angular rad/s, duration s).
#
# OPEN-LOOP TURNS ARE IMPRECISE, AND THE PROFILE MUST NOT DEPEND ON THEM.
#
# This is the constraint that took three attempts to get right. A commanded
# 180 degree turn does not deliver 180 degrees: acceleration limits and wheel
# slip left a measured 0.474 rad/s against 0.500 commanded, about 9 degrees
# short over the segment. Nothing corrects it, because open loop means no
# feedback. Drive 3.5 m after that and the 9 degrees becomes 0.55 m of lateral
# drift; in practice it reached 1.22 m and put the robot into the rack face at
# y = 1.35, where it sat jammed for the remaining 25 s of the run.
#
# So the profile never turns and then drives far. Instead:
#
#   * Translation is tested with forward/reverse along the SAME heading, which
#     cannot drift sideways however imprecise the wheels are.
#   * Rotation is tested with paired +theta/-theta in-place turns, which cancel:
#     any error in the first is largely undone by the second, and neither
#     translates.
#   * Arcs are short, paired left-right-left, and followed by a reverse rather
#     than a straight run.
#
# The result stays within x in [-12.0, -8.5] and |y| < 0.10 even with a 10%
# turn-rate error applied in either direction. tests/test_openloop_profile.py integrates it with a
# deliberate heading perturbation injected after every rotation and asserts the
# clearance still holds, so the profile is checked against the error that
# actually occurs rather than against a perfect world.
DEFAULT_PROFILE = [
    ("settle", 0.00, 0.00, 3.0),
    ("forward 3.5 m", 0.40, 0.00, 8.75),
    ("reverse 3.5 m", -0.40, 0.00, 8.75),
    ("rotate +180", 0.00, 0.50, 6.3),
    ("rotate -180", 0.00, -0.50, 6.3),
    ("forward 1.5 m", 0.40, 0.00, 3.75),
    ("arc left", 0.30, 0.10, 2.0),
    ("arc right", 0.30, -0.20, 2.0),
    ("arc left", 0.30, 0.10, 2.0),
    ("reverse 2 m", -0.40, 0.00, 5.0),
    ("rotate +90", 0.00, 0.50, 3.15),
    ("rotate -90", 0.00, -0.50, 3.15),
    ("stop", 0.00, 0.00, 3.0),
]


class OpenLoopDriver(Node):
    def __init__(self) -> None:
        super().__init__("openloop_driver")
        self.declare_parameter(
            "rate_hz", 20.0, ParameterDescriptor(description="Command publish rate.")
        )
        self.declare_parameter(
            "speed_scale",
            1.0,
            ParameterDescriptor(
                description=(
                    "Scales every velocity in the profile. Durations are NOT scaled, "
                    "so this changes the distance covered, not the sequence."
                )
            ),
        )

        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._profile = list(DEFAULT_PROFILE)
        self._scale = float(self.get_parameter("speed_scale").value)

        self._index = 0
        self._segment_started = None
        self._announced = None

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)

        total = sum(seg[3] for seg in self._profile)
        self.get_logger().info(
            "open-loop driver: %d segments, %.0f s total, speed scale %.2f"
            % (len(self._profile), total, self._scale)
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        now = self._now()
        if self._segment_started is None:
            self._segment_started = now

        if self._index >= len(self._profile):
            self._stop()
            self.get_logger().info("profile complete")
            raise SystemExit(0)

        label, linear, angular, duration = self._profile[self._index]
        if self._announced != self._index:
            self.get_logger().info(
                "segment %d/%d: %s (v=%.2f w=%.2f for %.0fs)"
                % (self._index + 1, len(self._profile), label,
                   linear * self._scale, angular * self._scale, duration)
            )
            self._announced = self._index

        if now - self._segment_started >= duration:
            self._index += 1
            self._segment_started = now
            return

        msg = Twist()
        msg.linear.x = linear * self._scale
        msg.angular.z = angular * self._scale
        self._pub.publish(msg)

    def _stop(self) -> None:
        # Several times, because a single message on a best-effort path can be
        # lost and a robot left driving into a wall after the test ends is a
        # poor way to end a test.
        for _ in range(5):
            self._pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenLoopDriver()
    code = 0
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit) as exc:
        code = getattr(exc, "code", 0) or 0
    finally:
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
