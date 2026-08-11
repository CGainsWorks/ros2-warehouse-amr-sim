"""Induce the failures that the recovery behaviours exist to handle.

Recovery behaviours that have never been provoked are decoration. This node
schedules specific, repeatable faults during a mission and records exactly when
each was applied, so that ``analysis/recovery_matrix.py`` can line the fault up
against the recovery the behaviour tree actually chose.

FAULTS AND WHAT EACH IS MEANT TO PROVOKE

``block_aisle``
    Teleport a pallet into the robot's path. Provokes the local costmap to
    invalidate the current plan; expected response is a replan, escalating to a
    costmap clear and a spin if the obstacle is not resolvable from the current
    viewpoint.

``kidnap``
    Teleport the robot itself several metres down the aisle without telling the
    localisation stack. This is the hardest case in this environment: the
    racking repeats every 2.7 m, so a displacement that is a whole number of
    bays leaves AMCL with a perfectly plausible — and wrong — pose. Expected
    response is a growing particle spread and, if configured, a global
    relocalisation.

``sensor_dropout``
    Stop republishing the LiDAR for a few seconds. Provokes the costmap's
    ``expected_update_rate`` watchdog and the scan matcher's rejection path.
    Expected response is that the robot slows or stops rather than continuing on
    stale data.

``localisation_jump``
    Publish a deliberately wrong ``/initialpose``, one racking bay from truth.
    Provokes exactly the aliasing failure the environment is designed around.

Every fault is applied through the simulator's own service interface and logged
with a simulation timestamp. Faults are scheduled by time rather than by robot
position so that a run is reproducible even when the robot's own behaviour
changes in response to them.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import List

import rclpy
import yaml
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_srvs.srv import Empty

from amr_common.transforms import quat_from_yaw


class FaultInjector(Node):
    def __init__(self) -> None:
        super().__init__("fault_injector")

        self.declare_parameter(
            "schedule_file", "", ParameterDescriptor(description="YAML fault schedule.")
        )
        self.declare_parameter("output", "results/faults.json")

        path = str(self.get_parameter("schedule_file").value)
        if not path:
            raise RuntimeError("schedule_file parameter is required")
        self._schedule = self._load_schedule(path)

        self._applied: List[dict] = []
        self._pending = list(self._schedule)
        self._start = self._now()

        self._set_state = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._relocalise = self.create_client(Empty, "/reinitialize_global_localization")

        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            "fault injector armed with %d faults: %s"
            % (len(self._schedule), ", ".join("%s@%.0fs" % (f["type"], f["at"]) for f in self._schedule))
        )

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _load_schedule(self, path: str) -> List[dict]:
        with open(path, "r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
        faults = sorted(raw.get("faults", []), key=lambda f: float(f["at"]))
        if not faults:
            raise ValueError("%s: no faults defined" % path)
        return faults

    def _tick(self) -> None:
        elapsed = self._now() - self._start
        while self._pending and elapsed >= float(self._pending[0]["at"]):
            fault = self._pending.pop(0)
            self._apply(fault, elapsed)

        if not self._pending and self._applied:
            self._write()

    def _apply(self, fault: dict, elapsed: float) -> None:
        kind = str(fault["type"])
        self.get_logger().warn("INJECTING FAULT %s at t=%.1f s" % (kind, elapsed))

        detail = "applied"
        try:
            if kind in ("block_aisle", "kidnap"):
                detail = self._teleport(fault)
            elif kind == "localisation_jump":
                detail = self._localisation_jump(fault)
            elif kind == "relocalise":
                detail = self._request_relocalisation()
            elif kind == "sensor_dropout":
                # Handled outside this node: the launch file wraps the LiDAR
                # topic in a relay that scripts/inject_faults.sh stops and
                # restarts. A node cannot make a Gazebo sensor stop publishing
                # without unloading the plugin, and unloading it is not
                # recoverable within a run.
                detail = "signalled (external relay)"
            else:
                detail = "unknown fault type, skipped"
                self.get_logger().error("unknown fault type %r" % kind)
        except Exception as exc:  # keep the mission running; the fault is the point
            detail = "failed: %s" % exc
            self.get_logger().error("fault %s failed: %s" % (kind, exc))

        self._applied.append(
            {"t": elapsed, "type": kind, "params": {k: v for k, v in fault.items() if k != "at"}, "detail": detail}
        )

    def _teleport(self, fault: dict) -> str:
        if not self._set_state.wait_for_service(timeout_sec=2.0):
            return "failed: /gazebo/set_entity_state unavailable"

        request = SetEntityState.Request()
        request.state.name = str(fault["model"])
        request.state.reference_frame = "world"
        request.state.pose.position.x = float(fault["x"])
        request.state.pose.position.y = float(fault["y"])
        request.state.pose.position.z = float(fault.get("z", 0.0))
        qx, qy, qz, qw = quat_from_yaw(float(fault.get("yaw", 0.0)))
        request.state.pose.orientation.x = qx
        request.state.pose.orientation.y = qy
        request.state.pose.orientation.z = qz
        request.state.pose.orientation.w = qw

        self._set_state.call_async(request)
        return "teleported %s to (%.2f, %.2f)" % (request.state.name, request.state.pose.position.x,
                                                  request.state.pose.position.y)

    def _localisation_jump(self, fault: dict) -> str:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(fault["x"])
        msg.pose.pose.position.y = float(fault["y"])
        qx, qy, qz, qw = quat_from_yaw(float(fault.get("yaw", 0.0)))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Confident covariance, deliberately: a tentative bad pose would be
        # corrected almost immediately and would not test anything.
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05
        cov[35] = 0.02
        msg.pose.covariance = cov
        self._initial_pose_pub.publish(msg)
        return "published wrong /initialpose at (%.2f, %.2f)" % (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _request_relocalisation(self) -> str:
        if not self._relocalise.wait_for_service(timeout_sec=2.0):
            return "failed: /reinitialize_global_localization unavailable"
        self._relocalise.call_async(Empty.Request())
        return "requested global relocalisation"

    def _write(self) -> None:
        out_path = Path(str(self.get_parameter("output").value))
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps({"faults": self._applied}, indent=2), encoding="utf-8")
        self.get_logger().info("wrote %s (%d faults)" % (out_path, len(self._applied)))
        self._applied = []  # only write once


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaultInjector()
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
