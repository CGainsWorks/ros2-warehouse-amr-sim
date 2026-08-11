"""Drive a sequence of pick faces and record what happened.

Sends the waypoint list to Nav2's ``NavigateThroughPoses`` action, watches the
behaviour tree log for recovery activity, and writes a JSON mission record that
``analysis/recovery_matrix.py`` turns into the recovery behaviour matrix.

Recovery events are taken from ``/behavior_tree_log`` rather than inferred from
robot motion. Inferring "it must have been recovering because it stopped" is
guesswork that cannot distinguish a Spin recovery from a legitimate in-place
turn onto a new heading, and the whole point of the matrix is to say which
recovery fired and whether it worked.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.msg import BehaviorTreeLog
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.node import Node

from amr_common.transforms import quat_from_yaw

# BT node names that represent a recovery having been invoked. Taken from the
# names in amr_navigation/behavior_trees/*.xml; if those are renamed this must
# follow, which is why they are spelled out rather than pattern-matched.
RECOVERY_NODES = {
    "ClearLocalCostmap-Recovery": "clear_local_costmap",
    "ClearGlobalCostmap-Recovery": "clear_global_costmap",
    "ClearLocalCostmap-Context": "clear_local_costmap_context",
    "ClearGlobalCostmap-Context": "clear_global_costmap_context",
    "Spin": "spin",
    "BackUp": "back_up",
    "Wait": "wait",
}


class MissionRunner(Node):
    def __init__(self) -> None:
        super().__init__("mission_runner")

        self.declare_parameter(
            "waypoints_file", "", ParameterDescriptor(description="YAML list of goal poses.")
        )
        self.declare_parameter("output", "results/mission.json")
        self.declare_parameter(
            "timeout_sec",
            300.0,
            ParameterDescriptor(description="Abort the mission after this long."),
        )

        path = str(self.get_parameter("waypoints_file").value)
        if not path:
            raise RuntimeError("waypoints_file parameter is required")
        self._waypoints = self._load_waypoints(path)
        self.get_logger().info("loaded %d waypoints from %s" % (len(self._waypoints), path))

        self._events: List[dict] = []
        self._recovery_counts: Dict[str, int] = {}
        self._feedback_samples = 0
        self._result: Optional[dict] = None

        self.create_subscription(BehaviorTreeLog, "/behavior_tree_log", self._on_bt_log, 20)
        self._client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")

        self._start_time = self._now()
        self.create_timer(1.0, self._tick)
        self._sent = False
        self._goal_handle = None

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _load_waypoints(self, path: str) -> List[PoseStamped]:
        with open(path, "r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
        if not isinstance(raw, dict) or "waypoints" not in raw:
            raise ValueError("%s: expected a top-level 'waypoints' key" % path)

        poses = []
        for entry in raw["waypoints"]:
            pose = PoseStamped()
            pose.header.frame_id = str(entry.get("frame_id", "map"))
            pose.pose.position.x = float(entry["x"])
            pose.pose.position.y = float(entry["y"])
            qx, qy, qz, qw = quat_from_yaw(float(entry.get("yaw", 0.0)))
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            poses.append(pose)
        if not poses:
            raise ValueError("%s: no waypoints defined" % path)
        return poses

    def _on_bt_log(self, msg: BehaviorTreeLog) -> None:
        for event in msg.event_log:
            # IDLE -> RUNNING is the transition that means "this node has just
            # been invoked". Counting every status change instead would count a
            # single recovery several times as it ticks.
            if event.previous_status == "IDLE" and event.current_status == "RUNNING":
                kind = RECOVERY_NODES.get(event.node_name)
                if kind is None:
                    continue
                self._recovery_counts[kind] = self._recovery_counts.get(kind, 0) + 1
                self._events.append(
                    {
                        "t": self._now() - self._start_time,
                        "node": event.node_name,
                        "recovery": kind,
                    }
                )
                self.get_logger().warn("recovery invoked: %s" % event.node_name)

    def _tick(self) -> None:
        if not self._sent:
            if not self._client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info("waiting for navigate_through_poses action server")
                return
            goal = NavigateThroughPoses.Goal()
            goal.poses = self._waypoints
            self._client.send_goal_async(goal, feedback_callback=self._on_feedback).add_done_callback(
                self._on_goal_response
            )
            self._sent = True
            self._start_time = self._now()
            self.get_logger().info("mission sent: %d waypoints" % len(self._waypoints))
            return

        elapsed = self._now() - self._start_time
        if self._result is None and elapsed > float(self.get_parameter("timeout_sec").value):
            self.get_logger().error("mission timed out after %.0f s" % elapsed)
            self._finish("timeout", elapsed)

    def _on_feedback(self, feedback) -> None:
        self._feedback_samples += 1

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("mission goal rejected by Nav2")
            self._finish("rejected", self._now() - self._start_time)
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        status = future.result().status
        outcome = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled",
        }.get(status, "unknown(%d)" % status)
        self._finish(outcome, self._now() - self._start_time)

    def _finish(self, outcome: str, elapsed: float) -> None:
        if self._result is not None:
            return
        self._result = {
            "outcome": outcome,
            "duration_s": elapsed,
            "waypoints": len(self._waypoints),
            "recovery_counts": dict(self._recovery_counts),
            "recovery_events": self._events,
            "feedback_samples": self._feedback_samples,
        }
        out_path = Path(str(self.get_parameter("output").value))
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(self._result, indent=2), encoding="utf-8")

        self.get_logger().info(
            "mission %s in %.1f s; recoveries: %s"
            % (outcome, elapsed, self._recovery_counts or "none")
        )
        self.get_logger().info("wrote %s" % out_path)
        raise SystemExit(0 if outcome == "succeeded" else 1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionRunner()
    exit_code = 0
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit) as exc:
        exit_code = getattr(exc, "code", 0) or 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
