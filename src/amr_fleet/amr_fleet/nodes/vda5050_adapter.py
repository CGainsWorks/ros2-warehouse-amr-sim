"""VDA 5050 adapter: MQTT on one side, Nav2 on the other.

Subscribes to ``uagv/v2/<manufacturer>/<serialNumber>/order`` and
``.../instantActions``, drives each released node of an accepted order with
Nav2's ``navigate_to_pose`` action, and publishes ``.../state`` once a second
and on every change. ``.../connection`` carries ONLINE (retained) and the MQTT
last will CONNECTIONBROKEN.

All the protocol decisions are in :mod:`amr_fleet.core.vda5050`, which has no
ROS or MQTT imports and is unit tested offline. This node only moves messages.

Threading: paho runs its network loop on its own thread, so incoming MQTT
messages are queued and handled on the ROS executor by a timer. Nothing on the
paho thread touches the vehicle state or ROS.
"""

from __future__ import annotations

import json
import queue

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from amr_common.transforms import quat_from_yaw, yaw_from_quat
from amr_fleet.core.vda5050 import Vehicle, topic


def _mqtt_client(client_id: str):
    """A paho client on either the 1.x or the 2.x API (noble ships 1.6)."""
    import paho.mqtt.client as mqtt

    if hasattr(mqtt, "CallbackAPIVersion"):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=client_id)
    return mqtt.Client(client_id=client_id)


class Vda5050Adapter(Node):
    def __init__(self) -> None:
        super().__init__("vda5050_adapter")
        self.declare_parameter("broker_host", "localhost")
        self.declare_parameter("broker_port", 1883)
        self.declare_parameter("manufacturer", "CGainsWorks")
        self.declare_parameter("serial_number", "amr-001")
        self.declare_parameter("map_id", "warehouse")
        self.declare_parameter("state_period", 1.0)

        param = self.get_parameter
        self._vehicle = Vehicle(str(param("manufacturer").value), str(param("serial_number").value),
                                str(param("map_id").value))
        manufacturer, serial = self._vehicle.manufacturer, self._vehicle.serial_number
        self._topics = {name: topic(manufacturer, serial, name)
                        for name in ("order", "instantActions", "state", "connection")}

        self._inbox: "queue.Queue[tuple]" = queue.Queue()
        self._dirty = True
        self._goal_handle = None
        self._goal_node_id = ""

        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # The reported position is map -> base_footprint from TF: AMCL's
        # correction composed with the EKF's odometry, which is what Nav2 itself
        # drives on. /amcl_pose alone is only published when AMCL updates, every
        # 0.2 m of travel, so a robot that has just stopped would report where
        # it was up to 0.2 m ago; measured, the state put the vehicle 0.35 m
        # from a node Nav2 had correctly reached. /amcl_pose stays as the
        # fallback until TF is available. Transient local, matching AMCL's
        # publisher, so a late-joining adapter still gets the initial pose.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_pose,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 10)

        self._mqtt = _mqtt_client("vda5050-%s-%s" % (manufacturer, serial))
        self._mqtt.will_set(self._topics["connection"],
                            json.dumps(self._vehicle.connection("CONNECTIONBROKEN")), qos=1, retain=True)
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_message = self._on_message
        host, port = str(param("broker_host").value), int(param("broker_port").value)
        self._mqtt.connect_async(host, port, keepalive=30)
        self._mqtt.loop_start()
        self.get_logger().info("VDA 5050 adapter for %s/%s, broker %s:%d"
                               % (manufacturer, serial, host, port))

        self.create_timer(0.1, self._process_inbox)
        self.create_timer(float(param("state_period").value), self._publish_state)

    # ------------------------------------------------------------ paho thread
    def _on_connect(self, client, userdata, flags, rc) -> None:
        if rc != 0:
            return
        client.subscribe(self._topics["order"], qos=0)
        client.subscribe(self._topics["instantActions"], qos=0)
        self._inbox.put(("connected", None))

    def _on_message(self, client, userdata, message) -> None:
        try:
            payload = json.loads(message.payload.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            payload = None
        self._inbox.put((message.topic, payload))

    # ------------------------------------------------------------ ROS thread
    def _process_inbox(self) -> None:
        while True:
            try:
                kind, payload = self._inbox.get_nowait()
            except queue.Empty:
                break
            if kind == "connected":
                self._mqtt.publish(self._topics["connection"],
                                   json.dumps(self._vehicle.connection("ONLINE")), qos=1, retain=True)
                self.get_logger().info("connected to the broker; ONLINE")
            elif kind == self._topics["order"]:
                self._handle_order(payload)
            elif kind == self._topics["instantActions"]:
                self._handle_instant_actions(payload)
            self._dirty = True
        if self._dirty:
            self._publish_state()

    def _handle_order(self, payload) -> None:
        self._refresh_position()
        order = self._vehicle.receive_order(payload if payload is not None else "not json")
        if order is None:
            if self._vehicle.errors:
                self.get_logger().warn("order refused: %s" % self._vehicle.errors[-1]["errorDescription"])
            return
        self.get_logger().info("order %s accepted: %d nodes" % (order.order_id, len(order.nodes)))
        self._drive_next()

    def _handle_instant_actions(self, payload) -> None:
        executed = self._vehicle.receive_instant_actions(payload)
        if "cancelOrder" in executed and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            self.get_logger().info("order cancelled")

    def _drive_next(self) -> None:
        node = self._vehicle.next_goal()
        if node is None:
            self.get_logger().info("order %s complete" % self._vehicle.order_id)
            self._dirty = True
            return
        if not self._nav.wait_for_server(timeout_sec=1.0):
            self._vehicle.navigation_failed(node.node_id, "navigate_to_pose is not available")
            self._dirty = True
            return
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = node.position.x
        pose.pose.position.y = node.position.y
        theta = node.position.theta
        if theta is None:  # no heading requested: keep the current one
            theta = self._vehicle.position[2] if self._vehicle.position else 0.0
        qx, qy, qz, qw = quat_from_yaw(theta)
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        goal.pose = pose
        self._goal_node_id = node.node_id
        self._vehicle.driving = True
        self._dirty = True
        self.get_logger().info("driving to node %s (%.2f, %.2f)" % (node.node_id, node.position.x,
                                                                    node.position.y))
        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self._vehicle.navigation_failed(self._goal_node_id, "Nav2 rejected the goal")
            self._dirty = True
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        status = future.result().status
        self._goal_handle = None
        node_id = self._goal_node_id
        self._dirty = True
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self._vehicle.node_states and self._vehicle.node_states[0].node_id == node_id:
                self._vehicle.node_reached(node_id)
                self.get_logger().info("node %s reached" % node_id)
                self._drive_next()
            return
        if status == GoalStatus.STATUS_CANCELED:
            return
        self._vehicle.navigation_failed(node_id, "navigation to node %s ended with status %d"
                                        % (node_id, status))
        self.get_logger().error("navigation to node %s failed (status %d)" % (node_id, status))

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self._vehicle.position = (p.position.x, p.position.y,
                                  yaw_from_quat(p.orientation.x, p.orientation.y,
                                                p.orientation.z, p.orientation.w))

    def _on_odom(self, msg: Odometry) -> None:
        t = msg.twist.twist
        self._vehicle.velocity = (t.linear.x, t.linear.y, t.angular.z)

    def _refresh_position(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_footprint", Time())
        except TransformException:
            return  # keep the last /amcl_pose until TF is up
        t, q = tf.transform.translation, tf.transform.rotation
        self._vehicle.position = (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def _publish_state(self) -> None:
        self._dirty = False
        self._refresh_position()
        self._mqtt.publish(self._topics["state"], json.dumps(self._vehicle.state()), qos=0)

    def shutdown(self) -> None:
        try:
            self._mqtt.publish(self._topics["connection"],
                               json.dumps(self._vehicle.connection("OFFLINE")), qos=1, retain=True)
            self._mqtt.disconnect()
            self._mqtt.loop_stop()
        except Exception:  # pragma: no cover - best effort on the way down
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Vda5050Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
