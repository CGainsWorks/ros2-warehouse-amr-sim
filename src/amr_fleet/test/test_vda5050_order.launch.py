"""Headless integration test: a VDA 5050 order over MQTT, driven to completion in simulation.

Run with ``launch_test src/amr_fleet/test/test_vda5050_order.launch.py`` from a
built and sourced workspace with ``mosquitto`` on the PATH (CI installs it from
apt). The launch starts a private Mosquitto broker on a non-default port,
gz-sim with the warehouse and the full stack, and the VDA 5050 adapter. The
test then plays the master control:

1. waits for ``connection`` ONLINE and a ``state`` with an initialised position;
2. waits for Nav2 to be active (an order sent earlier would fail on a goal the
   navigator cannot yet accept, which is correct behaviour but not this test);
3. publishes a three-node order: the node the vehicle stands on, a node
   abreast of the first pick face, and one 1.5 m further on;
4. watches ``state`` until the order is reported complete: no node or edge
   states left, ``lastNodeId`` the final node, not driving, no errors.

It then checks the reported position against the final node and against
simulator ground truth. Every number this produces is simulated.
"""

from __future__ import annotations

import json
import math
import os
import queue
import time
import unittest
from pathlib import Path

import launch_testing
import launch_testing.actions
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from amr_fleet.core.vda5050 import OrderBuilder, order_complete, topic

BROKER_PORT = 18830
MANUFACTURER, SERIAL, MAP = "CGainsWorks", "amr-001", "warehouse"
ORDER_POSES = [(-12.0, 0.0, 0.0), (-9.45, 0.0, 0.0), (-7.95, 0.0, 0.0)]
RESULTS = Path(os.environ.get("AMR_TEST_RESULTS", "/tmp/amr_integration"))
# Half real time: every node gets twice the wall-clock budget per simulated
# second, which is what keeps a shared CI runner from starving Nav2.
SIM_RTF = os.environ.get("AMR_SIM_RTF", "0.5")
# A gz partition of its own. A gz server can outlive the launch that started
# it (ros_gz_sim runs it under a ruby wrapper, and killing the wrapper can
# orphan the server), and a second simulation on the same partition shares
# every gz topic with it: measured, the robot's ground truth jumped 15 m
# between two worlds and cmd_vel drove both robots.
GZ_PARTITION = "amr_vda5050_order_%d" % os.getpid()
NODE_TOLERANCE_M = 0.35
STARTUP_TIMEOUT_S = 600.0
ORDER_TIMEOUT_S = 900.0  # wall clock; the simulation runs below real time on CI


@pytest.mark.launch_test
def generate_test_description():
    broker = ExecuteProcess(
        cmd=["mosquitto", "-p", str(BROKER_PORT)],
        name="mosquitto",
        output="screen",
    )
    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("amr_fleet"), "launch", "fleet.launch.py"])
        ),
        launch_arguments={"broker_port": str(BROKER_PORT), "headless": "true",
                          "real_time_factor": SIM_RTF}.items(),
    )
    return (
        LaunchDescription([SetEnvironmentVariable("GZ_PARTITION", GZ_PARTITION), broker,
                           TimerAction(period=1.0, actions=[fleet]),
                           launch_testing.actions.ReadyToTest()]),
        {},
    )


def _mqtt_client(client_id):
    import paho.mqtt.client as mqtt

    if hasattr(mqtt, "CallbackAPIVersion"):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=client_id)
    return mqtt.Client(client_id=client_id)


class TestVda5050Order(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("vda5050_master_control_test")
        cls.truth = []
        cls.node.create_subscription(
            Odometry, "/ground_truth/pose",
            lambda m: cls.truth.append((m.pose.pose.position.x, m.pose.pose.position.y)),
            qos_profile_sensor_data)

        cls.messages = queue.Queue()
        cls.client = _mqtt_client("master-control-test")
        cls.client.on_message = lambda client, userdata, msg: cls.messages.put(
            (msg.topic, json.loads(msg.payload.decode("utf-8"))))
        deadline = time.time() + 60
        while True:
            try:
                cls.client.connect("localhost", BROKER_PORT, keepalive=30)
                break
            except OSError:
                if time.time() > deadline:
                    raise
                time.sleep(1)
        for name in ("state", "connection"):
            cls.client.subscribe(topic(MANUFACTURER, SERIAL, name), qos=0)
        cls.client.loop_start()

    @classmethod
    def tearDownClass(cls):
        cls.client.loop_stop()
        cls.client.disconnect()
        cls.node.destroy_node()
        rclpy.shutdown()

    def _next(self, predicate, timeout, what):
        deadline = time.time() + timeout
        last = None
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.0)
            try:
                name, payload = self.messages.get(timeout=0.5)
            except queue.Empty:
                continue
            if name.endswith("/state"):
                last = payload
            if predicate(name, payload):
                return payload
        self.fail("timed out waiting for %s; last state: %s" % (what, json.dumps(last)[:2000]))

    def _wait_for_nav2(self, timeout):
        client = self.node.create_client(GetState, "/bt_navigator/get_state")
        deadline = time.time() + timeout
        while time.time() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
                if future.result() is not None and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                    return
            time.sleep(2.0)
        self.fail("Nav2 did not become active")

    def test_order_is_driven_to_completion(self):
        started = time.time()
        self._next(lambda n, p: n.endswith("/connection") and p.get("connectionState") == "ONLINE",
                   STARTUP_TIMEOUT_S, "connection ONLINE")
        self._next(lambda n, p: n.endswith("/state") and p.get("agvPosition", {}).get("positionInitialized"),
                   STARTUP_TIMEOUT_S, "an initialised position")
        self._wait_for_nav2(STARTUP_TIMEOUT_S)

        order = OrderBuilder(MANUFACTURER, SERIAL, MAP, "it-order-1", ORDER_POSES).build()
        sent = time.time()
        self.client.publish(topic(MANUFACTURER, SERIAL, "order"), json.dumps(order), qos=0)

        accepted = self._next(lambda n, p: n.endswith("/state") and p.get("orderId") == "it-order-1",
                              60.0, "the order to be accepted")
        self.assertEqual(accepted["errors"], [], accepted)

        final = self._next(lambda n, p: n.endswith("/state") and (order_complete(p) or bool(p.get("errors"))),
                           ORDER_TIMEOUT_S, "the order to complete")
        self.assertTrue(order_complete(final), final)
        self.assertEqual(final["lastNodeId"], "n2")
        self.assertEqual(final["lastNodeSequenceId"], 4)

        goal = ORDER_POSES[-1]
        reported = final["agvPosition"]
        reported_error = math.hypot(reported["x"] - goal[0], reported["y"] - goal[1])
        self.assertLess(reported_error, NODE_TOLERANCE_M, reported)

        for _ in range(50):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(self.truth, "no ground truth received")
        truth = self.truth[-1]
        truth_error = math.hypot(truth[0] - goal[0], truth[1] - goal[1])
        self.assertLess(truth_error, NODE_TOLERANCE_M, "ground truth ends at (%.2f, %.2f), %.2f m from the node"
                        % (truth[0], truth[1], truth_error))

        RESULTS.mkdir(parents=True, exist_ok=True)
        with open(RESULTS / "vda5050_order.json", "w", encoding="utf-8", newline="\n") as handle:
            json.dump({
                "simulated": True,
                "test": "vda5050_order",
                "order": order,
                "final_state": final,
                "reported_error_m": reported_error,
                "ground_truth_error_m": truth_error,
                "wall_seconds_to_online": sent - started,
                "wall_seconds_order": time.time() - sent,
            }, handle, indent=2)
