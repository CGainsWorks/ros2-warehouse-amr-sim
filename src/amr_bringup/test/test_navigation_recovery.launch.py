"""Headless integration test: simulator, full stack, one mission, one induced failure.

Run with ``launch_test src/amr_bringup/test/test_navigation_recovery.launch.py``
from a built and sourced workspace (CI does exactly that, see
.github/workflows/ci.yml). It brings up gz-sim with the warehouse world and the
robot, the EKF, AMCL against the committed map, Nav2 and the perception nodes,
then:

1. sends a single-goal mission 10.65 m down the aisle (config/integration_mission.yaml);
2. ten seconds in, sets a pallet down on the destination and removes it eight
   seconds later (config/integration_fault.yaml), so the planner has no path
   and the behaviour tree must run its recovery ladder;
3. logs AMCL, EKF, wheel odometry and ground truth throughout.

After the launch shuts down it asserts that the mission succeeded, that the
fault fired and was cleared, that at least one recovery from the ladder ran,
that the robot finished on the goal according to ground truth, and that AMCL
tracked ground truth with an RMSE inside the bound below. A summary is written
to ``$AMR_TEST_RESULTS/navigation_recovery.json`` (default
``/tmp/amr_integration``) so CI can keep it as an artifact.

Every number this produces is simulated.
"""

from __future__ import annotations

import json
import math
import os
import sys
import unittest
from pathlib import Path

import launch_testing
import launch_testing.actions
import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

REPO_ROOT = Path(__file__).resolve().parents[3]
RESULTS = Path(os.environ.get("AMR_TEST_RESULTS", "/tmp/amr_integration")) / "navigation_recovery"
# Half real time: every node gets twice the wall-clock budget per simulated
# second, which is what keeps a shared CI runner from starving Nav2.
SIM_RTF = os.environ.get("AMR_SIM_RTF", "0.5")
# A gz partition of its own. A gz server can outlive the launch that started
# it (ros_gz_sim runs it under a ruby wrapper, and killing the wrapper can
# orphan the server), and a second simulation on the same partition shares
# every gz topic with it: measured, the robot's ground truth jumped 15 m
# between two worlds and cmd_vel drove both robots.
GZ_PARTITION = "amr_navigation_recovery_%d" % os.getpid()

GOAL = (-1.35, 0.0)
# Bounds the assertions use. Generous enough for a loaded CI runner, tight
# enough that a localiser one racking bay out (2.7 m) or a robot stopped short
# of its goal fails.
GOAL_TOLERANCE_M = 0.35
AMCL_ATE_RMSE_BOUND_M = 0.30
MISSION_TIMEOUT_S = 1200.0  # wall clock; the simulation runs below real time on CI


@pytest.mark.launch_test
def generate_test_description():
    RESULTS.mkdir(parents=True, exist_ok=True)
    for stale in RESULTS.glob("*"):
        stale.unlink()

    bringup = FindPackageShare("amr_bringup")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, "launch", "simulation.launch.py"])),
        launch_arguments={"headless": "true", "real_time_factor": SIM_RTF}.items(),
    )

    mission_runner = Node(
        package="amr_bringup",
        executable="mission_runner",
        name="mission_runner",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "waypoints_file": PathJoinSubstitution([bringup, "config", "integration_mission.yaml"]),
            "output": str(RESULTS / "mission.json"),
            # Simulated seconds. The leg is about 25 s of driving; the rest is
            # the recovery ladder and the wait for the pallet to go.
            "timeout_sec": 240.0,
        }],
    )
    fault_injector = Node(
        package="amr_bringup",
        executable="fault_injector",
        name="fault_injector",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "schedule_file": PathJoinSubstitution([bringup, "config", "integration_fault.yaml"]),
            "output": str(RESULTS / "faults.json"),
        }],
    )
    pose_logger = Node(
        package="amr_bringup",
        executable="pose_logger",
        name="pose_logger",
        output="screen",
        parameters=[{"use_sim_time": True, "output_dir": str(RESULTS)}],
    )

    return (
        LaunchDescription([
            SetEnvironmentVariable("GZ_PARTITION", GZ_PARTITION),
            simulation,
            # After localisation is up; mission_runner itself waits for Nav2's
            # action server, and the fault clock starts when the goal executes.
            TimerAction(period=8.0, actions=[pose_logger, fault_injector, mission_runner]),
            launch_testing.actions.ReadyToTest(),
        ]),
        {"mission_runner": mission_runner},
    )


class TestMissionRuns(unittest.TestCase):
    def test_mission_runner_finishes(self, proc_info, mission_runner):
        proc_info.assertWaitForShutdown(process=mission_runner, timeout=MISSION_TIMEOUT_S)


@launch_testing.post_shutdown_test()
class TestMissionOutcome(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(REPO_ROOT / "analysis"))
        cls.mission = json.loads((RESULTS / "mission.json").read_text(encoding="utf-8"))
        faults_path = RESULTS / "faults.json"
        cls.faults = json.loads(faults_path.read_text(encoding="utf-8"))["faults"] if faults_path.exists() else []

        from localisation_error import evaluate, load_pose_csv

        truth = load_pose_csv(str(RESULTS / "ground_truth.csv"))
        cls.final_truth = (float(truth.x[-1]), float(truth.y[-1]))
        cls.amcl = evaluate(load_pose_csv(str(RESULTS / "amcl_pose.csv")), truth).as_dict()
        cls.ekf = evaluate(load_pose_csv(str(RESULTS / "ekf_odom.csv")), truth, align=True).as_dict()
        cls.wheel = evaluate(load_pose_csv(str(RESULTS / "wheel_odom.csv")), truth, align=True).as_dict()

        summary = {
            "simulated": True,
            "test": "navigation_recovery",
            "mission": cls.mission,
            "faults": cls.faults,
            "final_ground_truth_xy": cls.final_truth,
            "goal_xy": GOAL,
            "amcl": cls.amcl,
            "ekf": cls.ekf,
            "wheel_odometry": cls.wheel,
        }
        with open(RESULTS.parent / "navigation_recovery.json", "w", encoding="utf-8", newline="\n") as handle:
            json.dump(summary, handle, indent=2)

    def test_mission_succeeded(self):
        self.assertEqual(self.mission["outcome"], "succeeded", self.mission)

    def test_fault_fired_and_cleared(self):
        kinds = [f["type"] for f in self.faults]
        self.assertIn("block_goal", kinds, self.faults)
        self.assertIn("restore", kinds, self.faults)
        for fault in self.faults:
            self.assertFalse(str(fault["detail"]).startswith("failed"), fault)

    def test_a_recovery_behaviour_ran(self):
        ladder = {"clear_local_costmap", "clear_global_costmap", "spin", "back_up", "wait"}
        ran = ladder.intersection(self.mission["recovery_counts"])
        self.assertTrue(ran, "no recovery from the ladder ran: %s" % self.mission["recovery_counts"])

    def test_robot_finished_on_the_goal(self):
        error = math.hypot(self.final_truth[0] - GOAL[0], self.final_truth[1] - GOAL[1])
        self.assertLess(error, GOAL_TOLERANCE_M, "ground truth ends %.2f m from the goal" % error)

    def test_amcl_tracked_ground_truth(self):
        self.assertLess(self.amcl["ate_rmse_m"], AMCL_ATE_RMSE_BOUND_M, self.amcl)
        self.assertEqual(self.amcl["n_jumps_over_0p5m"], 0, self.amcl)
