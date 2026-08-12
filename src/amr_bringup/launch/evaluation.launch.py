"""Run the demo mission with logging, and optionally with induced faults.

This is what `make eval` and `make faults` run. It assumes the simulation and
the stack are already up (``simulation.launch.py``), and adds only the three
nodes that observe and drive the run.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    results_dir = LaunchConfiguration("results_dir")
    bringup_share = FindPackageShare("amr_bringup")

    mission_runner = Node(
        package="amr_bringup",
        executable="mission_runner",
        name="mission_runner",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "waypoints_file": LaunchConfiguration("waypoints_file"),
                "output": [results_dir, "/mission.json"],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("results_dir", default_value="results"),
            DeclareLaunchArgument(
                "inject_faults",
                default_value="false",
                description="Also run the fault injector. `make faults` sets this.",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "mission_waypoints.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "fault_schedule",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "fault_schedule.yaml"]
                ),
            ),

            # The logger starts first and unconditionally. A run whose mission
            # fails is still a run worth having the pose logs for.
            Node(
                package="amr_bringup",
                executable="pose_logger",
                name="pose_logger",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "output_dir": results_dir}],
            ),

            # Give Nav2 time to finish activating before offering it a mission.
            # When the mission ends — succeeded, aborted or rejected — tear the
            # whole launch down. Without this, pose_logger keeps spinning
            # forever and the run only ends when the calling script's timeout
            # fires, which wasted eight minutes per run before it was noticed.
            # Shutdown also gives pose_logger a clean SIGINT so it flushes and
            # closes its CSVs rather than leaving a truncated final line.
            RegisterEventHandler(
                OnProcessExit(
                    target_action=mission_runner,
                    on_exit=[
                        LogInfo(msg="mission runner exited; shutting down the evaluation"),
                        Shutdown(reason="mission complete"),
                    ],
                )
            ),

            TimerAction(
                period=5.0,
                actions=[
                    mission_runner,
                    Node(
                        package="amr_bringup",
                        executable="fault_injector",
                        name="fault_injector",
                        output="screen",
                        condition=IfCondition(LaunchConfiguration("inject_faults")),
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "schedule_file": LaunchConfiguration("fault_schedule"),
                                "output": [results_dir, "/faults.json"],
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
