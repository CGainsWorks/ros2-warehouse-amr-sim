"""Run the demo mission with logging, and optionally with induced faults.

This is what `make eval` and `make faults` run. It assumes the simulation and
the stack are already up (``simulation.launch.py``), and adds only the three
nodes that observe and drive the run.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    results_dir = LaunchConfiguration("results_dir")
    bringup_share = FindPackageShare("amr_bringup")

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
            TimerAction(
                period=5.0,
                actions=[
                    Node(
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
                    ),
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
