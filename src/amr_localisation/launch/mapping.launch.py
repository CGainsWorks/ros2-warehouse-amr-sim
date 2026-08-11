"""Cartographer mapping run.

Brings up Cartographer plus the occupancy grid publisher against an already
running simulation. Drive the robot with ``scripts/run_mapping.sh``, then save
the result with the ``/write_state`` service or the map_saver invocation printed
at the end of that script.

Deliberately does not launch AMCL or the EKF's map-frame consumers: during
mapping, Cartographer is the authority on ``map -> odom`` and running AMCL at
the same time gives two publishers on one transform.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_dir = PathJoinSubstitution([FindPackageShare("amr_localisation"), "config"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo clock. Always true for a simulated mapping run.",
            ),
            DeclareLaunchArgument(
                "resolution",
                default_value="0.05",
                description=(
                    "Occupancy grid resolution in metres. Must match the "
                    "global costmap resolution in amr_navigation, or the static "
                    "layer resamples the map every update."
                ),
            ),
            DeclareLaunchArgument(
                "publish_period_sec",
                default_value="1.0",
                description="How often the assembled occupancy grid is republished.",
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory",
                    config_dir,
                    "-configuration_basename",
                    "cartographer_warehouse.lua",
                ],
                remappings=[
                    ("scan", "/scan"),
                    ("odom", "/odometry/filtered"),
                    ("imu", "/imu/data"),
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-resolution",
                    LaunchConfiguration("resolution"),
                    "-publish_period_sec",
                    LaunchConfiguration("publish_period_sec"),
                ],
            ),
        ]
    )
