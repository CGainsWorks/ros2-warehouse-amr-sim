"""EKF + scan odometry + map server + AMCL.

Start order matters and is enforced by the lifecycle manager rather than by
sleeps. The EKF and the scan matcher are ordinary nodes and come up
immediately; map_server and AMCL are lifecycle nodes and are configured and
activated in that order, because AMCL cannot build its likelihood field until
the map has been published.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    config_dir = PathJoinSubstitution([FindPackageShare("amr_localisation"), "config"])

    lifecycle_nodes = ["map_server", "amcl"]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("amr_bringup"), "maps", "warehouse.yaml"]
                ),
                description="Occupancy grid produced by the Cartographer mapping run.",
            ),
            DeclareLaunchArgument(
                "use_scan_odometry",
                default_value="true",
                description=(
                    "Run the LiDAR scan matcher and fuse it into the EKF. "
                    "Setting this false leaves the EKF on wheel odometry and IMU "
                    "alone, which is the ablation reported in "
                    "docs/localisation-accuracy.md."
                ),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([config_dir, "ekf.yaml"]),
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[("odometry/filtered", "/odometry/filtered")],
            ),
            Node(
                package="amr_localisation",
                executable="scan_odometry_node",
                name="scan_odometry_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_scan_odometry")),
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("~/scan", "/scan"),
                    ("~/odom", "/odom"),
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[
                    PathJoinSubstitution([config_dir, "amcl.yaml"]),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localisation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ]
    )
