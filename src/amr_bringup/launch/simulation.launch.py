"""Gazebo, the robot, and the full stack. This is what `make demo` runs.

Composes the other packages' launch files rather than redefining anything:
description -> Gazebo -> localisation -> navigation -> perception. Each stage is
individually launchable, which is what makes it possible to restart one of them
during debugging without tearing down the simulator.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    bringup_share = FindPackageShare("amr_bringup")
    world = PathJoinSubstitution([bringup_share, "worlds", "warehouse.world"])
    zones = PathJoinSubstitution([bringup_share, "zones", "warehouse_zones.yaml"])
    map_yaml = PathJoinSubstitution([bringup_share, "maps", "warehouse.yaml"])
    perception_params = PathJoinSubstitution(
        [FindPackageShare("amr_perception"), "config", "perception.yaml"]
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("amr_description"), "launch", "description.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            # With the relay enabled the LiDAR publishes to scan_raw and
            # amr_bringup's scan_relay forwards it to scan. Off by default, so
            # a normal run has nothing between the sensor and the stack.
            "scan_topic": PythonExpression(
                ["'scan_raw' if '", LaunchConfiguration("use_scan_relay"), "' == 'true' else 'scan'"]
            ),
        }.items(),
    )

    localisation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("amr_localisation"), "launch", "localisation.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_yaml,
            "use_scan_odometry": LaunchConfiguration("use_scan_odometry"),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("amr_navigation"), "launch", "navigation.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("headless", default_value="true",
                                  description="Run Gazebo without the GUI. True in Docker and CI."),
            DeclareLaunchArgument(
                "use_scan_odometry",
                default_value="false",
                description=(
                    "Fuse LiDAR scan odometry into the EKF. Default false on "
                    "measured evidence; see docs/localisation-accuracy.md."
                ),
            ),
            DeclareLaunchArgument(
                "use_navigation",
                default_value="true",
                description=(
                    "Start Nav2. Set false for the open-loop localisation harness, "
                    "which drives a fixed velocity profile so that localisation "
                    "quality can be measured without the recovery/slip feedback "
                    "loop that makes a mission run circular."
                ),
            ),
            DeclareLaunchArgument(
                "use_scan_relay",
                default_value="false",
                description=(
                    "Interpose amr_bringup's scan_relay between the LiDAR and the "
                    "stack so a sensor dropout can be induced. Only run_faults.sh "
                    "sets this; every other run is wired sensor-to-stack directly."
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("x", default_value="-12.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),

            Node(
                package="amr_bringup",
                executable="scan_relay",
                name="scan_relay",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_scan_relay")),
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "input_topic": "scan_raw",
                    "output_topic": "scan",
                }],
            ),
            ExecuteProcess(
                cmd=["gzserver", "--verbose", "-s", "libgazebo_ros_init.so",
                     "-s", "libgazebo_ros_factory.so", world],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["gzclient"],
                output="screen",
                condition=UnlessCondition(LaunchConfiguration("headless")),
            ),

            description,

            # Spawn after a short delay so robot_state_publisher has published
            # /robot_description. spawn_entity reads it as a parameter, and a
            # race here fails with a bare "entity pushed to spawn queue" and no
            # robot, which is a miserable thing to debug.
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        name="spawn_amr",
                        output="screen",
                        arguments=[
                            "-topic", "/robot_description",
                            "-entity", "warehouse_amr",
                            "-x", LaunchConfiguration("x"),
                            "-y", LaunchConfiguration("y"),
                            "-z", "0.05",
                            "-Y", LaunchConfiguration("yaw"),
                        ],
                    )
                ],
            ),

            # Localisation waits for the robot to exist, so that AMCL's first
            # scan callback has a TF tree to work with.
            TimerAction(period=6.0, actions=[localisation]),

            # Nav2 before perception, and the gap is deliberate.
            #
            # The two perception nodes each do roughly 35 ms of numpy per scan at
            # 10 Hz. Starting them alongside Nav2 puts that load on the machine
            # exactly while the lifecycle manager is building costmaps and
            # stepping seven servers through configure/activate. On a loaded host
            # that starves the manager: controller_server and smoother_server
            # come up, planner_server stops answering its lifecycle service
            # altogether, and the bringup stalls with no error logged anywhere.
            # planner_server configures in well under a second when it is not
            # competing for CPU.
            #
            # Nav2 tolerates its observation sources being absent for the first
            # few seconds — the costmap simply warns that a buffer has not been
            # updated — so deferring perception costs nothing and removes the
            # contention.
            TimerAction(
                period=10.0,
                actions=[
                    GroupAction(
                        actions=[navigation],
                        condition=IfCondition(LaunchConfiguration("use_navigation")),
                    )
                ],
            ),

            TimerAction(
                period=28.0,
                actions=[
                    Node(
                        package="amr_perception",
                        executable="segmentation_node",
                        name="segmentation_node",
                        output="screen",
                        parameters=[perception_params, {"use_sim_time": use_sim_time}],
                        remappings=[("~/input", "/lidar_3d/points")],
                    ),
                    Node(
                        package="amr_perception",
                        executable="zone_occupancy_node",
                        name="zone_occupancy_node",
                        output="screen",
                        parameters=[
                            perception_params,
                            {"use_sim_time": use_sim_time, "zone_config": zones},
                        ],
                        remappings=[("~/obstacles", "/segmentation_node/obstacles")],
                    ),
                ],
            ),

            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                arguments=[
                    "-d",
                    PathJoinSubstitution([FindPackageShare("amr_description"), "rviz", "amr.rviz"]),
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
