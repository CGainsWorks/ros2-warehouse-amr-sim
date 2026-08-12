"""Nav2 servers for the warehouse AMR.

Brings up the planner, controller, smoother, behaviour, BT navigator, waypoint
follower and velocity smoother, plus a lifecycle manager to sequence them.

Localisation is NOT started here — it lives in amr_localisation. Keeping them
separate means the localisation ablation runs (EKF with and without scan
odometry) can restart one half of the stack without disturbing the other.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Order matters. The lifecycle manager configures and activates in this
# sequence, and the controller cannot configure until its costmap exists.
LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    behaviour_trees = PathJoinSubstitution(
        [FindPackageShare("amr_navigation"), "behavior_trees"]
    )

    common = [params_file, {"use_sim_time": use_sim_time}]

    # The behaviour tree paths must be resolved here, not in nav2_params.yaml.
    # That file is plain YAML with no substitution support, so a
    # $(find-pkg-share ...) written there reaches bt_navigator verbatim and it
    # fails to configure. Overriding after the params file means the value is a
    # real absolute path by the time the node sees it.
    bt_navigator_params = common + [
        {
            "default_nav_to_pose_bt_xml": PathJoinSubstitution(
                [behaviour_trees, "navigate_w_recovery.xml"]
            ),
            "default_nav_through_poses_bt_xml": PathJoinSubstitution(
                [behaviour_trees, "navigate_through_poses_w_recovery.xml"]
            ),
        }
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("amr_navigation"), "config", "nav2_params.yaml"]
                ),
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=common,
                # The velocity smoother sits between the controller and the base,
                # so the controller must not publish straight to /cmd_vel.
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=common,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=common,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=common,
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=bt_navigator_params,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=common,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=common,
                remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": LIFECYCLE_NODES,
                    }
                ],
            ),
        ]
    )
