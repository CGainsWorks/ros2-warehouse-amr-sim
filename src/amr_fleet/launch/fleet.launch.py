"""The VDA 5050 adapter, optionally with the whole simulated stack under it.

    ros2 launch amr_fleet fleet.launch.py broker_host:=localhost

`docker compose -f docker/docker-compose.yml up fleet` runs this against the
Mosquitto service in the same compose file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("amr_bringup"), "launch", "simulation.launch.py"])
        ),
        launch_arguments={"headless": LaunchConfiguration("headless"),
                          "real_time_factor": LaunchConfiguration("real_time_factor")}.items(),
        condition=IfCondition(LaunchConfiguration("use_simulation")),
    )
    adapter = Node(
        package="amr_fleet",
        executable="vda5050_adapter",
        name="vda5050_adapter",
        output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("amr_fleet"), "config", "vda5050.yaml"]),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "broker_host": LaunchConfiguration("broker_host"),
                "broker_port": ParameterValue(LaunchConfiguration("broker_port"), value_type=int),
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("broker_host", default_value="localhost"),
        DeclareLaunchArgument("broker_port", default_value="1883"),
        DeclareLaunchArgument("use_simulation", default_value="true",
                              description="Also bring up gz-sim and the stack."),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        simulation,
        adapter,
    ])
