"""Publish the robot description and TF for the fixed joints.

Split out from the simulation launch so that it can be run on its own against a
real robot, or against a bag, without pulling in Gazebo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gazebo = LaunchConfiguration("use_gazebo")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "urdf", "amr.urdf.xacro"]
    )

    # The space after 'xacro ' is required: Command joins these into one string.
    # value_type=str is not optional. Without it the launch system tries to infer
    # a parameter type from the expanded URDF text, and a description that
    # happens to start with a digit gets published as a number.
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_gazebo:=", use_gazebo]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "use_gazebo",
                default_value="true",
                description="Include the Gazebo plugin block. False for a real robot or a bag replay.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time, "robot_description": robot_description}
                ],
            ),
        ]
    )
