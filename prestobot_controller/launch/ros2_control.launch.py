from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the 'is_sim' launch argument
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="Set to true to skip launching the ros2_control_node for simulation.",
    )

    # Find the package share directory
    prestobot_controller_pkg = FindPackageShare("prestobot_controller")

    # Define the ros2_control_node, which runs only if 'is_sim' is false
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution(
                [
                    prestobot_controller_pkg,
                    "config",
                    "prestobot_controller.yaml",
                ]
            )
        ],
        condition=UnlessCondition(LaunchConfiguration("is_sim")),
    )

    # Define the controller manager spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    return LaunchDescription(
        [
            is_sim_arg,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]
    )