from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    # 1. Declare the launch argument
    auto_arg = DeclareLaunchArgument(
        "auto",
        default_value="true",
        description="Set to 'true' to launch the HMI node for autonomous mode. If 'false', only RViz will be launched.",
    )
    # 2. Define the path to the rviz configuration file
    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("prestobot_navigation"),
            "rviz",
            "navigation_config.rviz",
        ]
    )

    # 3. Define the rviz2 node (launched unconditionally)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    # 4. Define the HMI node, to be launched only if 'auto' is true
    hmi_node = Node(
        package="prestobot_py_pkg",
        executable="hmi",
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto")),
    )

    # 5. Return the LaunchDescription
    return LaunchDescription(
        [
            auto_arg,
            rviz_node,
            hmi_node,
        ]
    )