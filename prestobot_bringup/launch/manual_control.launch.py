from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the launch argument that this file will accept
    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/diff_drive_controller/cmd_vel",
        description="The final output topic for velocity commands."
    )

    # Define the teleop include
    teleop_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("teleop_twist_joy"),
                    "launch",
                    "teleop-launch.py",
                ]
            )
        ),
        launch_arguments={"joy_config": "xbox"}.items(),
    )

    # Define the twist_stamper node
    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel')
        ]
    )

    # # Define the custom topic_relay node
    # topic_relay_node = Node(
    #     package="prestobot_py_pkg",
    #     executable="topic_relay",
    #     output="screen",
    #     parameters=[
    #         {"input_topic": "/cmd_vel"},
    #         {"output_topic": LaunchConfiguration("output_topic")},
    #         {"message_type": "geometry_msgs/msg/TwistStamped"},
    #     ],
    # )

    # Return the LaunchDescription with all the teleop components
    return LaunchDescription([
        output_topic_arg,
        teleop_include,
        twist_stamper_node,
        # topic_relay_node,
    ])