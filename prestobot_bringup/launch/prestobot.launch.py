from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Declare the launch arguments
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="Set to 'false' when launching on the real robot.",
    )

    auto_arg = DeclareLaunchArgument(
        "auto",
        default_value="true",
        description="Set to 'true' to launch the navigation stack.",
    )

    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/diff_drive_controller/cmd_vel",
        description="The output topic for the cmd_vel relay."
    )

    # Arguments for initial robot pose in simulation
    x_pose_arg = DeclareLaunchArgument(
        "x_pose",
        default_value="-33.0",
        description="Initial x position of the robot in the simulation."
    )
    y_pose_arg = DeclareLaunchArgument(
        "y_pose",
        default_value="-16.0",
        description="Initial y position of the robot in the simulation."
    )
    z_pose_arg = DeclareLaunchArgument(
        "z_pose",
        default_value="0.01",
        description="Initial z position of the robot in the simulation."
    )


    # 2. Include robot_description.launch.xml (unconditionally)
    robot_description_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prestobot_description"),
                    "launch",
                    "robot_description.launch.xml",
                ]
            )
        ),
        # Pass the 'is_sim' argument down to the included launch file
        launch_arguments={"is_sim": LaunchConfiguration("is_sim")}.items(),
    )

    # 3. Include ros2_control.launch.py and pass the is_sim argument to it
    ros2_control_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prestobot_controller"),
                    "launch",
                    "ros2_control.launch.py",
                ]
            )
        ),
        # Pass the 'is_sim' argument down to the included launch file
        launch_arguments={"is_sim": LaunchConfiguration("is_sim")}.items(),
    )

    # 4. Include gazebo.launch.xml ONLY if is_sim is true
    gazebo_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prestobot_simulation"),
                    "launch",
                    "gazebo.launch.xml",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("is_sim")),
        # Pass the pose arguments down to the gazebo launch file
        launch_arguments={
            "x_pose": LaunchConfiguration("x_pose"),
            "y_pose": LaunchConfiguration("y_pose"),
            "z_pose": LaunchConfiguration("z_pose"),
        }.items(),
    )

    # 5. Include navigation.launch.xml ONLY if 'auto' is true
    navigation_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prestobot_navigation"),
                    "launch",
                    "navigation.launch.xml",
                ]
            )
        ),
        # This launch file is only included if the 'auto' argument is 'true'
        condition=IfCondition(LaunchConfiguration("auto")),
        # Pass arguments down to the included launch file
        launch_arguments={
            "output_topic": LaunchConfiguration("output_topic"),
            "use_sim_time": LaunchConfiguration("is_sim") # Link use_sim_time to is_sim
        }.items(),
    )

    # 6. Include sllidar_c1_launch.py ONLY if is_sim is false
    sllidar_c1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sllidar_ros2"),
                    "launch",
                    "sllidar_c1_launch.py",
                ]
            )
        ),
        condition=UnlessCondition(LaunchConfiguration("is_sim")),
    )


    # 7. Return the LaunchDescription
    return LaunchDescription(
        [
            is_sim_arg,
            auto_arg,
            output_topic_arg,
            x_pose_arg,
            y_pose_arg,
            z_pose_arg,
            robot_description_include,
            ros2_control_include,
            gazebo_include,
            navigation_include,
            sllidar_c1_include,
        ]
    )