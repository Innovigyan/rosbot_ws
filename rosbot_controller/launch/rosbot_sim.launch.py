import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    rosbot_description_dir = get_package_share_directory("rosbot_description")
    rosbot_controller_dir = get_package_share_directory("rosbot_controller")

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.023",
        description="Wheel radius of the robot"
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17",
        description="Wheel separation of the robot"
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            rosbot_description_dir, "urdf", "rosbot.urdf.xacro"
        ),
        description="Absolute path to robot urdf file"
    )
    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=os.path.join(
            rosbot_controller_dir, "config", "rosbot_controllers.yaml"
        ),
        description="Path to ros2_control controller configuration YAML file"
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    model = LaunchConfiguration("model")
    controller_config = LaunchConfiguration("controller_config")

    # Include gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rosbot_description_dir,
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={"model": model}.items()
    )

    # Robot description parameter for robot_state_publisher
    robot_description = Command([
        "xacro ", model
    ])

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time
        }]
    )

    # joint_state_publisher node (needed to publish static/fixed joint transforms)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # rviz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(rosbot_description_dir, "rviz", "display.rviz")],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # EKF Node
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[
            os.path.join(rosbot_controller_dir, "config", "ekf.yaml"),
            {"use_sim_time": use_sim_time}
        ]
    )

    # ros2_control nodes
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controller_config,
        ],
    )

    diff_drive_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controller_config,
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        model_arg,
        ekf_node,
        controller_config_arg,
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        diff_drive_base_controller_spawner
    ])