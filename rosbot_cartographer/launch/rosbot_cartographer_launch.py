from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', '/home/thebrobot/rosbot_ws/src/rosbot_cartographer/config',
                '-configuration_basename', 'rosbot_cartographer_config.lua'
            ],
            remappings=[
                ('/odom', '/odometry/filtered'),  
                ('/scan', '/scan')  
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            parameters=[{'resolution': 0.05, 'use_sim_time': True}]
        )
    ])
