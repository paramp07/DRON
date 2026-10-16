import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launches mapping in replay simulation mode with use_sim_time=True
        Node(
            package='dron_mapping',
            executable='thermal_mapping_node',
            name='thermal_mapping_replay',
            parameters=[{
                'use_sim_time': True,
                'session_id': 'replay_session',
                'drone_id': 'drone_01'
            }],
            output='screen'
        )
    ])
