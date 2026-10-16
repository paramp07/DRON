import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Core thermal mapping node
        Node(
            package='dron_mapping',
            executable='thermal_mapping_node',
            name='thermal_mapping_node',
            parameters=[{
                'depth_tolerance_m': 0.15,
                'session_id': 'session_flight_01',
                'drone_id': 'drone_01'
            }],
            output='screen'
        ),
        # Snapshot server for late-joining Unity clients
        Node(
            package='dron_mapping',
            executable='map_snapshot_server',
            name='map_snapshot_server',
            output='screen'
        )
    ])
