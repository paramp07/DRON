import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dron_capture',
            executable='stereo_driver',
            name='stereo_camera_driver',
            parameters=[{
                'frame_rate': 15.0,
                'use_hardware': True,
                'left_frame_id': 'left_optical_frame',
                'right_frame_id': 'right_optical_frame'
            }],
            output='screen'
        ),
        Node(
            package='dron_capture',
            executable='thermal_driver',
            name='thermal_camera_driver',
            parameters=[{
                'device_port': '/dev/ttyACM0',
                'frame_rate': 9.0,
                'use_hardware': True,
                'frame_id': 'thermal_optical_frame'
            }],
            output='screen'
        )
    ])
