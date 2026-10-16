import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launches stereo_image_proc or RTAB-Map stereo odometry
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'approx_sync': True,
                'queue_size': 10
            }],
            remappings=[
                ('left/image_rect', '/drone_01/stereo/left/image_rect'),
                ('right/image_rect', '/drone_01/stereo/right/image_rect'),
                ('left/camera_info', '/drone_01/stereo/left/camera_info'),
                ('right/camera_info', '/drone_01/stereo/right/camera_info'),
            ],
            output='screen'
        )
    ])
