from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera',
            output='screen',
            parameters=[
                {'video_device': '/dev/video2'},
                {'image_width': 1280},
                {'image_height': 720},
                {'pixel_format': 'yuyv'},   
                {'camera_frame_id': 'usb_camera'}
            ],
            remappings=[
                ('/image_raw', '/image_rect')
            ]
        )
    ])
