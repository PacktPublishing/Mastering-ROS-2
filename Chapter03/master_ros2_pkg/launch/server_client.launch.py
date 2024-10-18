import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First node
        Node(
            package='master_ros2_pkg',
            executable='simple_server',
            name='simple_server',
            output='screen',
        ),
        # Second node
        Node(
            package='master_ros2_pkg',
            executable='simple_client',
            name='simple_client',
            output='screen',
        ),
    ])


