import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First node
        Node(
            package='master_ros2_pkg',
            executable='action_server',
            name='action_server',
            output='screen',
        ),
        # Second node
        Node(
            package='master_ros2_pkg',
            executable='action_client',
            name='action_client',
            output='screen',
        ),
    ])


