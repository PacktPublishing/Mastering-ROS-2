import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First node
        Node(
            package='master_ros2_pkg',
            executable='publisher_node',
            name='publisher_node',
            output='screen',
        ),
        # Second node
        Node(
            package='master_ros2_pkg',
            executable='subscriber_node',
            name='subscriber_node',
            output='screen',
        ),
    ])


