from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='ros2_basic_agent',
            executable='ros2_ai_agent_turtlesim',
            name='ros2_ai_agent_turtlesim',
            output='screen',
            emulate_tty=True,
        )
    ])
