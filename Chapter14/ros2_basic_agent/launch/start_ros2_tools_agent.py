from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_basic_agent',
            executable='ros2_ai_agent_basic_tools',  # Changed executable name
            name='ros2_ai_agent_basic_tools',        # Changed node name to match
            output='screen',
            emulate_tty=True,
        )
    ])
