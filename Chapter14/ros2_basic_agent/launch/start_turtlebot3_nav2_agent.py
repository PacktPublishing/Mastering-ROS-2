from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include the Nav2 simulation launch file
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/tb3_simulation_launch.py']),
        launch_arguments={
            'headless': 'False',
            'slam': 'True'
        }.items()
    )

    # Define the AI agent node with a delay
    delayed_agent = TimerAction(
        period=10.0,  # 10 second delay
        actions=[
            Node(
                package='ros2_basic_agent',
                executable='ros2_ai_agent_nav2',
                name='ros2_ai_agent_turtlebot3',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    return LaunchDescription([
        nav2_launch,
        delayed_agent
    ])
