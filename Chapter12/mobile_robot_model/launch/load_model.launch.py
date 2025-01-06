from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
import os

def generate_launch_description():

    
    ld = LaunchDescription()
    
    robot_model_dir = get_package_share_directory('mobile_robot_model')
    urdf_path = os.path.join(robot_model_dir, 'urdf', 'mobile_robot.urdf')
    urdf = open(urdf_path).read()           


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf}],
    )

    ld.add_action( robot_state_publisher_node )

    return ld