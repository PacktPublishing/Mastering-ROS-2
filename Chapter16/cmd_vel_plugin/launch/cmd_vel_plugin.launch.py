
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    xacro_path = 'urdf/diff_drive.urdf.xacro'

    robot_description = PathJoinSubstitution([
        get_package_share_directory('cmd_vel_plugin'),	
        xacro_path
    ])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',        
        parameters=[{
            'robot_description':Command(['xacro ', robot_description])
        }]
    )

    
    # Spawn
    spawn_node = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                    '-name', 'diff_drive',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.1',
                    '-r', '0',
                    '-p', '0',
                    '-Y', '0',
                    '-topic', '/robot_description'],
                 output='screen')

    
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 empty.sdf'
        }.items()
    )


    workspace_dir = os.getenv("ROS_WORKSPACE", default=os.getcwd())
    plugin_dir = os.path.join(workspace_dir, "build/cmd_vel_plugin")    

    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[plugin_dir]
    )
    
    '''
    workspace_dir = os.getenv("ROS_WORKSPACE", default=os.getcwd())
    plugin_dir = os.path.join(workspace_dir, "build/hello_world_ros")    
    
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[plugin_dir]
    )
    '''

    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/model/diff_drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )


    return LaunchDescription([      
        ign_resource_path,  
        robot_state_publisher_node,
        spawn_node,
        gazebo_node,    
        bridge
    ])