import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro
from os.path import join

def generate_launch_description():

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rrbot = get_package_share_directory('rrbot_gazebo')
    world_file = LaunchConfiguration("world_file", default = join(pkg_ros_gz_rrbot, "worlds", "empty.world"))

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_ros_gz_rrbot, 'urdf', 'rrbot.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rrbot, 'config', 'ros_gz_bridge.yaml')
    
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Start Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Start Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])

        }.items()
    )

    # Start RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ros_gz_rrbot, 'rviz', 'model.rviz')],
    )

    # Spawn Robot in Gazebo   
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'name': 'rrbot',
                    'topic': 'robot_description'}],
        output='screen',
    )


      # Bridge ROS topics and Gazebo messages for establishing communication
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
          'config_file': ros_gz_bridge_config,
        }],
        output='screen'
      )  
    # Start bridge fo cameras    
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
          '/camera/depth_image',
          '/camera/image',
        ],
        remappings=[
          ('/camera/depth_image', '/camera/depth/image_rect_raw'),
          ('/camera/image', '/camera/color/image_raw'),
        ],
      )

    # Start arm controller
    start_arm_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'arm_controller'],
            output='screen')

    # Launch joint state broadcaster
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'],
            output='screen')

    # Launch the joint state broadcaster after spawning the robot
    load_joint_state_broadcaster_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=spawn ,
        on_exit=[start_joint_state_broadcaster_cmd],))

    # Launch the arm controller after launching the joint state broadcaster
    load_arm_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=start_joint_state_broadcaster_cmd,
        on_exit=[start_arm_controller_cmd],))
               

        # Launch the rqt_joint_trajectory_controller standalone
    rqt_traj_controller = ExecuteProcess(
            cmd=['rqt', '--standalone', 'rqt_joint_trajectory_controller'],
            output='screen',
        )

    # Delay the start by 7 seconds using TimerAction
    delayed_rqt_traj_controller = TimerAction(
        period=10.0,  # Delay in seconds
        actions=[rqt_traj_controller]
    )




    return LaunchDescription(
        [
            # Nodes and Launches
            gazebo,
            spawn,
            start_gazebo_ros_bridge_cmd,
            start_gazebo_ros_image_bridge_cmd,
            robot_state_publisher,
            load_arm_controller_cmd,
            load_joint_state_broadcaster_cmd,
            rviz,
            delayed_rqt_traj_controller,
        ]
    )
