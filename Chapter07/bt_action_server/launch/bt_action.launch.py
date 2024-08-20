from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    tree_path = PathJoinSubstitution ( [get_package_share_directory('bt_action_server'), 'trees', 'bt_action.xml' ])

    ld = LaunchDescription()
    
    bt_node = Node(
            package='bt_action_server',
            executable='bt_action',
            name='bt_action',
            output='screen',
            parameters=[{'tree_xml_file': tree_path}],
        )
    
    ld.add_action( bt_node )

    return ld

