from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from os import environ
from os.path import join


# Change this to your Groot2 executable path.
# This currently defaults to a Groot2 AppImage file in your home directory.
groot2_executable = join(environ.get("HOME", "/"), "Groot2/bin/groot2")


def get_autonomy_and_visualization_nodes(context, *args, **kwargs):

    xml_file_name = "nav_tree_naive.xml"
    print(f"\nUsing Behavior tree file: {xml_file_name}\n")

    pkg_tb_autonomy = get_package_share_directory("nav2_bt_client")
    xml_file_path = join(pkg_tb_autonomy, "bt_xml", xml_file_name)

    return [
        # Main autonomy node.
        Node(
            package="nav2_bt_client",
            executable="nav2_bt_node",
            name="nav2_bt_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "location_file": LaunchConfiguration("location_file"),
                    "tree_xml_file": xml_file_path,
                }
            ],
        ),
        # Behavior tree visualization with Groot2.
        ExecuteProcess(
            cmd=[groot2_executable, "--nosplash", "true", "--file", xml_file_path]
        ),
    ]


def generate_launch_description():
    pkg_tb_worlds = get_package_share_directory("nav2_bt_client")
    default_world_dir = join(pkg_tb_worlds, "config", "map_locations.yaml")

    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "location_file",
                default_value=TextSubstitution(text=default_world_dir),
                description="YAML file name containing map locations in the world.",
            ),
            # Autonomy node and behavior tree visualization nodes
            OpaqueFunction(function=get_autonomy_and_visualization_nodes),
        ]
    )
