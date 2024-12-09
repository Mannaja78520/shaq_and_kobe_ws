import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # locations = os.path.join(
    #     get_package_share_directory('wrg_core'),
    #     'config',
    #     'locations.yaml'
    # )

    node_hello = Node(
        package="abu_core",
        executable="helloworld.py",
        name="hello_world_node",
        # output="screen",
        namespace="",
    )
        
    
    ld.add_action(node_hello)

    return ld