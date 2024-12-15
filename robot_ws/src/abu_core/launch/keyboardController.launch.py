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

    keyboard_control = Node(
        package="abu_core",
        executable="keyboard_control.py",
        name="KeyboardControl_Node",
        output="screen",
        namespace="",
    )
    
    test_sub_control = Node(
        package="abu_core",
        executable="test_sub_cmd_vel.py",
        name="Test_sub_cmd_vel_Node",
        # output="screen",
        namespace="",
    )
    
    ld.add_action(keyboard_control)
    ld.add_action(test_sub_control)

    return ld