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

    motor_config = os.path.join(
        get_package_share_directory('koby_core'),
        'config',
        'motor_config.yaml'
    )

    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        # output="screen",
        namespace="",
        # arguments=["--dev", "/dev/input/js0"],  # replace with your joystick device path
    )

    joystick_control = Node(
        package="koby_core",
        executable="joystick_control.py",
        name="Joystick_Node",
        # output="screen",
        namespace="",
    )
    
    test_sub_control = Node(
        package="koby_core",
        executable="test_sub_cmd_vel.py",
        name="Test_sub_cmd_vel_Node",
        output="screen",
        namespace="",
        parameters=[motor_config],
    )
    
    ld.add_action(joy)
    ld.add_action(joystick_control)
    ld.add_action(test_sub_control)

    return ld