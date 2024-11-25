from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    node_microros_1 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        # output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB1"],
    )
    
    ld.add_action(node_microros_1)

    return ld