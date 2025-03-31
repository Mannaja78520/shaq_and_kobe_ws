import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Define the path to the launch file directory
    launch_file_dir = os.path.join(get_package_share_directory('shaq_core'), 'launch')
    
    
    # Include microros.launch.py
    microros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'microros.launch.py')
        )
    )
    
    cmd_vel_to_motor_speed = Node(
        package="shaq_core",
        executable="cmd_vel_to_motor_speed.py",
        name="Cmd_Vel_To_Rpm",
        # output="screen",
        namespace="",
        # parameters=[motor_config], #Testing
    )
    
    cmd_vel_to_motor_speed = Node(
        package="shaq_core",
        executable="cmd_vel_to_motor_speed.py",
        name="Cmd_Vel_To_Rpm",
        # output="screen",
        namespace="",
        # parameters=[motor_config], #Testing
    )
    
    cmd_vel_auto_aim = Node(
        package="shaq_core",
        executable="cmd_vel_auto_aim.py",
        name="Cmd_Vel_Auto_Aim",
        # output="screen",
        namespace="",
        # parameters=[], #Testing
    )


    # Add actions to the launch description
    # ld.add_action(microros_launch)
    # ld.add_action(cmd_vel_to_motor_speed)
    ld.add_action(cmd_vel_auto_aim)

    return ld

if __name__ == '__main__':
    generate_launch_description()
