import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Define the path to the launch file directory
    launch_file_dir = os.path.join(get_package_share_directory('abu_core'), 'launch')
    
    
    # Include microros.launch.py
    microros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'microros.launch.py')
        )
    )


    # Add actions to the launch description
    ld.add_action(microros_launch)

    return ld

if __name__ == '__main__':
    generate_launch_description()
