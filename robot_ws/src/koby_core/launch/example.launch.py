import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Define the path to the launch file directory
    launch_file_dir = os.path.join(get_package_share_directory('wrg_core'), 'launch')
    
    turtlebot3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'turtlebot3_bringup.launch.py')
        )
    )
    
    # Include microros.launch.py
    microros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'microros.launch.py')
        )
    )

    # Include navigate.launch.py
    navigate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'navigate.launch.py')
        )
    )

    # Include state.launch.py
    state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'state.launch.py')
        )
    )
    
    monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'monitor.launch.py')
        )
    )
    
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'rviz.launch.py')
        )
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # Add actions to the launch description
    # ld.add_action(rviz_node)
    ld.add_action(turtlebot3_launch)
    ld.add_action(microros_launch)
    ld.add_action(state_launch)
    ld.add_action(navigate_launch)
    ld.add_action(rosbridge_node)
    # ld.add_action(monitor_launch)

    return ld

if __name__ == '__main__':
    generate_launch_description()
