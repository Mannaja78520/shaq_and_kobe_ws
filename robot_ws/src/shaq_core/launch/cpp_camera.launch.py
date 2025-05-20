import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    apriltag_auto_aim = Node(
        package="shaq_core",
        executable="apriltag_auto_aim",
        name="Apriltag_Distance",
        # output="screen",
        namespace="",
        # parameters=[], #Testing
    )

    hoop_detection_onnx = Node(
        package="shaq_core",
        executable="hoop_detection_onnx",
        name="hoop_detection_onnx",
        output="screen",
        namespace="",
        # parameters=[], #Testing
    )
    
    hoop_detection_TFLite = Node(
        package="shaq_core",
        executable="hoop_detection_TFLite",
        name="hoop_detection_TFLite",
        # output="screen",
        namespace="",
        # parameters=[], #Testing
    )

    camera_driver = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [256, 256],
            'time_per_frame': [1, 20]  # 20 FPS
        }],
        remappings=[
            ('/image_raw', '/shaq/image_raw')
        ],
        # output='screen'
    )



    # Add actions to the launch description
    ld.add_action(camera_driver)
    # ld.add_action(apriltag_auto_aim)
    ld.add_action(hoop_detection_onnx)


    return ld

if __name__ == '__main__':
    generate_launch_description()
