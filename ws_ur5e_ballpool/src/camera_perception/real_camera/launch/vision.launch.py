from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Definisci le variabili dinamiche (con i topic originali come default)
    color_topic = LaunchConfiguration('color_topic', default='/camera/color/image_raw')
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/depth/image_raw')
    info_topic = LaunchConfiguration('info_topic', default='/camera/color/camera_info')

    vision_node = Node(
        package="real_camera",
        executable="vision_node",
        output="screen",
        remappings=[
            ('/camera/color/image_raw', color_topic),
            ('/camera/depth/image_raw', depth_topic),
            ('/camera/color/camera_info', info_topic)
        ]
    )

    return LaunchDescription([
        # Dichiara gli argomenti per renderli visibili da terminale
        DeclareLaunchArgument('color_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('info_topic', default_value='/camera/color/camera_info'),
        vision_node
    ])