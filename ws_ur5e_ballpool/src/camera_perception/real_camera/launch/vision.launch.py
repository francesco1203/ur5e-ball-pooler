from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
   
    vision_node = Node(
        package="real_camera",
        executable="vision_node",
        output="screen",
    )

    return LaunchDescription([
        vision_node
    ])