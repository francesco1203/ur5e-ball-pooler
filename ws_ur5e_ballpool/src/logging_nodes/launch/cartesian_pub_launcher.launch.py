# rispetto a loggers.launch.py, questo launch file serve a lanciare solo il nodo cartesian_pose_publisher senza lanciare il bag_writer

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    package_name = 'logging_nodes'

    # Carica in automatico tutte le descrizioni (URDF, SRDF, kinematics) dal pacchetto moveit
    moveit_config = MoveItConfigsBuilder("arm_ur5e", package_name="moveit_config").to_moveit_configs()

    cartesian_publisher_node = Node(
        package=package_name,
        executable='cartesian_pose_publisher',  
        name='cartesian_pose_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description, 
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics
        ] 
    )

    return LaunchDescription([
        cartesian_publisher_node
    ])