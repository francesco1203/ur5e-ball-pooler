
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    package_name = 'logging_nodes'

    # Carica in automatico tutte le descrizioni (URDF, SRDF, kinematics) dal pacchetto moveit
    moveit_config = MoveItConfigsBuilder("ur5e", package_name="moveit_config").to_moveit_configs()

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