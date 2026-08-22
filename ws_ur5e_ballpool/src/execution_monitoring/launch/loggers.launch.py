from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    package_name = 'execution_monitoring'

    # Carica in automatico tutte le descrizioni (URDF, SRDF, kinematics) dal pacchetto moveit
    moveit_config = MoveItConfigsBuilder("left_arm_ur5e", package_name="moveit_config").to_moveit_configs()

    cartesian_logger_node = Node(
        package=package_name,
        executable='cartesian_logger',  
        name='cartesian_logger',
        output='screen',
        # Passiamo il dizionario generato dal builder
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic] 
    )

    # Configurazione del nodo logger dei giunti
    joint_logger_node = Node(
        package=package_name,
        executable='joint_logger',      
        name='joint_logger',
        output='screen'
    )

    # Configurazione del nodo logger della coppia
    torque_logger_node = Node(
        package=package_name,
        executable='torque_logger',
        name='torque_logger',
        output='screen'
    )

    # Configurazione del nodo logger del controller
    controller_logger_node = Node(
        package=package_name,
        executable='controller_logger',
        name='controller_logger',
        output='screen'
    )

    # Lancio i nodi
    return LaunchDescription([
        cartesian_logger_node,
        joint_logger_node,
        torque_logger_node,
        controller_logger_node
    ])