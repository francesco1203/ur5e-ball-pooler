from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'execution_monitoring'

    # Configurazione del nodo logger cartesiano
    cartesian_logger_node = Node(
        package=package_name,
        executable='cartesian_logger',  
        name='cartesian_logger',
        output='screen',
        parameters=[
            {'logging_period': 0.01}
        ]
    )

    # Configurazione del nodo logger dei giunti
    joint_logger_node = Node(
        package=package_name,
        executable='joint_logger',      
        name='joint_logger',
        output='screen',
        parameters=[
            {'logging_period': 0.01}
        ]
    )

    # Lancio i nodi
    return LaunchDescription([
        cartesian_logger_node,
        joint_logger_node
    ])