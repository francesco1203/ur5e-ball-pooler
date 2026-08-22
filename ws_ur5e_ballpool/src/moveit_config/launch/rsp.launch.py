# GENERATO DA MOVEITSETUPASSISTANT

# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_rsp_launch

# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("left_arm_ur5e", package_name="moveit_config").to_moveit_configs()
#     return generate_rsp_launch(moveit_config)



# PERSONALIZZATO PER MODIFICARE LA FREQUENZA DI PUBBLICAZIONE DELLO STATE PUBLISHER
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Costruiamo la configurazione di MoveIt come prima
    moveit_config = MoveItConfigsBuilder("left_arm_ur5e", package_name="moveit_config").to_moveit_configs()
    
    # Invece di usare il generatore automatico, dichiariamo il nodo esplicitamente
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": 100.0}      # <--- Aumentiamo la frequenza di pubblicazione a 100 Hz
        ],
    )
    
    return LaunchDescription([rsp_node])
