from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        robot_name="ur5e",  
        package_name="moveit_config"
    ).to_moveit_configs()

    clik_node = Node(
        package="shot_planning",
        executable="clik_node",
        output="screen",
        parameters=[
            # Configurazione di MoveIt
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,

            # Iperparametri algoritmici
            {
                "Tclik": 0.1,                       # Frequenza di calcolo del clik
                "gamma_on_T": 0.5,                    # guadagno del clik
                "default_max_iterations": 100,                # numero massimo di iterazioni
                "default_position_tolerance": 0.001,          # tolleranza di posizione (1mm)
                "default_orientation_tolerance": 0.01,        # tolleranza di orientamento (circa 1°)
                "singularity_trshld_warn": 0.1,      # soglia di warning -> vicini alla singolarità
                "singularity_trshld_error": 0.01,     # soglia di errore -> siamo in singolarità
                "lambda_max": 1.0,                    # lambda_max per DLS (Damped Least Squares)
                "save_on": True,                        # salva i dati su file
                "file_path": "/home/francesco/Desktop/ProgettoRobotica/ws_ur5e_ballpool/src/shot_planning/temp_data/clik_data.csv"  # percorso del file di output
            }
        ],
    )

    return LaunchDescription([
        clik_node
    ])