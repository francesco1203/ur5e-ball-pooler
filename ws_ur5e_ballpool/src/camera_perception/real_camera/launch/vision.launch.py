from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Dichiara l'argomento per il launch file
    required_samples_arg = DeclareLaunchArgument(
        'required_samples',
        default_value='200',
        description='Numero di campioni richiesti per la calibrazione (iperparametro)'
    )

    # 2. Configura il nodo passandogli il parametro
    vision_node = Node(
        package="real_camera",
        executable="vision_node",
        output="screen",
        parameters=[{
            'required_samples': LaunchConfiguration('required_samples')
        }]
    )

    tf_freezer_node = Node(
            package="real_camera",
            executable="tf_freezer_node",
            output="screen",
            
        )

    return LaunchDescription([
        required_samples_arg,
        vision_node,
        tf_freezer_node
    ])