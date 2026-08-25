import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    
    default_config_path = os.path.join(
        get_package_share_directory('fake_camera'),
        'config',
        'fake_camera_config_center_sx.yaml'
    )


    yaml_path_arg = DeclareLaunchArgument(
        'yaml_path', # Nome dell'argomento da usare nel terminale
        default_value=default_config_path,
        description='Path to the YAML configuration file for fake_camera'
    )

    fake_camera_node = Node(
        package="fake_camera",
        executable="fake_camera_node",
        output="screen",
        parameters=[{
            # Passiamo il LaunchConfiguration invece della stringa fissa
            'yaml_file_path': LaunchConfiguration('yaml_path')
        }]
    )

    # 4. Aggiungi sia l'argomento che il nodo alla LaunchDescription
    return LaunchDescription([
        yaml_path_arg,
        fake_camera_node
    ])