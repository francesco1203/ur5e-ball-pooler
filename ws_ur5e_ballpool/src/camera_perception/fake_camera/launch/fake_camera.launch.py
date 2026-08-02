import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Trova il percorso assoluto installato del file YAML
    config_path = os.path.join(
        get_package_share_directory('fake_camera'),
        'config',
        'fake_camera_config.yaml'
    )

    fake_camera_node = Node(
        package="fake_camera",
        executable="fake_camera_node", # Nome dell'eseguibile
        output="screen",
        parameters=[{
            # 'position_update_timer': -1.0,
            'yaml_file_path': config_path
        }]
    )

    return LaunchDescription([
        fake_camera_node
    ])