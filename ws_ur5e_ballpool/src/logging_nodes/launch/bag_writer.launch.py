import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    package_name = 'logging_nodes'

    # Trova il path di installazione (es. /home/.../ws_ur5e_ballpool/install/logging_nodes)
    install_prefix = get_package_prefix(package_name)
    
    # Prendi la parte PRIMA di '/install' (indice 0) per avere la root del workspace
    ws_root = install_prefix.split('/install')[0]
    
    # Risali di un altro livello
    project_root = os.path.dirname(ws_root) 
    
    # Costruisci dinamicamente il percorso
    bag_dir = os.path.join(project_root, 'data', 'bagdata')

    # Crea la cartella bagdata se non esiste ancora
    os.makedirs(bag_dir, exist_ok=True)

    # Dichiara l'argomento per il titolo della prova
    test_title_arg = DeclareLaunchArgument(
        'test_title',
        default_value='prova_default',
        description='Titolo della prova (usato per nominare la sottocartella dei bag)'
    )


    # Crea gli oggetti che recupereranno i valori a runtime
    test_title_config = LaunchConfiguration('test_title')

    bag_writer_node = Node(
        package=package_name,
        executable='bag_writer',
        name='bag_writer',
        parameters=[{
            'bag_base_path': bag_dir,
            'test_title': test_title_config,
            'bag_format': 'mcap'
        }]
    )

    # Ricordati di aggiungere il nuovo argomento alla LaunchDescription
    return LaunchDescription([
        test_title_arg,
        bag_writer_node
    ])