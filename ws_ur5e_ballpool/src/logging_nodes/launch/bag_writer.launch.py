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
    
    # Se per "fuori dal workspace" intendi la cartella "ProgettoRobotica",
    # puoi risalire di un altro livello usando os.path.dirname():
    project_root = os.path.dirname(ws_root) 
    
    # Costruisci dinamicamente il percorso (in ProgettoRobotica/data/bagdata)
    # Se invece volevi salvare in ws_ur5e_ballpool/data/bagdata, usa 'ws_root' al posto di 'project_root'
    bag_dir = os.path.join(project_root, 'data', 'bagdata')

    # Crea la cartella bagdata se non esiste ancora
    os.makedirs(bag_dir, exist_ok=True)


     # Dichiara l'argomento da linea di comando (con un valore di default)
    test_title_arg = DeclareLaunchArgument(
        'test_title',
        default_value='prova_default',
        description='Titolo della prova (usato per nominare la sottocartella dei bag)'
    )

    # Crea l'oggetto che recupererà il valore a runtime
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

    # 4. Ricordati di aggiungere test_title_arg alla LaunchDescription!
    return LaunchDescription([
        test_title_arg,
        bag_writer_node
    ])