
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # <--- IMPORT AGGIUNTO
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    # UR type, scelta (default ur5e, quello in laboratorio)
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            default_value="ur5e"
        )
    )

    # sovrascrittura parametri dal file xacro, per esempio per la safety
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )

    # File e package per la descrizione del robot, in questo caso xacro
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="arm_description",
            description="Description package with robot URDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="arm.urdf.xacro",
            description="URDF/XACRO description file with the robot arms.",
        )
    )

    # File e package della camera
    declared_arguments.append(
        DeclareLaunchArgument(
            "real_camera_package",
            default_value="real_camera",
            description="config files for camera",
        )
    )


    # UR type, scelta (default ur5e, quello in laboratorio)
    ur_type = LaunchConfiguration("ur_type")

    # sovrascrittura parametri dal file xacro, per esempio per la safety
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    # File e package per la descrizione del robot, in questo caso xacro
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    camera_package = LaunchConfiguration("real_camera_package")


    # metto tutto in questa struttura avvolgendo Command con ParameterValue
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
                " ",
                "safety_limits:=",
                safety_limits,
                " ",
                "safety_pos_margin:=",
                safety_pos_margin,
                " ",
                "safety_k_position:=",
                safety_k_position,
                " ",
                "name:=",
                "ur",
                " ",
                "ur_type:=",
                ur_type,
                " ",
            ]
        ),
        value_type=str  # <--- FORZA LA RESA A STRINGA TESTUALE PER EVITARE L'ERRORE YAML
    )
    
    robot_description = {"robot_description": robot_description_content}


    # RViz config file
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(camera_package), "rviz", "test_camera.rviz"]
    )


    # nodi da eseguire
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # # faccio partire il nodo di vision da launch file separato
    # vision_node = Node(
    #     package="real_camera",
    #     executable="vision_node",
    #     output="screen"
    # )

  
  
    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        #vision_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)