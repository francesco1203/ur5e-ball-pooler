import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Sfruttiamo il builder SOLO per ottenere la robot_description senza dover richiamare xacro a mano
    moveit_config = MoveItConfigsBuilder(
        "ur5e", package_name="moveit_config"
    ).to_moveit_configs()

    moveit_config_launch_dir = os.path.join(
        get_package_share_directory("moveit_config"), "launch"
    )

    def include(launch_file_name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_launch_dir, launch_file_name)
            )
        )

    # 2. Manteniamo solo il Robot State Publisher per pubblicare l'albero delle TF (inclusa la camera)
    rsp = include("rsp.launch.py")

    controllers_file = os.path.join(
        get_package_share_directory("moveit_config"),
        "config",
        "ros2_controllers.yaml"
    )

    # 3. Nodo principale di MuJoCo
    mujoco_ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        name="controller_manager", # ATTENZIONE: Questo forza il nome del nodo a coincidere con quello nel tuo YAML
        output="both",
        parameters=[
            moveit_config.robot_description,
            controllers_file,
            # use_sim_time rimosso da qui perché gestito globalmente dal SetParameter in basso
        ],
        remappings=[
            ('/realsense_plugin/realsense_d435/color', 'camera/camera/color/image_raw'),
            ('/realsense_plugin/realsense_d435/depth','camera/camera/aligned_depth_to_color/image_raw'),
            ('/realsense_plugin/realsense_d435/camera_info', 'camera/camera/color/camera_info')
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            # Forza il clock di simulazione per tutti i nodi
            SetParameter(name="use_sim_time", value=True),
            rsp,
            mujoco_ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ]
    )