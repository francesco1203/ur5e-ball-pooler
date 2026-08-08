import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "left_arm_ur5e", package_name="moveit_config"
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

    static_virtual_joint_tfs = include("static_virtual_joint_tfs.launch.py")
    rsp = include("rsp.launch.py")
    move_group = include("move_group.launch.py")
    # moveit_rviz = include("moveit_rviz.launch.py")
    spawn_controllers = include("spawn_controllers.launch.py")

    controllers_file = os.path.join(
            get_package_share_directory("moveit_config"),
            "config",
            "ros2_controllers.yaml" # <-- verifica il nome esatto del tuo file nella cartella config
        )

    # Al posto del ros2_control_node standard (controller_manager),
    # usiamo quello fornito da mujoco_ros2_control, che fa girare
    # la fisica di MuJoCo come backend della simulazione.
    mujoco_ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            moveit_config.robot_description,
            controllers_file,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [
            # Applica use_sim_time=True a TUTTI i nodi lanciati qui sotto,
            # inclusi quelli avviati tramite gli IncludeLaunchDescription
            # (move_group, rsp, rviz, spawner). Necessario perche' MuJoCo
            # pubblica un clock di simulazione e i nodi devono sincronizzarsi
            # su quello invece che sull'orologio di sistema.
            SetParameter(name="use_sim_time", value=True),
            static_virtual_joint_tfs,
            rsp,
            mujoco_ros2_control_node,
            move_group,
            # moveit_rviz,
            spawn_controllers,
        ]
    )