
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
      moveit_config = MoveItConfigsBuilder(
          "ur5e", package_name="moveit_config"
      ).to_moveit_configs()

      moveit_config_launch_dir = os.path.join(
          get_package_share_directory("moveit_config"), "launch"
      )

      def include(launch_file_name):
          return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(moveit_config_launch_dir, launch_file_name)))

      static_virtual_joint_tfs = include("static_virtual_joint_tfs.launch.py")
      rsp = include("rsp.launch.py")
      move_group = include("move_group.launch.py")
      moveit_rviz = include("moveit_rviz.launch.py")

      controllers_file = os.path.join(
          get_package_share_directory("moveit_config"),
          "config",
          "ros2_controllers.yaml"
      )

      # Dashboard client per controllo automatico del robot
      dashboard_client = Node(
          package="ur_robot_driver",
          executable="dashboard_client",
          name="dashboard_client",
          output="screen",
          emulate_tty=True,
          parameters=[{"robot_ip": "192.168.56.101"}],
      )

      # Il controller manager che si interfaccia con il driver UR
      ros2_control_node = Node(
          package="controller_manager",
          executable="ros2_control_node",
          output="both",
          parameters=[
              moveit_config.robot_description,
              controllers_file,
          ],
      )

      # FORZIAMO L'AVVIO DEL BROADCASTER
      jsb_spawner = Node(
          package="controller_manager",
          executable="spawner",
          arguments=["joint_state_broadcaster",
  "--controller-manager", "/controller_manager"],
      )

      # FORZIAMO L'AVVIO DEL CONTROLLER DELL'ARM
      arm_spawner = Node(
          package="controller_manager",
          executable="spawner",
          arguments=["left_arm_controller",
  "--controller-manager", "/controller_manager"],
      )

      return LaunchDescription(
          [
              static_virtual_joint_tfs,
              rsp,
              dashboard_client,
              ros2_control_node,
              move_group,
              moveit_rviz,
              TimerAction(period=2.0, actions=[jsb_spawner,
  arm_spawner]),
          ]
      )
