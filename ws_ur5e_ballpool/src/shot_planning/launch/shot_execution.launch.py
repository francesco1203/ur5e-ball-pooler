from launch import LaunchDescription
from launch_ros.actions import Node
# from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():


    # moveit_config = MoveItConfigsBuilder(
    #     robot_name="ur5e",  
    #     package_name="moveit_config"
    # ).to_moveit_configs()

    task_node = Node(
        package="shot_planning",
        executable="task_node",
        output="screen",
        parameters=[
            # Configurazione di MoveIt
            # moveit_config.robot_description,
            # moveit_config.robot_description_semantic,
            # moveit_config.robot_description_kinematics,

            # Iperparametri algoritmici
            {
                "def_cartesian_planning_time": 10.0,      #10s
                "joint_planning_algorithm": "RRTConnectkConfigDefault",
                "cartesian_planning_algorithm": "RRTstarkConfigDefault",


                # SCELTE TRA ALGORITMI DI PLANNING (da ompl_planning.yaml):
                # - APSConfigDefault
                # - SBLkConfigDefault
                # - ESTkConfigDefault
                # - LBKPIECEkConfigDefault
                # - BKPIECEkConfigDefault
                # - KPIECEkConfigDefault
                # - RRTkConfigDefault
                # - RRTConnectkConfigDefault
                # - RRTstarkConfigDefault
                # - TRRTkConfigDefault
                # - PRMkConfigDefault
                # - PRMstarkConfigDefault
                # - FMTkConfigDefault
                # - BFMTkConfigDefault
                # - PDSTkConfigDefault
                # - STRIDEkConfigDefault
                # - BiTRRTkConfigDefault
                # - LBTRRTkConfigDefault
                # - BiESTkConfigDefault
                # - ProjESTkConfigDefault
                # - LazyPRMkConfigDefault
                # - LazyPRMstarkConfigDefault
                # - SPARSkConfigDefault
                # - SPARStwokConfigDefault
                # - TrajOptDefault
            }
        ],
    )

    return LaunchDescription([
        task_node
    ])