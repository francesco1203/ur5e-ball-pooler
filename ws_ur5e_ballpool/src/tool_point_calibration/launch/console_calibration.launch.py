from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tool_point_calibration',
            executable='console_calibration',
            name='console_calibration',
            output='screen',
            parameters=[{
                'base_frame': 'ur5e_base_link',
                'tool0_frame': 'ur5e_tool0',
                'num_samples': 4,
            }],
        )
    ])
