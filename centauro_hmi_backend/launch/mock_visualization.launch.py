import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('centauro_hmi_backend')
    with open(os.path.join(share, 'urdf', 'arm.urdf'), encoding='utf-8') as f:
        robot_description = f.read()
    rviz_path = os.path.join(share, 'rviz', 'arm.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Arrancar RViz2'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', rviz_path],
        ),
    ])
