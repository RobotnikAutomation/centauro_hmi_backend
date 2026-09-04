import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('centauro_hmi_backend')
    return LaunchDescription([
        Node(
            package='centauro_hmi_backend',
            executable='hmi_backend',
            name='centauro_hmi_backend',
            output='screen',
            parameters=[os.path.join(share, 'config', 'backend.yaml')],
        )
    ])
