from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('centauro_hmi_backend')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Arrancar RViz2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'backend.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'mock_visualization.launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
        ),
    ])
