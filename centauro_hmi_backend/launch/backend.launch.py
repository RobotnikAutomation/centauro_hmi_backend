import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('centauro_hmi_backend')
    config_path = os.path.join(share, 'config', 'backend.yaml')
    with open(config_path, encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    robot_parameters = config['centauro_hmi_backend']['ros__parameters']
    backend_node = Node(
        package='centauro_hmi_backend',
        executable='hmi_backend',
        name='centauro_hmi_backend',
        output='screen',
        parameters=[config_path],
    )

    ik_params_path = os.path.join(
        get_package_share_directory('cartesian_kinematics_for_hmi_teleoperation'),
        'config', 'ik_action_server.yaml')
    ik_node = Node(
        package='cartesian_kinematics_for_hmi_teleoperation',
        executable='ik_action_server',
        name='cartesian_ik_action_server',
        namespace=robot_parameters.get('ik.namespace', 'robot'),
        output='screen',
        parameters=[ik_params_path, {
            'target_frame': robot_parameters.get('ik.target_frame', 'hmi_tcp_target'),
            'tcp_frame': robot_parameters.get('ik.tcp_frame', 'robot_arm_tool0'),
            'joint_states_topic': robot_parameters.get(
                'robot.joint_states_topic', '/robot/joint_states'),
        }],
    )

    if robot_parameters.get('robot.type', 'mock') != 'real':
        return LaunchDescription([ik_node, backend_node])

    waiter_node = Node(
        package='centauro_hmi_backend',
        executable='wait_for_teleoperation',
        name='wait_for_teleoperation_node',
        output='screen',
        parameters=[{
            'robot.teleoperation_node_name': robot_parameters.get(
                'robot.teleoperation_node_name', '/robot/arm_teleoperation_node'
            ),
        }],
    )

    def start_backend_after_waiter(event, context):
        if event.returncode == 0:
            return [backend_node]
        return [
            Shutdown(reason='Real robot backend was not started because teleoperation is unavailable'),
        ]

    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(
            target_action=waiter_node,
            on_exit=start_backend_after_waiter,
        )),
        waiter_node,
        ik_node,
    ])
