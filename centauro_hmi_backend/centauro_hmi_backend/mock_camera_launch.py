import os

import yaml
from launch.actions import ExecuteProcess, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from .camera import FRONT_CAMERA_TOPIC


def mock_camera_actions(transport, rosbag_path, share):
    if transport not in ('websocket', 'webrtc'):
        raise ValueError('mock.camera_transport must be websocket or webrtc')
    bag_path = os.path.abspath(os.path.expanduser(rosbag_path))
    metadata_path = os.path.join(bag_path, 'metadata.yaml')
    with open(metadata_path, encoding='utf-8') as metadata_file:
        metadata = yaml.safe_load(metadata_file)
    topics = metadata['rosbag2_bagfile_information']['topics_with_message_count']
    if not any(
        entry['topic_metadata']['name'] == FRONT_CAMERA_TOPIC
        and entry['topic_metadata']['type'] == 'sensor_msgs/msg/Image'
        and entry['message_count'] > 0
        for entry in topics
    ):
        raise ValueError(f'Rosbag {bag_path} has no sensor_msgs/msg/Image frames on {FRONT_CAMERA_TOPIC}')

    player = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path, '--loop',
            '--disable-keyboard-controls', '--read-ahead-queue-size', '10',
            '--topics', FRONT_CAMERA_TOPIC,
        ],
        name='mock_camera_bag',
        output='screen',
    )
    processes = [player]
    if transport == 'webrtc':
        processes.append(Node(
            package='webrtc_video_stream',
            executable='webrtc_video_stream_node.py',
            name='webrtc_video_stream_node',
            output='screen',
            parameters=[{
                'config_file': os.path.join(share, 'config', 'mock_camera_webrtc.yaml'),
                'host': '0.0.0.0',
                'port': 8081,
                'enable_audio': False,
                'video_codec': 'VP8',
            }],
        ))
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[Shutdown(reason='Mock camera process exited; stopping backend launch')],
        ))
        for process in processes
    ]
    return handlers + processes
