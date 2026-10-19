import asyncio
import base64
import importlib.util
import json
from pathlib import Path
import threading
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest
import yaml
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image

from centauro_hmi_backend.camera import FRONT_CAMERA_TOPIC, ImageStreamBridge
from centauro_hmi_backend.mock_camera_launch import mock_camera_actions
from centauro_hmi_backend.node import HmiBackend
from centauro_hmi_backend.robots.mock_robot import MockRobot


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def make_bridge():
    node = Mock()
    return ImageStreamBridge(node, [f'front_camera={FRONT_CAMERA_TOPIC}']), node


def image(stamp=10, pixels=b'\x00\x01\xff'):
    msg = Image()
    msg.header.stamp.sec = stamp
    msg.width = 1
    msg.height = 1
    msg.encoding = 'rgb8'
    msg.step = 3
    msg.data = pixels
    return msg


def test_image_subscription_accepts_reliable_and_best_effort_publishers():
    bridge, node = make_bridge()
    subscription = node.create_subscription.call_args.args
    assert subscription[0] is Image
    assert subscription[1] == FRONT_CAMERA_TOPIC
    assert subscription[3].reliability == ReliabilityPolicy.BEST_EFFORT
    assert bridge.snapshot() == []


def test_camera_payload_and_loop_timestamp_reset():
    bridge, node = make_bridge()
    callback = node.create_subscription.call_args.args[2]
    callback(image(stamp=20))
    first = bridge.snapshot()[0]
    assert first == {
        'stream_id': 'front_camera',
        'topic': FRONT_CAMERA_TOPIC,
        'encoding': 'base64',
        'format': 'ros_image',
        'data': 'AAH/',
        'image_encoding': 'rgb8',
        'width': 1,
        'height': 1,
        'step': 3,
        'is_bigendian': False,
        'timestamp': {'sec': 20, 'nanosec': 0},
        'sequence': 1,
    }
    assert base64.b64decode(first['data']) == bytes(image().data)
    assert bridge.snapshot() == []
    callback(image(stamp=10))
    looped = bridge.snapshot()[0]
    assert looped['timestamp']['sec'] == 10
    assert looped['sequence'] == 2


def test_camera_only_keeps_latest_frame():
    bridge, node = make_bridge()
    callback = node.create_subscription.call_args.args[2]
    for stamp in range(100):
        callback(image(stamp=stamp))
    frames = bridge.snapshot()
    assert len(frames) == 1
    assert frames[0]['sequence'] == 100
    assert frames[0]['timestamp']['sec'] == 99
    assert bridge.snapshot() == []


@pytest.fixture
def rosbag(tmp_path):
    metadata = {
        'rosbag2_bagfile_information': {
            'topics_with_message_count': [{
                'topic_metadata': {
                    'name': FRONT_CAMERA_TOPIC,
                    'type': 'sensor_msgs/msg/Image',
                },
                'message_count': 594,
            }],
        },
    }
    (tmp_path / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
    return tmp_path


@pytest.mark.parametrize('transport', ['websocket', 'webrtc'])
def test_mock_launch_loops_only_camera_topic_and_selects_webrtc(rosbag, transport):
    actions = mock_camera_actions(transport, str(rosbag), str(PACKAGE_ROOT))
    processes = [action for action in actions if isinstance(action, ExecuteProcess)]
    player = processes[0]
    cmd = [
        ''.join(part.perform(LaunchContext()) for part in argument)
        for argument in player.cmd
    ]
    assert cmd == [
        'ros2', 'bag', 'play', str(rosbag), '--loop',
        '--disable-keyboard-controls', '--read-ahead-queue-size', '10',
        '--topics', FRONT_CAMERA_TOPIC,
    ]
    webrtc_nodes = [p for p in processes if isinstance(p, Node)]
    assert len(webrtc_nodes) == (1 if transport == 'webrtc' else 0)
    if webrtc_nodes:
        assert webrtc_nodes[0].node_package == 'webrtc_video_stream'
    assert len(actions) == len(processes) * 2


def test_mock_launch_rejects_unknown_transport(rosbag):
    with pytest.raises(ValueError, match='websocket or webrtc'):
        mock_camera_actions('invalid', str(rosbag), str(PACKAGE_ROOT))


def test_mock_launch_rejects_missing_bag(tmp_path):
    with pytest.raises(FileNotFoundError):
        mock_camera_actions('websocket', str(tmp_path), str(PACKAGE_ROOT))


@pytest.mark.parametrize('change', ['topic', 'type', 'empty'])
def test_mock_launch_rejects_bag_without_camera_images(rosbag, change):
    metadata_path = rosbag / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text())
    topic = metadata['rosbag2_bagfile_information']['topics_with_message_count'][0]
    if change == 'topic':
        topic['topic_metadata']['name'] = '/other'
    elif change == 'type':
        topic['topic_metadata']['type'] = 'std_msgs/msg/String'
    else:
        topic['message_count'] = 0
    metadata_path.write_text(yaml.safe_dump(metadata))
    with pytest.raises(ValueError, match='has no sensor_msgs/msg/Image frames'):
        mock_camera_actions('websocket', str(rosbag), str(PACKAGE_ROOT))


def test_webrtc_source_matches_mock_websocket_topic():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'mock_camera_webrtc.yaml').read_text()
    )
    assert config['webrtc_video_stream_node']['ros__parameters']['sources'] == [{
        'name': 'front_camera', 'type': 'topic',
        'src': FRONT_CAMERA_TOPIC, 'codec': 'VP8',
    }]


@pytest.mark.parametrize('transport', ['websocket', 'webrtc'])
@pytest.mark.parametrize('frames', [[], [{'format': 'ros_image', 'data': 'AAH/'}]])
def test_publisher_uses_ros_images_only_in_websocket_mode(monkeypatch, transport, frames):
    from centauro_hmi_backend import node as backend_module

    backend = HmiBackend.__new__(HmiBackend)
    backend.robot = MockRobot()
    backend.lock = threading.Lock()
    backend.real_status = None
    backend.mock_camera = Mock() if transport == 'websocket' else None
    if backend.mock_camera:
        backend.mock_camera.snapshot.return_value = frames
    backend.transport = SimpleNamespace(publish=AsyncMock())
    backend.get_parameter = lambda _: SimpleNamespace(value=20.0)
    iterations = iter([True, False])
    monkeypatch.setattr(backend_module.rclpy, 'ok', lambda: next(iterations))
    asyncio.run(backend.publisher_loop())
    messages = [
        json.loads(call.args[0])
        for call in backend.transport.publish.call_args_list
    ]
    camera = [m for m in messages if m['type'] == 'tool_camera']
    assert len(camera) == (len(frames) if transport == 'websocket' else 0)
    if camera:
        assert camera[0]['payload']['format'] == 'ros_image'
    assert {'telemetry', 'constraints', 'robot_status'} <= {m['type'] for m in messages}


def test_real_launch_never_starts_bag_or_webrtc(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'backend_launch', PACKAGE_ROOT / 'launch' / 'backend.launch.py'
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'mock_camera_actions', Mock(side_effect=AssertionError))
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(PACKAGE_ROOT))
    context = LaunchContext()
    context.launch_configurations.update({
        'robot_type': 'real',
        'mock_camera_transport': 'webrtc',
        'mock_rosbag_path': '/nonexistent',
    })
    actions = module.launch_backend(
        context, str(PACKAGE_ROOT), str(PACKAGE_ROOT / 'config/backend.yaml'), {}
    )
    assert all(not isinstance(action, ExecuteProcess) or isinstance(action, Node)
               for action in actions)
    assert [action.node_package for action in actions if isinstance(action, Node)] == [
        'centauro_hmi_backend', 'cartesian_kinematics_for_hmi_teleoperation',
    ]
