import asyncio
import base64
import importlib.util
import json
import math
from pathlib import Path
import struct
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest
import rosbag2_py
from launch import LaunchContext
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import websockets

from centauro_hmi_backend.node import HmiBackend
from centauro_hmi_backend.point_cloud import (
    DEFAULT_POINTCLOUD_ROSBAG_PATH, DEFAULT_SCANS_ROSBAG_PATH,
    MockPointCloudReplay, create_mock_point_cloud_replay,
    point_cloud_payload, scan_to_point_cloud,
)
from centauro_hmi_backend.transport import WebSocketTransport


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def scan():
    msg = LaserScan(
        angle_min=0.0, angle_increment=math.pi / 2,
        range_min=0.5, range_max=2.0,
        ranges=[1.0, 2.0, float('nan'), float('inf'), -1.0, 0.25, 3.0],
    )
    msg.header.frame_id = 'laser'
    msg.header.stamp.sec = 42
    return msg


def cloud():
    msg = PointCloud2(
        height=1, width=1, point_step=16, row_step=20,
        fields=[
            PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(('x', 'y', 'z', 'intensity'))
        ],
        is_bigendian=True, is_dense=False,
        data=struct.pack('>ffff', 1.0, 2.0, float('nan'), 4.0) + b'pad!',
    )
    msg.header.frame_id = 'lidar'
    msg.header.stamp.sec = 43
    return msg


@pytest.fixture
def bag(tmp_path):
    def write(msg, count=2):
        path = str(tmp_path / msg.__class__.__name__)
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'),
        )
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name='/recorded', type=f'sensor_msgs/msg/{msg.__class__.__name__}',
            serialization_format='cdr',
        ))
        for index in range(count):
            writer.write('/recorded', serialize_message(msg), 1_000_000_000 + index * 20_000_000)
        del writer
        return path
    return write


def test_scan_projection_filters_invalid_ranges_and_preserves_frame():
    msg = scan()
    result = scan_to_point_cloud(msg)
    assert result.header == msg.header
    assert (result.width, result.height, result.point_step, result.row_step) == (2, 1, 12, 24)
    assert [f.name for f in result.fields] == ['x', 'y', 'z']
    assert struct.unpack('<ffffff', bytes(result.data)) == pytest.approx(
        (1, 0, 0, 0, 2, 0), abs=1e-6,
    )
    assert result.is_dense and not result.is_bigendian


def test_empty_scan_produces_valid_empty_point_cloud():
    msg = scan()
    msg.ranges = [float('inf')]
    result = scan_to_point_cloud(msg)
    assert result.width == result.row_step == 0
    assert bytes(result.data) == b''


def test_native_cloud_is_lossless_including_fields_endianness_and_padding():
    msg = cloud()
    payload = point_cloud_payload(msg, '/recorded', 'pointcloud')
    assert base64.b64decode(payload['data']) == bytes(msg.data)
    assert payload['fields'][-1] == {
        'name': 'intensity', 'offset': 12, 'datatype': 7, 'count': 1,
    }
    assert payload['is_bigendian'] and not payload['is_dense']
    assert payload['row_step'] == 20
    assert payload['timestamp'] == {'sec': 43, 'nanosec': 0}
    assert payload['frame_id'] == 'lidar'


@pytest.mark.parametrize('source,make_msg', [('scans', scan), ('pointcloud', cloud)])
def test_reader_selects_type_and_converts_only_scans(bag, source, make_msg):
    msg = make_msg()
    replay = MockPointCloudReplay(source, bag(msg))
    timestamp, payload = replay.read_frame()
    # LaserScan angles are rounded to float32 by CDR serialization.
    recorded = deserialize_message(serialize_message(msg), type(msg))
    expected = scan_to_point_cloud(recorded) if source == 'scans' else recorded
    assert timestamp == 1_000_000_000
    assert payload == point_cloud_payload(expected, '/recorded', source)
    assert replay.read_frame() is not None
    assert replay.read_frame() is None


def test_selected_bag_must_exist_and_contain_expected_type(bag, tmp_path):
    with pytest.raises(FileNotFoundError, match='rosbag not found'):
        MockPointCloudReplay('scans', str(tmp_path / 'missing'))
    with pytest.raises(ValueError, match='has no sensor_msgs/msg/LaserScan'):
        MockPointCloudReplay('scans', bag(cloud()))
    with pytest.raises(ValueError, match='has no sensor_msgs/msg/LaserScan'):
        MockPointCloudReplay('scans', bag(scan(), count=0))


@pytest.mark.parametrize('source', ['none', 'scans', 'pointcloud'])
def test_real_mode_never_opens_mock_bags(source):
    assert create_mock_point_cloud_replay('real', source, '/missing', '/missing') is None


def test_disabled_mode_does_not_open_bags():
    assert create_mock_point_cloud_replay('mock', 'none', '/missing', '/missing') is None


@pytest.mark.parametrize('source', ['both', '', 'scan', 'scans,pointcloud'])
def test_invalid_or_multiple_sources_rejected(source):
    with pytest.raises(ValueError, match='must be none, scans or pointcloud'):
        create_mock_point_cloud_replay('mock', source, '/missing', '/missing')


@pytest.mark.parametrize('source,make_msg', [('scans', scan), ('pointcloud', cloud)])
def test_only_selected_path_is_opened(bag, source, make_msg):
    path = bag(make_msg())
    paths = (path, '/missing') if source == 'scans' else ('/missing', path)
    replay = create_mock_point_cloud_replay('mock', source, *paths)
    assert replay.source == source
    assert replay.path == path


async def receive_loop(replay, expected_count):
    backend = HmiBackend.__new__(HmiBackend)
    backend.mock_point_cloud = replay
    backend.get_logger = lambda: Mock()
    backend.transport = WebSocketTransport('127.0.0.1', 0, AsyncMock())
    await backend.transport.start()
    task = None
    try:
        port = backend.transport._server.sockets[0].getsockname()[1]
        # Native clouds in the supplied recording exceed the default 1 MiB limit.
        async with websockets.connect(f'ws://127.0.0.1:{port}', max_size=None) as client:
            task = asyncio.create_task(backend.point_cloud_loop())
            frames = []
            received_at = []
            while True:
                raw = await asyncio.wait_for(client.recv(), timeout=20.0)
                msg = json.loads(raw)
                assert msg['type'] == 'point_cloud'
                frames.append(msg['payload'])
                received_at.append(asyncio.get_running_loop().time())
                if frames[-1]['loop_index'] == 1:
                    break
            assert len(frames) == expected_count + 1
            assert [f['sequence'] for f in frames] == list(range(1, expected_count + 2))
            assert {f['source'] for f in frames} == {replay.source}
            assert frames[0]['data'] == frames[-1]['data']
            assert frames[0]['timestamp'] == frames[-1]['timestamp']
            assert frames[0]['frame_id'] == frames[-1]['frame_id']
            assert all(f['loop_index'] == 0 for f in frames[:-1])
            return received_at
    finally:
        if task is not None:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        await backend.transport.stop()


@pytest.mark.parametrize('source,make_msg', [('scans', scan), ('pointcloud', cloud)])
@pytest.mark.parametrize('count', [1, 3])
def test_websocket_replay_loops_at_recorded_speed(bag, source, make_msg, count):
    replay = MockPointCloudReplay(source, bag(make_msg(), count=count))
    received_at = asyncio.run(receive_loop(replay, count))
    assert received_at[-1] - received_at[0] >= (count - 1) * 0.02


@pytest.mark.parametrize('source,path,count,duration', [
    ('scans', DEFAULT_SCANS_ROSBAG_PATH, 554, 10.747902596),
    ('pointcloud', DEFAULT_POINTCLOUD_ROSBAG_PATH, 62, 6.059184458),
])
def test_supplied_recordings_over_websocket(source, path, count, duration):
    if not Path(path).exists():
        pytest.skip('Host recording is not available')
    received_at = asyncio.run(receive_loop(MockPointCloudReplay(source, path), count))
    elapsed = received_at[-2] - received_at[0]
    assert elapsed == pytest.approx(duration, abs=0.5)


def test_replay_failure_is_logged_and_sent_to_websocket():
    backend = HmiBackend.__new__(HmiBackend)
    backend.mock_point_cloud = SimpleNamespace(run=AsyncMock(side_effect=ValueError('bad data')))
    backend.transport = SimpleNamespace(publish=AsyncMock())
    logger = Mock()
    backend.get_logger = lambda: logger
    asyncio.run(backend.point_cloud_loop())
    logger.error.assert_called_once()
    response = json.loads(backend.transport.publish.call_args.args[0])
    assert response['type'] == 'error'
    assert response['payload'] == {'code': 'point_cloud_replay_failed', 'message': 'bad data'}


def test_shutdown_cancels_replay_and_telemetry_before_stopping_transport():
    async def check():
        backend = HmiBackend.__new__(HmiBackend)
        backend.transport = SimpleNamespace(stop=AsyncMock())
        backend._point_cloud_task = asyncio.create_task(asyncio.sleep(60))
        backend._publisher_task = asyncio.create_task(asyncio.sleep(60))
        await backend._shutdown_async()
        assert backend._point_cloud_task.cancelled()
        assert backend._publisher_task.cancelled()
        backend.transport.stop.assert_awaited_once()
    asyncio.run(check())


def test_launch_forwards_source_and_both_paths(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'point_cloud_launch', PACKAGE_ROOT / 'launch/backend.launch.py',
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(PACKAGE_ROOT))
    monkeypatch.setattr(module, 'mock_camera_actions', Mock(return_value=[]))
    node = Mock()
    monkeypatch.setattr(module, 'Node', node)
    context = LaunchContext()
    context.launch_configurations.update({
        'robot_type': 'mock', 'mock_camera_transport': 'websocket',
        'mock_rosbag_path': '/camera', 'mock_point_cloud_source': 'scans',
        'mock_scans_rosbag_path': '/scans', 'mock_pointcloud_rosbag_path': '/cloud',
    })
    module.launch_backend(context, str(PACKAGE_ROOT), '/backend.yaml', {})
    params = node.call_args_list[0].kwargs['parameters'][1]
    assert params['mock.point_cloud_source'] == 'scans'
    assert params['mock.scans_rosbag_path'] == '/scans'
    assert params['mock.pointcloud_rosbag_path'] == '/cloud'
