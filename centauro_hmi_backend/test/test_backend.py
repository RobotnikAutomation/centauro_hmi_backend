import asyncio
import json
import math
import threading
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest
from rclpy.time import Time
import websockets

from centauro_hmi_backend.node import HmiBackend
from centauro_hmi_backend.robots.mock_robot import MockRobot
from centauro_hmi_backend.stats import InboundStats
from centauro_hmi_backend.transport import WebSocketTransport


class FakeWebSocket:
    def __init__(self):
        self.messages = []
        self.remote_address = ('127.0.0.1', 12345)


class FakeTransport:
    async def send_to_client(self, client, raw):
        client.messages.append(raw)
        return True


class FakeModelProvider:
    def manifest(self):
        return {'model_id': 'model-1', 'size': 3, 'sha256': 'hash', 'chunk_size': 3}

    def bundle(self):
        return b'zip'

    def chunks(self):
        return iter([{'model_id': 'model-1', 'sequence': 0, 'total': 1, 'data': 'emlw'}])


def make_backend():
    backend = HmiBackend.__new__(HmiBackend)
    backend.lock = threading.Lock()
    backend.robot = MockRobot()
    backend.transport = FakeTransport()
    backend.robot_model = FakeModelProvider()
    backend.robot_description = None
    backend.tf_frames = Mock()
    backend.tf_observation = Mock()
    backend.tf_frames.snapshot.return_value = {'frames': []}
    backend._last_tf_frames = None
    backend.inbound_stats = InboundStats()
    backend.events = []
    backend._publish_event = lambda kind, payload: backend.events.append((kind, payload))
    return backend


def send(backend, websocket, data):
    asyncio.run(backend.ws_command(websocket, json.dumps(data)))


def test_command_returns_ack_with_original_request_id():
    backend = make_backend()
    websocket = FakeWebSocket()

    send(backend, websocket, {
        'type': 'command',
        'request_id': 'req-1',
        'payload': {'name': 'teleoperation.enable'},
    })

    response = json.loads(websocket.messages[0])
    assert response['type'] == 'ack'
    assert response['request_id'] == 'req-1'
    assert response['payload'] == {'name': 'teleoperation.enable', 'accepted': True}


@pytest.mark.parametrize('name,method', [('tf.lookup', 'command'), ('tf.cancel', 'cancel')])
def test_tf_commands_forward_to_action_bridge(name, method):
    backend = make_backend()
    websocket = FakeWebSocket()
    bridge_method = getattr(backend.tf_observation, method)
    bridge_method.return_value = {'accepted': True, 'result': {'observation_id': 'obs-1'}}
    payload = {'name': name, 'reference_frame': 'world', 'observed_frame': 'tool0'}

    send(backend, websocket, {'request_id': 'tf-1', 'payload': payload})

    if method == 'command':
        bridge_method.assert_called_once_with(payload, 'tf-1', websocket)
    else:
        bridge_method.assert_called_once_with(payload, websocket)
    response = json.loads(websocket.messages[0])
    assert response['request_id'] == 'tf-1'
    assert response['payload']['result']['observation_id'] == 'obs-1'


def test_disconnect_cancels_clients_tf_observations():
    backend = make_backend()
    websocket = FakeWebSocket()
    backend.transport.clients = set()
    backend.get_logger = lambda: Mock()

    asyncio.run(backend.ws_disconnected(websocket))

    backend.tf_observation.cancel_owner.assert_called_once_with(websocket)


def test_invalid_json_returns_invalid_command_error():
    backend = make_backend()
    websocket = FakeWebSocket()

    asyncio.run(backend.ws_command(websocket, '{'))

    response = json.loads(websocket.messages[0])
    assert response['type'] == 'error'
    assert response['payload']['code'] == 'invalid_command'


def test_new_websocket_client_receives_current_real_robot_status():
    backend = make_backend()
    websocket = FakeWebSocket()
    backend.transport.clients = {websocket}
    backend.get_logger = lambda: type('Logger', (), {'info': lambda self, _: None})()
    backend.real_status = type(
        'StatusBridge',
        (),
        {'snapshot': lambda self: ({'robot_mode': {'mode': 7}}, [])},
    )()

    asyncio.run(backend.ws_connected(websocket))

    response = json.loads(websocket.messages[0])
    assert response['type'] == 'robot_status'
    assert response['payload']['robot_mode']['mode'] == 7


def test_new_mock_websocket_client_receives_robot_description():
    backend = make_backend()
    websocket = FakeWebSocket()
    backend.transport.clients = {websocket}
    backend.get_logger = lambda: Mock()
    backend.real_status = None
    backend.robot_description = '<robot name="rbkairos_plus"/>'

    asyncio.run(backend.ws_connected(websocket))

    response = json.loads(websocket.messages[0])
    assert response['type'] == 'robot_description'
    assert response['payload'] == {
        'topic': '/robot/robot_description',
        'data': backend.robot_description,
    }


def test_periodic_command_does_not_return_ack():
    backend = make_backend()
    websocket = FakeWebSocket()

    send(backend, websocket, {
        'type': 'command',
        'payload': {
            'name': 'teleoperation.deadman',
            'active': True,
        },
    })

    assert websocket.messages == []
    assert backend.robot.deadman


@pytest.mark.parametrize('robot_type', ['real', 'mock'])
def test_every_websocket_connection_receives_current_tf_frames(robot_type):
    backend = make_backend()
    backend.transport.clients = set()
    backend.get_logger = lambda: Mock()
    backend.real_status = Mock() if robot_type == 'real' else None
    if backend.real_status is not None:
        backend.real_status.snapshot.return_value = ({}, [])
    frames = {'frames': ['base_link', 'tool0', 'world']}
    backend.tf_frames.snapshot.return_value = frames
    backend._last_tf_frames = frames

    for _ in range(2):
        websocket = FakeWebSocket()
        asyncio.run(backend.ws_connected(websocket))
        response = json.loads(websocket.messages[-1])
        assert response['type'] == 'tf_frames'
        assert response['payload'] == frames


@pytest.mark.parametrize('robot_type', ['real', 'mock'])
def test_publisher_sends_tf_frames_only_when_list_changes(monkeypatch, robot_type):
    from centauro_hmi_backend import node as node_module

    backend = make_backend()
    backend.transport.publish = AsyncMock()
    backend.get_parameter = lambda _: SimpleNamespace(value=20.0)
    backend.real_status = Mock() if robot_type == 'real' else None
    if backend.real_status is not None:
        backend.real_status.snapshot.return_value = ({}, [])
    backend.mock_camera = None
    initial = {'frames': []}
    discovered = {'frames': ['base_link', 'world']}
    cleared = {'frames': []}
    backend.tf_frames.snapshot.side_effect = [initial, discovered, discovered, cleared]
    monkeypatch.setattr(node_module.rclpy, 'ok', Mock(side_effect=[True] * 4 + [False]))
    monkeypatch.setattr(node_module.asyncio, 'sleep', AsyncMock())

    asyncio.run(backend.publisher_loop())

    messages = [
        json.loads(call.args[0])
        for call in backend.transport.publish.call_args_list
    ]
    assert [
        msg['payload'] for msg in messages if msg['type'] == 'tf_frames'
    ] == [initial, discovered, cleared]
    assert len([msg for msg in messages if msg['type'] == 'telemetry']) == 4


@pytest.mark.parametrize('robot_type', ['real', 'mock'])
def test_tf_frames_reach_websocket_clients_and_reconnects(monkeypatch, robot_type):
    from centauro_hmi_backend import node as node_module

    backend = make_backend()
    backend.get_logger = lambda: Mock()
    backend.get_parameter = lambda _: SimpleNamespace(value=20.0)
    backend.real_status = Mock() if robot_type == 'real' else None
    if backend.real_status is not None:
        backend.real_status.snapshot.return_value = ({}, [])
    backend._last_real_robot_status = None
    backend.mock_camera = None
    monkeypatch.setattr(node_module.rclpy, 'ok', lambda: True)
    backend.transport = WebSocketTransport(
        '127.0.0.1', 0, backend.ws_command, backend.ws_connected,
    )

    async def receive_frames(client):
        while True:
            msg = json.loads(await client.recv())
            if msg['type'] == 'tf_frames':
                return msg['payload']

    async def run():
        await backend.transport.start()
        publisher = asyncio.create_task(backend.publisher_loop())
        port = backend.transport._server.sockets[0].getsockname()[1]
        url = f'ws://127.0.0.1:{port}'
        try:
            async with websockets.connect(url) as first, websockets.connect(url) as second:
                assert await asyncio.wait_for(receive_frames(first), 2.0) == {'frames': []}
                assert await asyncio.wait_for(receive_frames(second), 2.0) == {'frames': []}
                expected = {'frames': ['base_link', 'tool0', 'world']}
                backend.tf_frames.snapshot.return_value = expected
                for client in (first, second):
                    while await asyncio.wait_for(receive_frames(client), 2.0) != expected:
                        pass
            async with websockets.connect(url) as reconnected:
                assert await asyncio.wait_for(receive_frames(reconnected), 2.0) == expected
        finally:
            publisher.cancel()
            await asyncio.gather(publisher, return_exceptions=True)
            await backend.transport.stop()

    asyncio.run(run())


def test_mock_base_joystick_uses_periodic_websocket_interface():
    backend = make_backend()
    websocket = FakeWebSocket()

    send(backend, websocket, {
        'type': 'command',
        'payload': {
            'name': 'base.joystick',
            'twist': {'linear': [0.5, -0.25, 0.0], 'angular': [0.0, 0.0, 0.4]},
        },
    })

    assert websocket.messages == []
    assert backend.robot._base_velocity == pytest.approx([0.5, -0.25, 0.6])


def test_invalid_mock_base_joystick_returns_protocol_error():
    backend = make_backend()
    websocket = FakeWebSocket()

    send(backend, websocket, {
        'type': 'command',
        'payload': {'name': 'base.joystick', 'twist': {'linear': [2, 0, 0], 'angular': [0, 0, 0]}},
    })

    assert json.loads(websocket.messages[0])['payload']['code'] == 'invalid_base_joystick'
    assert backend.inbound_stats.take_interval()['errors'] == 1


def test_sim_base_tf_uses_elapsed_time_and_keeps_publishing_when_stopped():
    backend = make_backend()
    now = Time(seconds=10.0)
    backend.get_clock = lambda: SimpleNamespace(now=lambda: now)
    backend._last_sim_base_update = Time(seconds=9.8)
    backend._sim_base_broadcaster = Mock()
    backend.robot.command('base.joystick', {
        'twist': {'linear': [0.5, -0.25, 0.0], 'angular': [0.0, 0.0, 0.0]},
    })

    backend.publish_sim_base()

    transform = backend._sim_base_broadcaster.sendTransform.call_args.args[0]
    assert transform.header.stamp == now.to_msg()
    assert transform.header.frame_id == 'base_link'
    assert transform.child_frame_id == 'sim_base'
    assert transform.transform.translation.x == pytest.approx(0.1)
    assert transform.transform.translation.y == pytest.approx(-0.05)
    assert transform.transform.translation.z == 0.0
    assert transform.transform.rotation.w == 1.0

    backend.robot._last_base_command -= 1.0
    now = Time(seconds=10.2)
    backend.publish_sim_base()

    assert backend._sim_base_broadcaster.sendTransform.call_count == 2
    stopped = backend._sim_base_broadcaster.sendTransform.call_args.args[0]
    assert stopped.header.stamp == now.to_msg()
    assert stopped.transform == transform.transform


def test_sim_base_tf_encodes_yaw_as_unit_quaternion():
    backend = make_backend()
    now = Time(seconds=10.0)
    backend.get_clock = lambda: SimpleNamespace(now=lambda: now)
    backend._last_sim_base_update = now
    backend._sim_base_broadcaster = Mock()
    backend.robot.base_pose = [1.0, 2.0, math.pi / 2.0]

    backend.publish_sim_base()

    transform = backend._sim_base_broadcaster.sendTransform.call_args.args[0]
    rotation = transform.transform.rotation
    assert [rotation.x, rotation.y, rotation.z, rotation.w] == pytest.approx(
        [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]
    )


def test_command_without_name_is_rejected_and_counted_as_error():
    backend = make_backend()
    websocket = FakeWebSocket()

    send(backend, websocket, {'type': 'command', 'payload': {}})

    assert json.loads(websocket.messages[0])['payload']['code'] == 'invalid_command'
    assert backend.inbound_stats.take_interval()['errors'] == 1


def test_model_get_returns_manifest():
    backend = make_backend()
    websocket = FakeWebSocket()

    send(backend, websocket, {'type': 'command', 'request_id': 'model-1', 'payload': {'name': 'robot.model.get'}})

    response = json.loads(websocket.messages[0])
    assert response['payload']['result']['model_id'] == 'model-1'


def test_model_download_sends_ack_then_chunks():
    backend = make_backend()
    websocket = FakeWebSocket()

    send(backend, websocket, {'type': 'command', 'request_id': 'model-2', 'payload': {'name': 'robot.model.download'}})

    assert json.loads(websocket.messages[0])['type'] == 'ack'
    assert json.loads(websocket.messages[1])['type'] == 'robot_model_chunk'


def test_shutdown_timeout_still_stops_loop_and_destroys_node(monkeypatch):
    from unittest.mock import Mock
    from centauro_hmi_backend import node as node_module

    # Python 3.10's concurrent.futures.TimeoutError isn't builtins.TimeoutError.
    class LegacyFutureTimeoutError(Exception):
        pass

    monkeypatch.setattr(node_module, 'FutureTimeoutError', LegacyFutureTimeoutError)
    backend = HmiBackend.__new__(HmiBackend)
    backend._shutting_down = False
    backend.loop = Mock()
    future = Mock()
    future.result.side_effect = LegacyFutureTimeoutError

    def submit(coroutine, loop):
        coroutine.close()
        return future

    monkeypatch.setattr(node_module.asyncio, 'run_coroutine_threadsafe', submit)
    destroy = Mock()
    monkeypatch.setattr(node_module.Node, 'destroy_node', destroy)

    backend.destroy_node()

    future.cancel.assert_called_once_with()
    backend.loop.call_soon_threadsafe.assert_called_once_with(backend.loop.stop)
    destroy.assert_called_once_with()
