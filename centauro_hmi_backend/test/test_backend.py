import asyncio
import json
import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.time import Time

from centauro_hmi_backend.node import HmiBackend
from centauro_hmi_backend.robots.mock_robot import MockRobot
from centauro_hmi_backend.stats import InboundStats


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
