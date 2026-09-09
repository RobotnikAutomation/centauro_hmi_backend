import asyncio
import json
import threading

from centauro_hmi_backend.node import HmiBackend
from centauro_hmi_backend.robots.mock_robot import MockRobot
from centauro_hmi_backend.stats import InboundStats


class FakeWebSocket:
    def __init__(self):
        self.messages = []


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
