import asyncio
import json
import math
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from centauro_hmi_backend.node import HmiBackend
from centauro_hmi_backend.stats import InboundStats
from centauro_hmi_backend.tf_observation import TfObservationBridge
from centauro_hmi_backend.transport import WebSocketTransport
from geometry_msgs.msg import TransformStamped
from hmi_tfs_manager.action import ObserveTransform
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import websockets


def transform(parent, child, position, rotation=(0.0, 0.0, 0.0, 1.0)):
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    (msg.transform.translation.x, msg.transform.translation.y,
     msg.transform.translation.z) = position
    (msg.transform.rotation.x, msg.transform.rotation.y,
     msg.transform.rotation.z, msg.transform.rotation.w) = rotation
    return msg


@pytest.fixture
def tf_server():
    context = Context()
    rclpy.init(context=context)
    prefix = f'/hmi_tf_observation_test_{uuid.uuid4().hex}'
    node = Node(
        'client', context=context, namespace=prefix, use_global_arguments=False,
        cli_args=['--ros-args', '-r', f'/tf:={prefix}/tf',
                  '-r', f'/tf_static:={prefix}/tf_static'],
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    static = StaticTransformBroadcaster(node)
    # Publish before starting the server to exercise latched TF discovery.
    static.sendTransform([
        transform('base', 'rotated', (1.0, 2.0, 3.0),
                  (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))),
        transform('rotated', 'tool', (2.0, 0.0, 0.0)),
        transform('disconnected_root', 'disconnected_child', (0.0, 0.0, 0.0)),
    ])
    executable = (
        Path(get_package_prefix('hmi_tfs_manager')) / 'lib/hmi_tfs_manager/tf_action_server')
    action_name = prefix + '/observe_transform'
    with tempfile.TemporaryFile(mode='w+') as output:
        server = subprocess.Popen([
            str(executable), '--ros-args',
            '-r', f'observe_transform:={action_name}',
            '-r', f'/tf:={prefix}/tf',
            '-r', f'/tf_static:={prefix}/tf_static',
            '-p', 'use_sim_time:=true',
        ], stdout=output, stderr=subprocess.STDOUT)
        client = ActionClient(node, ObserveTransform, action_name)
        try:
            if not client.wait_for_server(timeout_sec=8.0):
                output.seek(0)
                pytest.fail(f'TF action server did not become ready: {output.read(4000)}')
            assert server.poll() is None
            yield node, client, action_name
        finally:
            client.destroy()
            server.terminate()
            try:
                server.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=5.0)
            executor.shutdown()
            thread.join(timeout=5.0)
            node.destroy_node()
            context.shutdown()


def wait_future(future, timeout=5.0):
    ready = threading.Event()
    future.add_done_callback(lambda _: ready.set())
    assert ready.wait(timeout), 'ROS future timed out'
    return future.result()


@pytest.mark.parametrize('reference,observed,position,quaternion_z', [
    ('base', 'tool', [1.0, 4.0, 3.0], math.sqrt(0.5)),
    ('tool', 'base', [-4.0, 1.0, -3.0], -math.sqrt(0.5)),
    ('base', 'base', [0.0, 0.0, 0.0], 0.0),
])
def test_action_composes_transforms_in_correct_direction(
        tf_server, reference, observed, position, quaternion_z):
    _, client, _ = tf_server
    feedback = []
    handle = wait_future(client.send_goal_async(ObserveTransform.Goal(
        reference_frame=reference, observed_frame=observed),
        feedback_callback=feedback.append))
    assert handle.accepted
    response = wait_future(handle.get_result_async())
    assert response.status == GoalStatus.STATUS_SUCCEEDED
    assert response.result.success
    result = response.result.transform
    assert result.header.frame_id == reference
    assert result.child_frame_id == observed
    assert [result.transform.translation.x, result.transform.translation.y,
            result.transform.translation.z] == pytest.approx(position)
    assert result.transform.rotation.z == pytest.approx(quaternion_z)
    assert feedback and feedback[-1].feedback.transform == result


@pytest.mark.parametrize('changes', [
    {'reference_frame': ''}, {'observed_frame': '/tool'}, {'observed_frame': 'bad frame'},
    {'timeout_sec': 0.0}, {'timeout_sec': float('nan')},
])
def test_action_rejects_invalid_goals(tf_server, changes):
    _, client, _ = tf_server
    goal = ObserveTransform.Goal(reference_frame='base', observed_frame='tool')
    for key, value in changes.items():
        setattr(goal, key, value)
    assert not wait_future(client.send_goal_async(goal)).accepted


@pytest.mark.parametrize('observed', ['missing', 'disconnected_child'])
def test_lookup_timeout_uses_wall_time_with_paused_sim_clock(tf_server, observed):
    _, client, _ = tf_server
    started = time.monotonic()
    handle = wait_future(client.send_goal_async(ObserveTransform.Goal(
        reference_frame='base', observed_frame=observed, timeout_sec=0.25)))
    response = wait_future(handle.get_result_async())
    assert 0.20 <= time.monotonic() - started < 3.0
    assert response.status == GoalStatus.STATUS_ABORTED
    assert not response.result.success
    assert 'TF lookup timed out' in response.result.message
    assert not response.result.transform.header.frame_id


def test_dynamic_tf_feedback_changes_and_cancel_retains_last_transform(tf_server):
    node, client, _ = tf_server
    broadcaster = TransformBroadcaster(node)
    feedback = []
    handle = wait_future(client.send_goal_async(ObserveTransform.Goal(
        reference_frame='base', observed_frame='moving', continuous=True, timeout_sec=2.0),
        feedback_callback=feedback.append))

    def publish_until_x(value):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            msg = transform('base', 'moving', (value, 0.0, 0.0))
            msg.header.stamp = node.get_clock().now().to_msg()
            broadcaster.sendTransform(msg)
            if feedback and feedback[-1].feedback.transform.transform.translation.x == value:
                return
            time.sleep(0.02)
        pytest.fail('Updated dynamic TF did not reach action feedback')

    publish_until_x(1.0)
    publish_until_x(2.0)
    assert wait_future(handle.cancel_goal_async()).goals_canceling
    response = wait_future(handle.get_result_async())
    assert response.status == GoalStatus.STATUS_CANCELED
    assert not response.result.success
    assert response.result.transform.transform.translation.x == 2.0


def test_websocket_one_shot_streaming_concurrent_timeout_cancel_and_disconnect(tf_server):
    node, _, action_name = tf_server

    async def run():
        backend = HmiBackend.__new__(HmiBackend)
        backend.inbound_stats = InboundStats()
        backend.loop = asyncio.get_running_loop()
        backend.event_pub = node.create_publisher(String, 'hmi_events', 10)
        backend.get_logger = node.get_logger
        backend.tf_observation = TfObservationBridge(node, backend._publish_event, action_name)
        backend.transport = WebSocketTransport(
            '127.0.0.1', 0, backend.ws_command, on_disconnect=backend.ws_disconnected)
        await backend.transport.start()
        port = backend.transport._server.sockets[0].getsockname()[1]
        url = f'ws://127.0.0.1:{port}'

        async def send(client, request_id, **payload):
            await client.send(json.dumps({
                'type': 'command', 'request_id': request_id, 'payload': payload,
            }))

        async def receive(client, predicate):
            while True:
                msg = json.loads(await asyncio.wait_for(client.recv(), 4.0))
                if predicate(msg):
                    return msg

        def ack(request_id):
            return lambda msg: msg['type'] == 'ack' and msg.get('request_id') == request_id

        def event(request_id, final):
            return lambda msg: msg['type'] == 'tf_transform' and (
                msg['payload']['request_id'] == request_id and msg['payload']['final'] == final)

        try:
            async with websockets.connect(url) as client, websockets.connect(url) as other:
                await send(client, 'one', name='tf.lookup',
                           reference_frame='base', observed_frame='tool')
                # Ack and feedback can race; keep every message until the final result.
                messages = []
                while not any(event('one', True)(msg) for msg in messages):
                    messages.append(json.loads(await asyncio.wait_for(client.recv(), 4.0)))
                if not any(ack('one')(msg) for msg in messages):
                    messages.append(await receive(client, ack('one')))
                result = next(msg['payload'] for msg in messages if event('one', True)(msg))
                assert result['success']
                assert result['position'] == pytest.approx([1.0, 4.0, 3.0])
                assert result['orientation'] == pytest.approx(
                    [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)])
                assert any(event('one', False)(msg) for msg in messages)
                assert next(msg for msg in messages if ack('one')(msg))['payload']['result'][
                    'observation_id'] == result['observation_id']
                broadcast = await receive(other, event('one', True))
                assert broadcast['payload'] == result

                await send(client, 'stream', name='tf.lookup', reference_frame='base',
                           observed_frame='tool', continuous=True, timeout_sec=0.15)
                stream_ack = await receive(client, ack('stream'))
                stream_id = stream_ack['payload']['result']['observation_id']
                await receive(client, event('stream', False))
                # Static/cached transforms remain valid beyond the lookup timeout.
                await asyncio.sleep(0.3)
                assert stream_id in backend.tf_observation._observations
                await send(client, 'missing', name='tf.lookup', reference_frame='base',
                           observed_frame='missing', timeout_sec=0.2)
                failure = (await receive(client, event('missing', True)))['payload']
                assert not failure['success'] and failure['status'] == 'aborted'
                assert 'position' not in failure
                assert stream_id in backend.tf_observation._observations

                await send(other, 'wrong-owner', name='tf.cancel', observation_id=stream_id)
                rejected = (await receive(other, ack('wrong-owner')))['payload']
                assert rejected['error']['code'] == 'unknown_tf_observation'
                await send(client, 'cancel', name='tf.cancel', observation_id=stream_id)
                cancellation = (await receive(client, event('stream', True)))['payload']
                assert cancellation['status'] == 'canceled'
                assert not cancellation['success']
                assert cancellation['position'] == pytest.approx([1.0, 4.0, 3.0])

                await send(client, 'disconnect', name='tf.lookup', reference_frame='base',
                           observed_frame='tool', continuous=True)
                await receive(client, event('disconnect', False))
                await client.close()
                disconnected = (await receive(other, event('disconnect', True)))['payload']
                assert disconnected['status'] == 'canceled'
                assert not backend.tf_observation._observations
        finally:
            await backend.transport.stop()

    asyncio.run(run())
