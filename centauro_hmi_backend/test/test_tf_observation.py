from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from centauro_hmi_backend.tf_observation import TfObservationBridge
from geometry_msgs.msg import TransformStamped
from hmi_tfs_manager.action import ObserveTransform
import pytest


def completed(value=None, error=None):
    future = Future()
    if error is not None:
        future.set_exception(error)
    else:
        future.set_result(value)
    return future


def make_bridge(ready=True, pending=False):
    events = []
    client = Mock()
    client.server_is_ready.return_value = ready
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = Future()
    handle.cancel_goal_async.return_value = completed(SimpleNamespace(goals_canceling=[1]))
    goal_future = Future() if pending else completed(handle)
    client.send_goal_async.return_value = goal_future
    bridge = TfObservationBridge(
        Mock(), lambda kind, payload: events.append((kind, payload)),
        '/observe_transform', action_client=client,
    )
    return bridge, client, handle, goal_future, events


PAYLOAD = {'reference_frame': 'base', 'observed_frame': 'tool'}


@pytest.mark.parametrize('extra', [
    {'reference_frame': ''}, {'reference_frame': 3}, {'observed_frame': None},
    {'observed_frame': '/tool'}, {'observed_frame': 'two frames'},
    {'observed_frame': 'tool\n'}, {'continuous': 'true'},
    {'timeout_sec': 0}, {'timeout_sec': -1}, {'timeout_sec': True},
    {'timeout_sec': '2'}, {'timeout_sec': float('nan')},
    {'timeout_sec': float('inf')}, {'timeout_sec': 10 ** 500},
])
def test_invalid_goal_is_explicitly_rejected(extra):
    bridge, client, _, _, _ = make_bridge()
    response = bridge.command({**PAYLOAD, **extra})
    assert response['error']['code'] == 'invalid_tf_request'
    client.send_goal_async.assert_not_called()


def test_missing_frames_and_unavailable_server():
    bridge, _, _, _, _ = make_bridge()
    assert not bridge.command({})['accepted']
    bridge, _, _, _, _ = make_bridge(ready=False)
    assert bridge.command(PAYLOAD)['error']['code'] == 'tf_unavailable'


def test_default_goal_feedback_and_result_have_exact_coordinates_and_correlation():
    bridge, client, handle, _, events = make_bridge()
    response = bridge.command(PAYLOAD, 'req-1')
    observation_id = response['result']['observation_id']
    goal = client.send_goal_async.call_args.args[0]
    assert (goal.reference_frame, goal.observed_frame) == ('base', 'tool')
    assert not goal.continuous
    assert goal.timeout_sec == 5.0
    transform = TransformStamped()
    transform.header.frame_id = 'base'
    transform.child_frame_id = 'tool'
    transform.header.stamp.sec = 42
    transform.transform.translation.x = 1.5
    transform.transform.rotation.w = 1.0
    callback = client.send_goal_async.call_args.kwargs['feedback_callback']
    callback(SimpleNamespace(feedback=ObserveTransform.Feedback(transform=transform)))
    handle.get_result_async.return_value.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=ObserveTransform.Result(
            success=True, message='Transform available', transform=transform),
    ))
    assert [kind for kind, _ in events] == ['tf_transform', 'tf_transform']
    feedback, result = [payload for _, payload in events]
    assert feedback == {
        'observation_id': observation_id, 'request_id': 'req-1',
        'reference_frame': 'base', 'observed_frame': 'tool',
        'final': False, 'success': True,
        'stamp': {'sec': 42, 'nanosec': 0},
        'position': [1.5, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0, 1.0],
    }
    assert result == {**feedback, 'final': True, 'status': 'succeeded',
                      'message': 'Transform available'}
    assert not bridge._observations


def test_cancel_before_acceptance_and_disconnect_cancel_all_only_owned_goals():
    bridge, client, handle, future, _ = make_bridge(pending=True)
    owner = object()
    first = bridge.command({**PAYLOAD, 'continuous': True}, owner=owner)
    observation_id = first['result']['observation_id']
    assert client.send_goal_async.call_args.args[0].continuous
    assert bridge.cancel({'observation_id': observation_id}, owner)['accepted']
    handle.cancel_goal_async.assert_not_called()
    future.set_result(handle)
    handle.cancel_goal_async.assert_called_once()

    client.send_goal_async.return_value = completed(handle)
    bridge.command(PAYLOAD, owner=owner)
    bridge.command(PAYLOAD, owner=object())
    handle.cancel_goal_async.reset_mock()
    bridge.cancel_owner(owner)
    assert handle.cancel_goal_async.call_count == 2
    assert bridge.cancel({'observation_id': observation_id}, object())['error']['code'] == (
        'unknown_tf_observation')


@pytest.mark.parametrize('payload', [{}, {'observation_id': []}, {'observation_id': ''}])
def test_cancel_requires_valid_id(payload):
    bridge, _, _, _, _ = make_bridge()
    assert bridge.cancel(payload)['error']['code'] == 'invalid_tf_request'


@pytest.mark.parametrize('status,success,expected', [
    (GoalStatus.STATUS_CANCELED, False, 'canceled'),
    (GoalStatus.STATUS_ABORTED, False, 'aborted'),
    (GoalStatus.STATUS_ABORTED, True, 'aborted'),
])
def test_terminal_status_controls_success_and_omits_missing_transform(status, success, expected):
    bridge, _, handle, _, events = make_bridge()
    bridge.command(PAYLOAD)
    handle.get_result_async.return_value.set_result(SimpleNamespace(
        status=status, result=ObserveTransform.Result(success=success, message='Stopped')))
    result = events[-1][1]
    assert result['status'] == expected and not result['success']
    assert 'position' not in result
    assert not bridge._observations


@pytest.mark.parametrize('phase', ['send', 'accept', 'reject', 'result', 'cancel'])
def test_action_failures_are_reported(phase):
    bridge, client, handle, future, events = make_bridge(pending=True)
    if phase == 'send':
        client.send_goal_async.side_effect = RuntimeError('Send failed')
    response = bridge.command(PAYLOAD)
    if phase == 'send':
        assert response['error']['code'] == 'tf_unavailable'
    else:
        if phase == 'accept':
            future.set_exception(RuntimeError('Accept failed'))
        elif phase == 'reject':
            future.set_result(Mock(accepted=False))
        else:
            future.set_result(handle)
            if phase == 'result':
                handle.get_result_async.return_value.set_exception(RuntimeError('Result failed'))
            else:
                handle.cancel_goal_async.return_value = completed(
                    error=RuntimeError('Cancel failed'))
                bridge.cancel({'observation_id': response['result']['observation_id']})
        assert not events[-1][1]['success']
        assert events[-1][1]['final'] == (phase != 'cancel')
        assert events[-1][1]['message']
    if phase != 'cancel':
        assert not bridge._observations
