from types import SimpleNamespace

import pytest

from centauro_hmi_backend.ik_bridge import CartesianIkBridge, parse_target_pose


class FakeFuture:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        callback(self)


class FakeGoalHandle:
    accepted = True

    def __init__(self):
        self.canceled = False
        self.result_future = None

    def cancel_goal_async(self):
        self.canceled = True

    def get_result_async(self):
        self.result_future = PendingFuture()
        return self.result_future


class PendingFuture:
    def add_done_callback(self, callback):
        self.callback = callback


class FakeActionClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.goals = []
        self.goal_handle = FakeGoalHandle()

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        self.feedback_callback = feedback_callback
        return FakeFuture(self.goal_handle)


class FakeBroadcaster:
    def __init__(self):
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(transform)


class FakeNode:
    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=0, nanosec=0)))


def make_bridge(ready=True):
    events = []
    client = FakeActionClient(ready)
    broadcaster = FakeBroadcaster()
    bridge = CartesianIkBridge(
        FakeNode(), lambda kind, payload: events.append((kind, payload)),
        'hmi_tcp_target', 'robot_arm_base_link', '/robot/follow_tcp_target', 0.5,
        action_client=client, tf_broadcaster=broadcaster,
    )
    return bridge, client, broadcaster, events


POSE = {'pose': {'position': [0.4, 0.0, 0.5], 'orientation': [0.0, 2.0, 0.0, 0.0]}}


def test_parse_target_pose_normalizes_quaternion_and_uses_default_frame():
    frame_id, position, orientation = parse_target_pose(POSE, 'robot_arm_base_link')
    assert frame_id == 'robot_arm_base_link'
    assert position == [0.4, 0.0, 0.5]
    assert orientation == [0.0, 1.0, 0.0, 0.0]


@pytest.mark.parametrize('pose', [
    {},
    {'pose': {'position': [0.0, 0.0], 'orientation': [0.0, 0.0, 0.0, 1.0]}},
    {'pose': {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0, 0.0]}},
    {'pose': {'position': [float('nan'), 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0, 1.0]}},
])
def test_parse_target_pose_rejects_invalid_payloads(pose):
    with pytest.raises(ValueError):
        parse_target_pose(pose, 'robot_arm_base_link')


def test_target_broadcasts_tf_and_sends_single_continuous_goal():
    bridge, client, broadcaster, _ = make_bridge()

    assert bridge.command(dict(POSE, frame_id='odom'))['accepted']
    assert bridge.command(POSE)['accepted']

    assert [t.header.frame_id for t in broadcaster.transforms] == ['odom', 'robot_arm_base_link']
    assert broadcaster.transforms[0].child_frame_id == 'hmi_tcp_target'
    assert len(client.goals) == 1
    assert client.goals[0].target_frame == 'hmi_tcp_target'
    assert client.goals[0].continuous


def test_target_rejected_when_server_unavailable():
    bridge, _, broadcaster, _ = make_bridge(ready=False)

    result = bridge.command(POSE)

    assert result['error']['code'] == 'ik_unavailable'
    assert len(broadcaster.transforms) == 1


def test_feedback_and_result_are_published_as_ik_solution():
    bridge, client, _, events = make_bridge()
    bridge.command(POSE)

    client.feedback_callback(SimpleNamespace(feedback=SimpleNamespace(
        joint_names=['j1'], positions=[0.1], position_error=0.0,
        orientation_error=0.0, joint_delta_norm=0.01)))
    client.goal_handle.result_future.callback(FakeFuture(SimpleNamespace(result=SimpleNamespace(
        success=True, message='Canceled', joint_names=['j1'], positions=[0.1]))))

    assert events[0] == ('ik_solution', {
        'final': False, 'joint_names': ['j1'], 'positions': [0.1], 'position_error': 0.0,
        'orientation_error': 0.0, 'joint_delta_norm': 0.01,
    })
    assert events[1][1]['final'] and events[1][1]['positions'] == [0.1]


def test_tick_cancels_goal_after_timeout():
    bridge, client, _, _ = make_bridge()
    bridge.command(POSE)
    bridge._last_target -= 1.0

    bridge.tick()

    assert client.goal_handle.canceled
