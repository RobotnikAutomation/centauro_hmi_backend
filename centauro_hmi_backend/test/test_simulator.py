import math

import pytest

from centauro_hmi_backend.robots.mock_robot import MockRobot, tool_position


def base_joystick(x=0.0, y=0.0, yaw=0.0):
    return {'twist': {'linear': [x, y, 0.0], 'angular': [0.0, 0.0, yaw]}}


def test_base_translates_without_arm_authorization_or_speed_scaling():
    sim = MockRobot(speed_percentage=1.0)
    joints = list(sim.joints)

    assert sim.command('base.joystick', base_joystick(0.5, -0.25))['accepted']
    sim.tick_base(0.2)

    assert sim.base_pose == pytest.approx([0.1, -0.05, 0.0])
    assert sim.joints == joints
    assert not sim.enabled and not sim.deadman


def test_base_uses_configured_speed_limits():
    sim = MockRobot(base_max_linear_speed=2.0, base_max_angular_speed=3.0)
    sim.command('base.joystick', base_joystick(0.5, -0.25, 0.4))

    assert sim._base_velocity == pytest.approx([1.0, -0.5, 1.2])


def test_base_translation_is_relative_to_current_heading():
    sim = MockRobot(base_max_angular_speed=math.pi)
    sim.command('base.joystick', base_joystick(yaw=1.0))
    sim.tick_base(0.5)
    sim.command('base.joystick', base_joystick(1.0, 0.5))
    sim.tick_base(0.2)

    assert sim.base_pose == pytest.approx([-0.1, 0.2, math.pi / 2.0])


def test_base_integrates_simultaneous_translation_and_rotation():
    sim = MockRobot(base_max_angular_speed=math.pi)
    sim.command('base.joystick', base_joystick(1.0, yaw=1.0))
    sim.tick_base(0.5)

    assert sim.base_pose == pytest.approx([1.0 / math.pi, 1.0 / math.pi, math.pi / 2.0])


def test_base_watchdog_stops_and_fresh_input_resumes(monkeypatch):
    now = [100.0]
    monkeypatch.setattr('centauro_hmi_backend.robots.mock_robot.time.monotonic', lambda: now[0])
    sim = MockRobot(command_timeout_sec=0.5)
    sim.command('base.joystick', base_joystick(1.0, -0.5, 0.4))
    sim.tick_base(0.1)
    pose = list(sim.base_pose)
    now[0] += 0.6
    sim.tick_base(0.1)
    sim.tick_base(0.1)

    assert sim.base_pose == pose
    assert sim._base_velocity == [0.0, 0.0, 0.0]
    sim.command('base.joystick', base_joystick(1.0))
    sim.tick_base(0.1)
    assert sim.base_pose != pose


def test_zero_base_command_stops_immediately():
    sim = MockRobot()
    sim.command('base.joystick', base_joystick(1.0))
    sim.tick_base(0.1)
    sim.command('base.joystick', base_joystick())
    sim.tick_base(0.1)

    assert sim.base_pose == [0.1, 0.0, 0.0]


@pytest.mark.parametrize('payload', [
    {},
    {'twist': None},
    {'twist': {'linear': [0.0, 0.0], 'angular': [0.0] * 3}},
    {'twist': {'linear': [0.0] * 3, 'angular': 'invalid'}},
    base_joystick(1.1),
    base_joystick(y=-1.1),
    base_joystick(yaw=float('nan')),
    base_joystick(float('inf')),
    base_joystick('not-a-number'),
    base_joystick(None),
    {'twist': {'linear': [0.0, 0.0, 0.1], 'angular': [0.0] * 3}},
    {'twist': {'linear': [0.0] * 3, 'angular': [0.1, 0.0, 0.0]}},
    {'twist': {'linear': [0.0] * 3, 'angular': [0.0, 0.1, 0.0]}},
])
def test_invalid_base_command_does_not_replace_velocity_or_refresh_watchdog(payload):
    sim = MockRobot()
    sim.command('base.joystick', base_joystick(0.5))
    last_command = sim._last_base_command

    result = sim.command('base.joystick', payload)

    assert result['accepted'] is False
    assert result['error']['code'] == 'invalid_base_joystick'
    assert sim._base_velocity == [0.5, 0.0, 0.0]
    assert sim._last_base_command == last_command


def test_tool_position_matches_urdf_joint_chain():
    assert tool_position([0.0] * 6) == (0.0, 0.0, -0.92)
    position = tool_position([0.0, -1.5707963267948966, 0.0, -1.5707963267948966, 0.0, 0.0])
    assert position[0] == pytest.approx(0.82)
    assert position[1] == pytest.approx(0.0)
    assert position[2] == pytest.approx(0.66)


def test_deadman_blocks_and_allows_joint_jog():
    sim = MockRobot()
    sim.command('teleoperation.enable', {})
    sim.command('arm.joint_jog', {'velocities': [1, 0, 0, 0, 0, 0]})
    sim.tick(0.1)
    assert sim.joints[0] == 0.0
    sim.command('teleoperation.deadman', {'active': True})
    sim.command('arm.joint_jog', {'velocities': [1, 0, 0, 0, 0, 0]})
    sim.tick(0.1)
    assert sim.joints[0] > 0.0


def test_disable_clears_deadman():
    sim = MockRobot()
    sim.command('teleoperation.enable', {})
    sim.command('teleoperation.deadman', {'active': True})
    sim.command('teleoperation.disable', {})
    assert not sim.enabled and not sim.deadman


def test_freedrive_requires_periodic_command():
    sim = MockRobot(command_timeout_sec=0.5)
    sim.command('teleoperation.freedrive', {'active': True})
    sim.tick(0.1)
    assert sim.freedrive

    sim.last_freedrive_command -= 0.6
    sim.tick(0.1)
    assert not sim.freedrive
    assert sim.last_status_reason == 'freedrive_timeout'


def test_freedrive_periodic_command_refreshes_watchdog():
    sim = MockRobot(command_timeout_sec=0.5)
    sim.command('teleoperation.freedrive', {'active': True})
    sim.last_freedrive_command -= 0.4
    sim.command('teleoperation.freedrive', {'active': True})
    sim.tick(0.1)
    assert sim.freedrive


def test_poses_list_returns_catalog_in_command_result():
    sim = MockRobot()
    result = sim.command('poses.list', {})

    assert result['accepted']
    assert result['result']['poses'] == [{
        'pose_id': 'home',
        'name': 'home',
        'pose': {'position': [0.0, -0.4, 0.5], 'orientation': [0.0, 1.0, 0.0, 0.0]},
        'frame_id': 'base_link',
        'is_home': True,
    }]


def test_unknown_command_is_rejected():
    result = MockRobot().command('unknown.command', {})

    assert not result['accepted']
    assert result['error']['code'] == 'unknown_command'


def test_plan_then_execute_trajectory():
    sim = MockRobot()
    assert sim.command('arm.plan_to_pose', {
        'operation_id': 'op-1',
        'pose': {'position': [0.4, 0.0, 0.5], 'orientation': [0.0, 1.0, 0.0, 0.0]},
        'frame_id': 'base_link',
    })['accepted']

    sim.tick(0.1)
    kind, trajectory = sim.take_events()[0]
    assert kind == 'planned_trajectory'
    assert trajectory['trajectory_id'] == 'traj-op-1'
    assert sim.operation.status == 'awaiting_confirmation'

    assert sim.command('arm.execute_trajectory', {
        'operation_id': 'op-1',
        'trajectory_id': 'traj-op-1',
    })['accepted']
    assert sim.operation.status == 'executing'

    sim.tick(3.0)
    assert sim.operation is None
    assert sim.take_events() == [(
        'operation_status',
        {'operation_id': 'op-1', 'status': 'succeeded', 'progress': 1.0},
    )]


def test_cancelled_planning_emits_final_operation_status():
    sim = MockRobot()
    sim.command('arm.plan_to_pose', {'operation_id': 'op-1', 'pose': {}})
    sim.tick(0.1)
    sim.take_events()

    assert sim.command('operation.cancel', {'operation_id': 'op-1'})['accepted']
    assert sim.operation is None
    assert sim.take_events() == [(
        'operation_status',
        {'operation_id': 'op-1', 'status': 'cancelled', 'progress': 1.0},
    )]


def test_execute_and_cancel_require_matching_operation_id():
    sim = MockRobot()
    sim.command('arm.plan_to_pose', {'operation_id': 'op-1', 'pose': {}})
    sim.tick(0.1)
    sim.take_events()

    execute = sim.command('arm.execute_trajectory', {
        'operation_id': 'op-2',
        'trajectory_id': 'traj-op-1',
    })
    cancel = sim.command('operation.cancel', {'operation_id': 'op-2'})

    assert execute['error']['code'] == 'operation_mismatch'
    assert cancel['error']['code'] == 'operation_mismatch'


def test_joint_configuration_plan_interpolates_and_execution_moves_arm():
    sim = MockRobot()
    initial = list(sim.joints)
    target = [0.2, -1.2, 0.4, -1.4, 0.0, 0.2]
    assert sim.command('arm.plan_to_joint_configuration', {
        'operation_id': 'op-joints',
        'positions': target,
    })['accepted']

    sim.tick(0.1)
    _, planned = sim.take_events()[0]
    points = planned['trajectory']['points']
    assert len(points) == 21
    assert points[0]['positions'] == initial
    assert points[-1]['positions'] == target

    assert sim.command('arm.execute_trajectory', {
        'operation_id': 'op-joints',
        'trajectory_id': 'traj-op-joints',
    })['accepted']
    sim.tick(1.5)
    assert sim.joints != initial
    assert sim.joints != target

    sim.tick(1.5)
    assert sim.joints == target


def test_joint_configuration_requires_six_positions():
    result = MockRobot().command('arm.plan_to_joint_configuration', {
        'operation_id': 'op-joints',
        'positions': [0.0] * 5,
    })

    assert not result['accepted']
    assert result['error']['code'] == 'invalid_joint_configuration'


def test_speed_is_clamped_to_safe_percentage_range():
    sim = MockRobot()

    sim.command('teleoperation.set_speed', {'percentage': 150})
    assert sim.speed_percentage == 100.0
    sim.command('teleoperation.set_speed', {'percentage': 0})
    assert sim.speed_percentage == 1.0


def test_joint_jog_watchdog_stops_motion():
    sim = MockRobot(command_timeout_sec=0.5)
    sim.command('teleoperation.enable', {})
    sim.command('teleoperation.deadman', {'active': True})
    sim.command('arm.joint_jog', {'velocities': [1, 0, 0, 0, 0, 0]})
    sim.last_motion_command -= 0.6

    sim.tick(0.1)

    assert sim.velocities == [0.0] * 6


def test_cartesian_jog_sets_six_joint_velocity_slots():
    sim = MockRobot()
    sim.command('teleoperation.enable', {})
    sim.command('teleoperation.deadman', {'active': True})

    result = sim.command('arm.cartesian_jog', {
        'twist': {'linear': [0.1, 0.2, 0.3]},
    })

    assert result['accepted']
    assert sim.velocities == [0.2, 0.4, 0.6, 0.2, 0.4, 0.6]


def test_pose_save_and_home_set_are_visible_in_catalog():
    sim = MockRobot()
    sim.command('poses.save', {
        'pose_id': 'pick',
        'pose_name': 'Pick pose',
        'pose': {'position': [1, 2, 3]},
    })
    sim.command('home.set', {'pose': {'position': [0, 0, 1]}})

    poses = {pose['pose_id']: pose for pose in sim.command('poses.list', {})['result']['poses']}
    assert poses['pick']['name'] == 'Pick pose'
    assert poses['home']['is_home']


def test_cancel_without_active_operation_is_rejected():
    result = MockRobot().command('operation.cancel', {'operation_id': 'missing'})

    assert not result['accepted']
    assert result['error']['code'] == 'operation_not_found'


def test_new_plan_cancels_previous_operation():
    sim = MockRobot()
    sim.command('arm.plan_to_joint_configuration', {
        'operation_id': 'old-op',
        'positions': [0.1] * 6,
    })
    result = sim.command('arm.plan_to_joint_configuration', {
        'operation_id': 'new-op',
        'positions': [0.2] * 6,
    })

    assert result['accepted']
    assert sim.operation.operation_id == 'new-op'
    assert sim.take_events() == [(
        'operation_status',
        {'operation_id': 'old-op', 'status': 'cancelled', 'progress': 0.0},
    )]


def test_execute_pending_trajectory_starts_current_plan():
    sim = MockRobot()
    sim.command('arm.plan_to_joint_configuration', {
        'operation_id': 'pending-op',
        'positions': [0.2] * 6,
    })
    sim.tick(0.1)
    sim.take_events()

    result = sim.command('arm.execute_pending_trajectory', {})

    assert result['accepted']
    assert result['result'] == {
        'operation_id': 'pending-op',
        'trajectory_id': 'traj-pending-op',
    }
    assert sim.operation.status == 'executing'
