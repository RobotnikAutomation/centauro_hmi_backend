import pytest

from centauro_hmi_backend.robots.real_robot import (
    ROBOT_JOINT_NAMES,
    RobotnikTeleoperationRobot,
)
from centauro_hmi_backend.robot import create_robot
from centauro_hmi_backend.pad_config import load_pad_speed_limits
from std_msgs.msg import Int32


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeClient:
    def service_is_ready(self):
        return False


class FakeParameterValue:
    def __init__(self, value):
        self.double_value = value


class FakeParameterResponse:
    def __init__(self, value):
        self.values = [FakeParameterValue(value)]


class FakeFuture:
    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        callback(self)


class FakeParameterClient:
    def __init__(self, value=0.1, ready=True):
        self.value = value
        self.ready = ready
        self.requested_names = []

    def services_are_ready(self):
        return self.ready

    def get_parameters(self, names):
        self.requested_names = names
        return FakeFuture(FakeParameterResponse(self.value))


class FakeNode:
    def __init__(self):
        self.publishers = {}
        self.subscriptions = {}

    def create_publisher(self, message_type, topic, depth):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, message_type, topic, callback, depth):
        self.subscriptions[topic] = callback
        return callback

    def create_client(self, service_type, service):
        return FakeClient()


def make_robot(parameter_client=None):
    node = FakeNode()
    parameter_client = parameter_client or FakeParameterClient()
    robot = RobotnikTeleoperationRobot(
        node, 0.5, '/robot/joint_states', parameter_client=parameter_client
    )
    return robot, node


def test_pad_speed_limits_are_loaded_from_robot_bringup_configs():
    assert load_pad_speed_limits('ps4') == (1.0, 1.5)
    assert load_pad_speed_limits('ps5') == (1.0, 1.5)


def test_ros_teleoperation_mode_feedback_updates_websocket_snapshot():
    robot, node = make_robot()
    mode_message = Int32()
    mode_message.data = 1

    node.subscriptions['/robot/arm/servo/teleoperation_mode/state'](mode_message)

    assert robot.snapshot()['teleoperation']['mode'] == 'cartesian_base'


def test_factory_selects_real_adapter():
    node = FakeNode()
    parameter_client = FakeParameterClient()

    robot = create_robot('real', 25.0, 0.5, node=node, parameter_client=parameter_client)

    assert isinstance(robot, RobotnikTeleoperationRobot)
    assert robot.supports_robot_model is False
    assert parameter_client.requested_names == ['default_velocity_percentage']
    assert robot.snapshot()['teleoperation']['speed_percentage'] == 10.0


def test_parameter_query_waits_for_teleoperation_node_and_retries_on_tick():
    parameter_client = FakeParameterClient(value=0.35, ready=False)
    robot, _ = make_robot(parameter_client)

    assert robot._teleoperation_speed_loaded is False
    parameter_client.ready = True
    robot.tick(0.05)

    assert robot._teleoperation_speed_loaded is True
    assert robot.snapshot()['teleoperation']['speed_percentage'] == 35.0


def test_enable_is_rejected_until_teleoperation_parameter_is_read():
    parameter_client = FakeParameterClient(ready=False)
    robot, _ = make_robot(parameter_client)

    result = robot.command('teleoperation.enable', {})

    assert result['accepted'] is False
    assert result['error']['code'] == 'teleoperation_parameter_unavailable'


def test_deadman_and_mode_publish_to_teleoperation_topics():
    robot, node = make_robot()

    assert robot.command('teleoperation.deadman', {'active': True})['accepted']
    assert node.publishers['/robot/arm/servo/deadman'].messages[-1].data is True
    assert robot.command('teleoperation.set_mode', {'mode': 'joint'})['accepted']
    assert node.publishers['/robot/arm/servo/teleoperation_mode'].messages[-1].data == 2


def test_joint_jog_reorders_and_normalizes_hmi_axes():
    robot, node = make_robot()
    robot.enabled = True
    robot.deadman = True

    result = robot.command('arm.joint_jog', {'velocities': [0.3, -0.2, 0.0, 0.0, 0.0, 0.0]})

    assert result['accepted']
    command = node.publishers['/robot/arm/servo/joint_teleoperation'].messages[-1]
    assert command.joint_names == ROBOT_JOINT_NAMES
    assert command.velocities == [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]


def test_cartesian_jog_maps_frame_and_direction():
    robot, node = make_robot()
    robot.enabled = True
    robot.deadman = True

    result = robot.command('arm.cartesian_jog', {
        'frame_id': 'robot_arm_tool0',
        'twist': {'linear': [0.05, 0.0, -0.2], 'angular': [0.0, 0.1, 0.0]},
    })

    assert result['accepted']
    assert node.publishers['/robot/arm/servo/teleoperation_mode'].messages[-1].data == 0
    command = node.publishers['/robot/arm/servo/cartesian_teleoperation'].messages[-1]
    assert command.header.frame_id == 'robot_arm_tool0'
    assert [command.twist.linear.x, command.twist.linear.y, command.twist.linear.z,
            command.twist.angular.x, command.twist.angular.y, command.twist.angular.z] == [1.0, 0.0, -1.0, 0.0, 1.0, 0.0]


def test_base_joystick_scales_planar_axes_and_publishes_twist():
    robot, node = make_robot()

    result = robot.command('base.joystick', {
        'twist': {'linear': [0.5, -0.25, 0.0], 'angular': [0.0, 0.0, 0.4]},
    })
    robot.publish_base_velocity()

    assert result['accepted']
    command = node.publishers['/robot/pad_teleop/cmd_vel'].messages[-1]
    assert (command.linear.x, command.linear.y, command.linear.z) == (0.5, -0.25, 0.0)
    assert (command.angular.x, command.angular.y, command.angular.z) == pytest.approx((0.0, 0.0, 0.6))


def test_base_joystick_rejects_out_of_range_axes():
    robot, node = make_robot()

    result = robot.command('base.joystick', {
        'twist': {'linear': [1.1, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0]},
    })

    assert result['accepted'] is False
    assert result['error']['code'] == 'invalid_base_joystick'
    assert node.publishers['/robot/pad_teleop/cmd_vel'].messages == []


def test_base_joystick_watchdog_publishes_zero(monkeypatch):
    robot, node = make_robot()
    robot.command('base.joystick', {
        'twist': {'linear': [1.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0]},
    })
    monkeypatch.setattr('centauro_hmi_backend.robots.real_robot.time.monotonic', lambda: robot._last_base_command + 1.0)

    robot.publish_base_velocity()

    command = node.publishers['/robot/pad_teleop/cmd_vel'].messages[-1]
    assert (command.linear.x, command.linear.y, command.angular.z) == (0.0, 0.0, 0.0)


def test_joint_state_feedback_is_reordered_to_hmi_order():
    robot, node = make_robot()
    sample = type('JointStateSample', (), {})()
    sample.name = list(ROBOT_JOINT_NAMES)
    sample.position = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    sample.velocity = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    node.subscriptions['/robot/joint_states'](sample)

    snapshot = robot.snapshot()
    assert snapshot['feedback_available'] is True
    assert snapshot['positions'] == [3.0, 2.0, 1.0, 4.0, 5.0, 6.0]
    assert snapshot['velocities'] == [30.0, 20.0, 10.0, 40.0, 50.0, 60.0]


def test_motion_planning_is_explicitly_unsupported():
    robot, _ = make_robot()

    result = robot.command('arm.plan_to_pose', {'pose': {}})

    assert result['accepted'] is False
    assert result['error']['code'] == 'unsupported_command'


def test_deadman_timeout_publishes_release_and_zero_motion(monkeypatch):
    robot, node = make_robot()
    robot.enabled = True
    robot.deadman = True
    robot._motion_kind = 'joint'
    robot._motion_active = True
    robot.last_deadman = 0.0
    robot.last_motion_command = 0.0
    monkeypatch.setattr('centauro_hmi_backend.robots.real_robot.time.monotonic', lambda: 1.0)

    robot.tick(0.05)

    assert robot.deadman is False
    assert node.publishers['/robot/arm/servo/deadman'].messages[-1].data is False
    assert node.publishers['/robot/arm/servo/joint_teleoperation'].messages[-1].velocities == [0.0] * 6
