import threading

from rclpy.qos import DurabilityPolicy
from robotnik_battery_msgs.msg import BatteryStatus

from centauro_hmi_backend.real_status import RealStatusBridge


def test_latched_arm_status_topics_request_transient_local_qos():
    class FakeNode:
        def __init__(self):
            self.subscriptions = []

        def create_subscription(self, message_type, topic, callback, qos):
            subscription = (message_type, topic, callback, qos)
            self.subscriptions.append(subscription)
            return subscription

    node = FakeNode()
    status_topics = {
        parameter: f"/{name}"
        for name, _, parameter in RealStatusBridge.STATUS_TOPICS
    }

    RealStatusBridge(node, [], "/battery", status_topics)

    qos_by_topic = {topic: qos for _, topic, _, qos in node.subscriptions}
    for topic in (
        "/robot_mode",
        "/robot_program_running",
        "/safety_mode",
    ):
        assert qos_by_topic[topic].durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_parse_image_streams_supports_multiple_named_topics():
    assert RealStatusBridge.parse_image_streams([
        'front=/robot/front/image_raw',
        'wrist=/robot/wrist/image_raw',
    ]) == [
        ('front', '/robot/front/image_raw'),
        ('wrist', '/robot/wrist/image_raw'),
    ]


def test_parse_image_streams_rejects_missing_name_or_topic():
    for invalid_spec in ('/robot/front/image_raw', 'front=', '=topic'):
        try:
            RealStatusBridge.parse_image_streams([invalid_spec])
        except ValueError as exc:
            assert 'stream_name=/absolute/topic' in str(exc)
        else:
            raise AssertionError(f'invalid image stream accepted: {invalid_spec}')


def test_parse_image_streams_rejects_duplicate_names():
    try:
        RealStatusBridge.parse_image_streams([
            'camera=/robot/front/image_raw',
            'camera=/robot/wrist/image_raw',
        ])
    except ValueError as exc:
        assert 'duplicate image stream name' in str(exc)
    else:
        raise AssertionError('duplicate stream names must be rejected')


def test_image_payload_includes_raw_data_and_stream_metadata():
    bridge = RealStatusBridge.__new__(RealStatusBridge)
    bridge._lock = threading.Lock()
    bridge._images = {'front': None}
    bridge._image_sequences = {'front': 0}
    bridge._published_image_sequences = {'front': 0}
    image = type('ImageSample', (), {})()
    image.data = [0, 1, 255]
    image.encoding = 'rgb8'
    image.width = 1
    image.height = 1
    image.step = 3
    image.is_bigendian = 0
    image.header = type('Header', (), {})()
    image.header.stamp = type('Stamp', (), {'sec': 12, 'nanosec': 34})()

    bridge._image_callback('front', '/robot/front/image_raw', image)
    status, frames = bridge.snapshot()

    assert status == {}
    assert len(frames) == 1
    assert frames[0]['stream_id'] == 'front'
    assert frames[0]['topic'] == '/robot/front/image_raw'
    assert frames[0]['data'] == 'AAH/'
    assert frames[0]['image_encoding'] == 'rgb8'
    assert frames[0]['width'] == 1
    assert frames[0]['height'] == 1
    assert frames[0]['step'] == 3
    assert frames[0]['timestamp'] == {'sec': 12, 'nanosec': 34}
    assert frames[0]['sequence'] == 1
    assert bridge.snapshot()[1] == []


def test_battery_level_is_forwarded_as_battery_percentage():
    bridge = RealStatusBridge.__new__(RealStatusBridge)
    bridge._lock = threading.Lock()
    bridge._battery = None
    bridge._status = {}
    bridge._images = {}
    bridge._image_sequences = {}
    bridge._published_image_sequences = {}
    battery = BatteryStatus()
    battery.level = 73.5
    battery.voltage = 24.0
    battery.current = 1.2

    bridge._battery_callback(battery)
    robot_status, _ = bridge.snapshot()

    assert robot_status['battery_percentage'] == 73.5
    assert robot_status['battery']['voltage'] == 24.0
