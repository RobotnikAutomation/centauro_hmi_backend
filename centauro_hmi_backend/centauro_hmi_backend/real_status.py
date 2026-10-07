import base64
import threading

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from robotnik_battery_msgs.msg import BatteryStatus
from robotnik_hardware_msgs.msg import MotorStatusArray
from robotnik_safety_msgs.msg import SafetyModeStatus
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_msgs.msg import ToolDataMsg


class RealStatusBridge:
    """Caches real robot status and camera frames for WebSocket publication."""

    STATUS_TOPICS = (
        ("base_status", MotorStatusArray, "robot.base_status_topic"),
        ("robot_mode", RobotMode, "robot.robot_mode_topic"),
        ("robot_program_running", Bool, "robot.robot_program_running_topic"),
        ("safety_mode", SafetyMode, "robot.safety_mode_topic"),
        ("tool_data", ToolDataMsg, "robot.tool_data_topic"),
        ("safety_module_status", SafetyModeStatus, "robot.safety_module_status_topic"),
    )
    LATCHED_STATUS_KEYS = {"robot_mode", "robot_program_running", "safety_mode"}

    def __init__(self, node, image_streams, battery_topic, status_topics):
        self._lock = threading.Lock()
        self._battery = None
        self._status = {}
        self._images = {}
        self._image_sequences = {}
        self._published_image_sequences = {}
        self._image_subscriptions = []
        self._status_subscriptions = []

        for stream_id, topic in self.parse_image_streams(image_streams):
            self._images[stream_id] = None
            self._image_sequences[stream_id] = 0
            self._published_image_sequences[stream_id] = 0
            self._image_subscriptions.append(
                node.create_subscription(
                    Image,
                    topic,
                    lambda msg, key=stream_id, image_topic=topic: self._image_callback(
                        key, image_topic, msg
                    ),
                    10,
                )
            )

        self._battery_subscription = node.create_subscription(
            BatteryStatus, battery_topic, self._battery_callback, 10
        )
        for key, message_type, parameter_name in self.STATUS_TOPICS:
            topic = status_topics[parameter_name]
            qos = 10
            if key in self.LATCHED_STATUS_KEYS:
                qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
            self._status_subscriptions.append(
                node.create_subscription(
                    message_type,
                    topic,
                    lambda msg, status_key=key: self._status_callback(status_key, msg),
                    qos,
                )
            )

    @staticmethod
    def parse_image_streams(stream_specs):
        streams = []
        seen = set()
        for spec in stream_specs:
            stream_id, separator, topic = str(spec).partition("=")
            if not separator or not stream_id.strip() or not topic.strip():
                raise ValueError("image stream entries must use 'stream_name=/absolute/topic' format")
            stream_id, topic = stream_id.strip(), topic.strip()
            if stream_id in seen:
                raise ValueError(f"duplicate image stream name: {stream_id}")
            seen.add(stream_id)
            streams.append((stream_id, topic))
        return streams

    def _image_callback(self, stream_id, topic, msg):
        payload = {
            "stream_id": stream_id,
            "topic": topic,
            "encoding": "base64",
            "format": "ros_image",
            "data": base64.b64encode(bytes(msg.data)).decode("ascii"),
            "image_encoding": msg.encoding,
            "width": msg.width,
            "height": msg.height,
            "step": msg.step,
            "is_bigendian": bool(msg.is_bigendian),
            "timestamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec,
            },
        }
        with self._lock:
            self._images[stream_id] = payload
            self._image_sequences[stream_id] += 1

    def _battery_callback(self, msg):
        battery = message_to_ordereddict(msg)
        with self._lock:
            self._battery = dict(battery)

    def _status_callback(self, key, msg):
        status = message_to_ordereddict(msg)
        with self._lock:
            self._status[key] = dict(status)

    def snapshot(self):
        with self._lock:
            battery = dict(self._battery) if self._battery is not None else None
            status = {key: dict(value) for key, value in self._status.items()}
            images = []
            for stream_id, image in self._images.items():
                sequence = self._image_sequences[stream_id]
                if image is not None and sequence > self._published_image_sequences[stream_id]:
                    frame = dict(image)
                    frame["sequence"] = sequence
                    images.append(frame)
                    self._published_image_sequences[stream_id] = sequence

        robot_status = dict(status)
        if battery is not None:
            robot_status["battery"] = battery
            robot_status["battery_percentage"] = battery.get("level")
        return robot_status, images