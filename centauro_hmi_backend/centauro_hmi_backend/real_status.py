from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from robotnik_battery_msgs.msg import BatteryStatus
from robotnik_hardware_msgs.msg import MotorStatusArray
from robotnik_safety_msgs.msg import SafetyModeStatus
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import Bool
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_msgs.msg import ToolDataMsg

from .camera import ImageStreamBridge


class RealStatusBridge(ImageStreamBridge):
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
        super().__init__(node, image_streams)
        self._battery = None
        self._status = {}
        self._status_subscriptions = []

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
            images = self._take_images_locked()

        robot_status = dict(status)
        if battery is not None:
            robot_status["battery"] = battery
            robot_status["battery_percentage"] = battery.get("level")
        return robot_status, images