import math
import threading
import time

from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Int32
from std_srvs.srv import Trigger


JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
ROBOT_JOINT_NAMES = [
    "robot_arm_elbow_joint",
    "robot_arm_shoulder_lift_joint",
    "robot_arm_shoulder_pan_joint",
    "robot_arm_wrist_1_joint",
    "robot_arm_wrist_2_joint",
    "robot_arm_wrist_3_joint",
]


class RobotnikTeleoperationRobot:
    """HMI adapter for the robotnik_servo teleoperation node's ROS interface."""

    supports_robot_model = False
    is_real = True

    def __init__(self, node, command_timeout_sec, joint_states_topic,
                 teleoperation_node_name="/robot/arm_teleoperation_node", parameter_client=None):
        self.node = node
        self.command_timeout_sec = command_timeout_sec
        self._teleoperation_speed_percentage = None
        self.speed_percentage = 0.0
        self._teleoperation_speed_loaded = False
        self._teleoperation_speed_request_pending = False
        self._speed_was_set_by_hmi = False
        self._teleoperation_parameter_client = parameter_client or AsyncParameterClient(
            node, teleoperation_node_name
        )
        self.enabled = False
        self.deadman = False
        self.freedrive = False
        self.mode = "joint"
        self.last_status_reason = "waiting_for_teleoperation_node"
        self.last_deadman = time.monotonic()
        self.last_freedrive_command = time.monotonic()
        self.last_motion_command = time.monotonic()
        self.operation = None
        self._lock = threading.RLock()
        self._positions = [0.0] * len(JOINT_NAMES)
        self._velocities = [0.0] * len(JOINT_NAMES)
        self._feedback_available = False
        self._motion_kind = None
        self._motion_active = False
        self._pending_speed_update = False

        self._deadman_pub = node.create_publisher(Bool, "/robot/arm/servo/deadman", 10)
        self._freedrive_pub = node.create_publisher(Bool, "/robot/arm/servo/freedrive", 10)
        self._mode_pub = node.create_publisher(Int32, "/robot/arm/servo/teleoperation_mode", 10)
        self._speed_pub = node.create_publisher(
            Float32, "/robot/arm/servo/change_max_velocity_percentage", 10
        )
        self._twist_pub = node.create_publisher(
            TwistStamped, "/robot/arm/servo/cartesian_teleoperation", 10
        )
        self._joint_pub = node.create_publisher(
            JointJog, "/robot/arm/servo/joint_teleoperation", 10
        )
        self._joint_states_sub = node.create_subscription(
            JointState, joint_states_topic, self._joint_state_callback, 10
        )
        self._enable_client = node.create_client(
            Trigger, "/robot/arm/servo/enable_arm_teleoperation"
        )
        self._disable_client = node.create_client(
            Trigger, "/robot/arm/servo/disable_arm_teleoperation"
        )
        self._request_teleoperation_speed_parameter()

    def _request_teleoperation_speed_parameter(self):
        if self._teleoperation_speed_loaded or self._teleoperation_speed_request_pending:
            return
        if not self._teleoperation_parameter_client.services_are_ready():
            return
        try:
            future = self._teleoperation_parameter_client.get_parameters(["default_velocity_percentage"])
        except Exception as exc:
            self.last_status_reason = f"teleoperation_parameter_error:{exc}"
            return
        self._teleoperation_speed_request_pending = True
        future.add_done_callback(self._teleoperation_speed_parameter_complete)

    def _teleoperation_speed_parameter_complete(self, future):
        with self._lock:
            self._teleoperation_speed_request_pending = False
            try:
                response = future.result()
                if len(response.values) != 1:
                    raise ValueError("expected one parameter result")
                fraction = float(response.values[0].double_value)
                if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
                    raise ValueError("default_velocity_percentage must be between 0.0 and 1.0")
            except Exception as exc:
                self.last_status_reason = f"teleoperation_parameter_error:{exc}"
                return

            self._teleoperation_speed_percentage = fraction * 100.0
            self._teleoperation_speed_loaded = True
            if not self._speed_was_set_by_hmi:
                self.speed_percentage = self._teleoperation_speed_percentage
            else:
                self._pending_speed_update = True
                self._publish_speed_update()
            self.last_status_reason = "teleoperation_speed_loaded"

    @staticmethod
    def _rejected(code, message):
        return {"accepted": False, "error": {"code": code, "message": message}}

    @staticmethod
    def _direction(value):
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("command values must be finite")
        return 1.0 if number > 0.0 else -1.0 if number < 0.0 else 0.0

    def _publish_bool(self, publisher, value):
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

    def _set_mode(self, mode):
        aliases = {
            "tcp": ("cartesian_tcp", 0),
            "cartesian_tcp": ("cartesian_tcp", 0),
            "cartesian": ("cartesian_base", 1),
            "base": ("cartesian_base", 1),
            "cartesian_base": ("cartesian_base", 1),
            "joint": ("joint", 2),
        }
        mapped = aliases.get(str(mode).lower())
        if mapped is None:
            return self._rejected("invalid_mode", f"unsupported teleoperation mode: {mode}")
        self.mode, value = mapped
        msg = Int32()
        msg.data = value
        self._mode_pub.publish(msg)
        return {"accepted": True}

    def _publish_speed_update(self):
        if not self._teleoperation_speed_loaded or not self.enabled or self.freedrive:
            self._pending_speed_update = True
            return
        delta = self.speed_percentage / 100.0 - self._teleoperation_speed_percentage / 100.0
        if delta:
            msg = Float32()
            msg.data = delta
            self._speed_pub.publish(msg)
        self._teleoperation_speed_percentage = self.speed_percentage
        self._pending_speed_update = False

    def _publish_zero_motion(self):
        if self._motion_kind == "joint":
            msg = JointJog()
            msg.joint_names = list(ROBOT_JOINT_NAMES)
            msg.velocities = [0.0] * len(ROBOT_JOINT_NAMES)
            self._joint_pub.publish(msg)
        elif self._motion_kind == "twist":
            self._twist_pub.publish(TwistStamped())
        self._motion_active = False
        self._velocities = [0.0] * len(JOINT_NAMES)

    def _request_teleoperation(self, enable):
        if enable and not self._teleoperation_speed_loaded:
            self._request_teleoperation_speed_parameter()
            return self._rejected(
                "teleoperation_parameter_unavailable",
                "teleoperation node default_velocity_percentage has not been read",
            )
        client = self._enable_client if enable else self._disable_client
        if not client.service_is_ready():
            return self._rejected("teleoperation_unavailable", "teleoperation service is not available")
        try:
            future = client.call_async(Trigger.Request())
        except (RuntimeError, TypeError) as exc:
            return self._rejected("teleoperation_request_failed", str(exc))
        future.add_done_callback(lambda completed: self._service_complete(completed, enable))
        self.last_status_reason = "enable_requested" if enable else "disable_requested"
        return {"accepted": True}

    def _service_complete(self, future, enable):
        with self._lock:
            try:
                response = future.result()
            except Exception as exc:
                self.last_status_reason = f"teleoperation_service_error:{exc}"
                return
            if not response.success:
                self.last_status_reason = response.message or "teleoperation_service_rejected"
                return
            self.enabled = enable
            if enable:
                self.last_status_reason = "enabled"
                if self._pending_speed_update:
                    self._publish_speed_update()
            else:
                self.deadman = False
                self.freedrive = False
                self._publish_bool(self._deadman_pub, False)
                self._publish_bool(self._freedrive_pub, False)
                self._publish_zero_motion()
                self.last_status_reason = "disabled"

    def _joint_state_callback(self, msg):
        with self._lock:
            positions = dict(zip(msg.name, msg.position))
            velocities = dict(zip(msg.name, msg.velocity))
            if not all(name in positions for name in ROBOT_JOINT_NAMES):
                return
            robot_positions = [float(positions[name]) for name in ROBOT_JOINT_NAMES]
            robot_velocities = [float(velocities.get(name, 0.0)) for name in ROBOT_JOINT_NAMES]
            reorder = (2, 1, 0, 3, 4, 5)
            self._positions = [robot_positions[index] for index in reorder]
            self._velocities = [robot_velocities[index] for index in reorder]
            self._feedback_available = True

    def _joint_jog(self, payload):
        values = payload.get("velocities")
        if not isinstance(values, (list, tuple)) or len(values) != len(JOINT_NAMES):
            return self._rejected("invalid_joint_jog", "velocities must contain six values")
        try:
            hmi_values = [self._direction(value) for value in values]
        except (TypeError, ValueError) as exc:
            return self._rejected("invalid_joint_jog", str(exc))
        if self.enabled and self.deadman and not self.freedrive:
            result = self._set_mode("joint")
            if not result["accepted"]:
                return result
            msg = JointJog()
            msg.joint_names = list(ROBOT_JOINT_NAMES)
            msg.velocities = [hmi_values[index] for index in (2, 1, 0, 3, 4, 5)]
            self._joint_pub.publish(msg)
            self._motion_kind = "joint"
            self._motion_active = True
            self.last_motion_command = time.monotonic()
        return {"accepted": True}

    def _cartesian_jog(self, payload):
        twist = payload.get("twist", {})
        if not isinstance(twist, dict):
            return self._rejected("invalid_cartesian_jog", "twist must be an object")
        linear, angular = twist.get("linear"), twist.get("angular")
        if not isinstance(linear, (list, tuple)) or len(linear) != 3:
            return self._rejected("invalid_cartesian_jog", "twist.linear must contain three values")
        if not isinstance(angular, (list, tuple)) or len(angular) != 3:
            return self._rejected("invalid_cartesian_jog", "twist.angular must contain three values")
        try:
            directions = [self._direction(value) for value in (*linear, *angular)]
        except (TypeError, ValueError) as exc:
            return self._rejected("invalid_cartesian_jog", str(exc))
        if self.enabled and self.deadman and not self.freedrive:
            frame_id = str(payload.get("frame_id", "robot_arm_base_link"))
            mode = "tcp" if frame_id in ("robot_arm_tool0", "tool0", "tcp") else "base"
            result = self._set_mode(mode)
            if not result["accepted"]:
                return result
            msg = TwistStamped()
            msg.header.frame_id = frame_id
            (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
             msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z) = directions
            self._twist_pub.publish(msg)
            self._motion_kind = "twist"
            self._motion_active = True
            self.last_motion_command = time.monotonic()
        return {"accepted": True}

    def command(self, name, payload):
        with self._lock:
            if name == "teleoperation.enable":
                return self._request_teleoperation(True)
            if name == "teleoperation.disable":
                self.deadman = False
                self.freedrive = False
                self._publish_bool(self._deadman_pub, False)
                self._publish_bool(self._freedrive_pub, False)
                self._publish_zero_motion()
                return self._request_teleoperation(False)
            if name == "teleoperation.deadman":
                self.deadman = bool(payload.get("active", False))
                self._publish_bool(self._deadman_pub, self.deadman)
                self.last_deadman = time.monotonic()
                if not self.deadman:
                    self._publish_zero_motion()
                    self.last_status_reason = "deadman_released"
                else:
                    self.last_status_reason = "deadman_active"
                return {"accepted": True}
            if name == "teleoperation.freedrive":
                self.freedrive = bool(payload.get("active", False))
                self._publish_bool(self._freedrive_pub, self.freedrive)
                self.last_freedrive_command = time.monotonic()
                if self.freedrive:
                    self._publish_zero_motion()
                self.last_status_reason = "freedrive_active" if self.freedrive else "freedrive_released"
                return {"accepted": True}
            if name == "teleoperation.set_mode":
                return self._set_mode(payload.get("mode", self.mode))
            if name == "teleoperation.set_speed":
                try:
                    percentage = float(payload.get("percentage", self.speed_percentage))
                except (TypeError, ValueError):
                    return self._rejected("invalid_speed", "percentage must be numeric")
                if not math.isfinite(percentage) or not 0.0 <= percentage <= 100.0:
                    return self._rejected("invalid_speed", "percentage must be between 0 and 100")
                self.speed_percentage = percentage
                self._speed_was_set_by_hmi = True
                self._pending_speed_update = True
                self._publish_speed_update()
                self.last_status_reason = "speed_changed"
                return {"accepted": True}
            if name == "arm.joint_jog":
                return self._joint_jog(payload)
            if name == "arm.cartesian_jog":
                return self._cartesian_jog(payload)
            return self._rejected("unsupported_command", f"{name} is not supported by the teleoperation node")

    def tick(self, dt):
        del dt
        self._request_teleoperation_speed_parameter()
        now = time.monotonic()
        with self._lock:
            if self.deadman and now - self.last_deadman > self.command_timeout_sec:
                self.deadman = False
                self._publish_bool(self._deadman_pub, False)
                self._publish_zero_motion()
                self.last_status_reason = "deadman_timeout"
            if self.freedrive and now - self.last_freedrive_command > self.command_timeout_sec:
                self.freedrive = False
                self._publish_bool(self._freedrive_pub, False)
                self.last_status_reason = "freedrive_timeout"
            if self._motion_active and now - self.last_motion_command > self.command_timeout_sec:
                self._publish_zero_motion()

    def take_events(self):
        return []

    def snapshot(self):
        with self._lock:
            return {
                "joint_names": list(JOINT_NAMES),
                "positions": list(self._positions),
                "velocities": list(self._velocities),
                "feedback_available": self._feedback_available,
                "frame_id": "robot_arm_base_link",
                "tool_frame": "robot_arm_tool0",
                "teleoperation": {
                    "enabled": self.enabled,
                    "active": self.enabled and self.deadman and not self.freedrive,
                    "deadman": self.deadman,
                    "freedrive": self.freedrive,
                    "mode": self.mode,
                    "speed_percentage": self.speed_percentage,
                    "safety": "unknown",
                    "reason": self.last_status_reason,
                },
            }