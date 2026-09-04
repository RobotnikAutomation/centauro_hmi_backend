import math
import time
from dataclasses import dataclass, field


JOINT_NAMES = [f"shoulder_{x}" for x in ("pan", "lift")] + [
    "elbow", "wrist_1", "wrist_2", "wrist_3"
]


def _mat_mul(left, right):
    return tuple(
        tuple(sum(left[row][index] * right[index][column] for index in range(3)) for column in range(3))
        for row in range(3)
    )


def _mat_vec(matrix, vector):
    return tuple(sum(matrix[row][index] * vector[index] for index in range(3)) for row in range(3))


def _rotation(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    if axis == 'x':
        return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))
    if axis == 'y':
        return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def tool_position(joints):
    """Compute tool position using the same joint chain as ``urdf/arm.urdf``."""
    pan, shoulder, elbow, wrist_1, wrist_2, wrist_3 = joints
    rotation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    position = (0.0, 0.0, 0.0)

    # Each pair is the joint origin and its axis. The origins and axes mirror
    # arm.urdf exactly; translations are expressed in the current link frame.
    chain = (
        ((0.0, 0.0, 0.08), 'z', pan),
        ((0.0, 0.0, 0.20), 'y', shoulder),
        ((0.0, 0.0, -0.42), 'y', elbow),
        ((0.0, 0.0, -0.40), 'y', wrist_1),
        ((0.0, 0.0, -0.14), 'z', wrist_2),
        ((0.0, 0.0, -0.12), 'y', wrist_3),
        ((0.0, 0.0, -0.12), None, 0.0),
    )
    for origin, axis, angle in chain:
        offset = _mat_vec(rotation, origin)
        position = tuple(position[index] + offset[index] for index in range(3))
        if axis is not None:
            rotation = _mat_mul(rotation, _rotation(axis, angle))
    return position


@dataclass
class Operation:
    operation_id: str
    kind: str
    status: str = "planned"
    progress: float = 0.0
    error: str = ""
    created_at: float = field(default_factory=time.time)
    target_pose: dict | None = None
    target_joint_positions: list[float] | None = None
    trajectory_id: str | None = None


class MockRobot:
    """Deterministic six-axis robot mock used until a real adapter is available."""

    def __init__(self, speed_percentage=25.0, command_timeout_sec=0.5):
        self.joints = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]
        self.velocities = [0.0] * 6
        self.speed_percentage = speed_percentage
        self.command_timeout_sec = command_timeout_sec
        self.enabled = False
        self.deadman = False
        self.freedrive = False
        self.mode = "joint"
        self.last_status_reason = "initialized"
        self.last_deadman = time.monotonic()
        self.last_motion_command = time.monotonic()
        self.last_freedrive_command = time.monotonic()
        self.operation = None
        self.planned_trajectories = {}
        self._events = []
        self.poses = {
            "home": {"name": "home", "frame_id": "base_link", "position": [0.0, -0.4, 0.5], "orientation": [0.0, 1.0, 0.0, 0.0], "is_home": True}
        }

    def tick(self, dt):
        now = time.monotonic()
        if self.deadman and now - self.last_deadman > self.command_timeout_sec:
            self.deadman = False
            self.last_status_reason = "deadman_timeout"
        if self.freedrive and now - self.last_freedrive_command > self.command_timeout_sec:
            self.freedrive = False
            self.last_status_reason = "freedrive_timeout"
        motion_expired = now - self.last_motion_command > self.command_timeout_sec
        if (not (self.enabled and self.deadman) or motion_expired or
                self.operation and self.operation.status != "executing"):
            self.velocities = [0.0] * 6
        for i in range(6):
            self.joints[i] = max(-math.pi, min(math.pi, self.joints[i] + self.velocities[i] * dt))
        if self.operation and self.operation.status == "planning":
            trajectory_id = f"traj-{self.operation.operation_id}"
            start = list(self.joints)
            target = self.operation.target_joint_positions or start
            points = []
            for index in range(21):
                ratio = index / 20.0
                points.append({"positions": [
                    initial + (final - initial) * ratio
                    for initial, final in zip(start, target)
                ]})
            trajectory = {
                "operation_id": self.operation.operation_id,
                "trajectory_id": trajectory_id,
                "trajectory": {
                    "joint_names": JOINT_NAMES,
                    "points": points,
                },
                "validation": {"valid": True},
                "final_pose": self.operation.target_pose or {
                    "frame_id": "base_link",
                    "position": list(tool_position(target)),
                    "orientation": [0.0, 1.0, 0.0, 0.0],
                },
            }
            self.planned_trajectories[trajectory_id] = trajectory
            self.operation.trajectory_id = trajectory_id
            self.operation.status = "awaiting_confirmation"
            self.operation.progress = 1.0
            self._events.append(("planned_trajectory", trajectory))
        elif self.operation and self.operation.status == "executing":
            self.operation.progress = min(1.0, self.operation.progress + dt / 3.0)
            trajectory = self.planned_trajectories[self.operation.trajectory_id]
            points = trajectory["trajectory"]["points"]
            position = self.operation.progress * (len(points) - 1)
            lower = min(int(position), len(points) - 1)
            upper = min(lower + 1, len(points) - 1)
            ratio = position - lower
            start = points[lower]["positions"]
            target = points[upper]["positions"]
            self.joints = [initial + (final - initial) * ratio for initial, final in zip(start, target)]
            self.velocities = [
                (final - initial) / 3.0
                for initial, final in zip(points[0]["positions"], points[-1]["positions"])
            ]
            if self.operation.progress >= 1.0:
                operation = self.operation
                self.operation = None
                self.planned_trajectories.pop(operation.trajectory_id, None)
                self.velocities = [0.0] * 6
                self._events.append(("operation_status", {
                    "operation_id": operation.operation_id,
                    "status": "succeeded",
                    "progress": 1.0,
                }))

    def command(self, name, payload):
        if name == "teleoperation.enable":
            self.enabled = True
            self.last_status_reason = "enabled"
        elif name == "teleoperation.disable":
            self.enabled = False; self.deadman = False; self.freedrive = False; self.velocities = [0.0] * 6
            self.last_status_reason = "disabled"
        elif name == "teleoperation.deadman":
            self.deadman = bool(payload.get("active", False))
            if self.deadman:
                self.last_deadman = time.monotonic()
                self.last_status_reason = "deadman_active"
            else:
                self.velocities = [0.0] * 6
                self.last_status_reason = "deadman_released"
        elif name == "teleoperation.set_mode":
            self.mode = payload.get("mode", self.mode)
            self.last_status_reason = "mode_changed"
        elif name == "teleoperation.set_speed":
            self.speed_percentage = max(1.0, min(100.0, float(payload.get("percentage", 25))))
            self.last_status_reason = "speed_changed"
        elif name == "teleoperation.freedrive":
            self.freedrive = bool(payload.get("active", False))
            if self.freedrive:
                self.last_freedrive_command = time.monotonic()
                self.last_status_reason = "freedrive_active"
            else:
                self.last_status_reason = "freedrive_released"
        elif name == "arm.joint_jog":
            if self.enabled and self.deadman:
                values = payload.get("velocities", [0.0] * 6)
                self.velocities = [float(v) * self.speed_percentage / 100.0 for v in values[:6]] + [0.0] * 6
                self.velocities = self.velocities[:6]
                self.last_motion_command = time.monotonic()
        elif name == "arm.cartesian_jog":
            if self.enabled and self.deadman:
                self.velocities = [float(payload.get("twist", {}).get("linear", [0, 0, 0])[i % 3]) * 2 for i in range(6)]
                self.last_motion_command = time.monotonic()
        elif name == "operation.cancel":
            if not self.operation:
                return self._rejected("operation_not_found", "no hay ninguna operación activa")
            if self.operation.operation_id != payload.get("operation_id"):
                return self._rejected("operation_mismatch", "la operación activa no coincide con operation_id")
            operation = self.operation
            self.operation = None
            self.planned_trajectories.pop(operation.trajectory_id, None)
            self._events.append(("operation_status", {
                "operation_id": operation.operation_id,
                "status": "cancelled",
                "progress": operation.progress,
            }))
        elif name == "arm.plan_to_pose":
            if self.operation:
                self._cancel_active_operation()
            operation_id = payload.get("operation_id", "op-1")
            final_pose = dict(payload.get("pose", {}))
            final_pose["frame_id"] = payload.get("frame_id", "base_link")
            self.operation = Operation(
                operation_id,
                "plan_to_pose",
                "planning",
                target_pose=final_pose,
            )
        elif name == "arm.plan_to_joint_configuration":
            if self.operation:
                self._cancel_active_operation()
            try:
                positions = [float(value) for value in payload["positions"]]
            except (KeyError, TypeError, ValueError):
                return self._rejected("invalid_joint_configuration", "positions debe contener seis valores numéricos")
            if len(positions) != len(JOINT_NAMES):
                return self._rejected("invalid_joint_configuration", "positions debe contener seis valores")
            operation_id = payload.get("operation_id", "op-1")
            self.operation = Operation(
                operation_id,
                "plan_to_joint_configuration",
                "planning",
                target_joint_positions=positions,
            )
        elif name == "arm.execute_trajectory":
            trajectory_id = payload.get("trajectory_id")
            trajectory = self.planned_trajectories.get(trajectory_id)
            if not trajectory:
                return self._rejected("trajectory_not_found", "la trayectoria no existe o ya no está disponible")
            if trajectory["operation_id"] != payload.get("operation_id"):
                return self._rejected("operation_mismatch", "la trayectoria no corresponde a la operación indicada")
            if not self.operation or self.operation.operation_id != payload.get("operation_id"):
                return self._rejected("operation_not_ready", "la operación no está esperando confirmación")
            self.operation.status = "executing"
            self.operation.progress = 0.0
            self.operation.trajectory_id = trajectory_id
        elif name == "arm.execute_pending_trajectory":
            if not self.operation or self.operation.status != "awaiting_confirmation":
                return self._rejected("operation_not_ready", "no hay ninguna trayectoria pendiente de confirmación")
            operation = self.operation
            operation.status = "executing"
            operation.progress = 0.0
            return {
                "accepted": True,
                "result": {
                    "operation_id": operation.operation_id,
                    "trajectory_id": operation.trajectory_id,
                },
            }
        elif name == "poses.list":
            return {"accepted": True, "result": self.list_poses()}
        elif name == "poses.save":
            pose = dict(payload.get("pose", {})); pose["name"] = payload.get("pose_name", payload.get("pose_id", "unnamed"))
            self.poses[payload.get("pose_id", pose["name"])] = pose
        elif name == "poses.execute":
            self.operation = Operation(payload.get("operation_id", "op-1"), "pose_execute", "executing")
        elif name == "home.set":
            pose = dict(payload.get("pose", {})); pose.update(name="home", is_home=True); self.poses["home"] = pose
        else:
            return self._rejected("unknown_command", f"comando no soportado: {name}")
        return {"accepted": True}

    @staticmethod
    def _rejected(code, message):
        return {"accepted": False, "error": {"code": code, "message": message}}

    def _cancel_active_operation(self):
        operation = self.operation
        self.operation = None
        if operation.trajectory_id:
            self.planned_trajectories.pop(operation.trajectory_id, None)
        self._events.append(("operation_status", {
            "operation_id": operation.operation_id,
            "status": "cancelled",
            "progress": operation.progress,
        }))

    def take_events(self):
        events = self._events
        self._events = []
        return events

    def list_poses(self):
        poses = []
        for pose_id, stored_pose in self.poses.items():
            pose = dict(stored_pose)
            name = pose.pop("name", pose_id)
            frame_id = pose.pop("frame_id", "base_link")
            is_home = pose.pop("is_home", pose_id == "home")
            poses.append({
                "pose_id": pose_id,
                "name": name,
                "pose": pose,
                "frame_id": frame_id,
                "is_home": is_home,
            })
        return {"poses": poses}

    def snapshot(self):
        return {"joint_names": JOINT_NAMES, "positions": self.joints, "velocities": self.velocities,
                "frame_id": "base_link", "tool_frame": "tool0", "teleoperation": {
                    "enabled": self.enabled, "active": self.enabled and self.deadman,
                    "deadman": self.deadman, "freedrive": self.freedrive, "mode": self.mode,
                    "speed_percentage": self.speed_percentage, "safety": "ok",
                    "reason": self.last_status_reason}}
