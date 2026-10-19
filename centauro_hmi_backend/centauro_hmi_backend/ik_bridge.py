import math
import threading
import time

from cartesian_kinematics_for_hmi_teleoperation.action import FollowTcpTarget
from geometry_msgs.msg import TransformStamped


def parse_target_pose(payload, default_frame_id):
    """Validates an HMI pose payload; returns (frame_id, position, unit quaternion xyzw)."""
    pose = payload.get('pose')
    if not isinstance(pose, dict):
        raise ValueError('pose must be an object')
    position, orientation = pose.get('position'), pose.get('orientation')
    if not isinstance(position, (list, tuple)) or len(position) != 3:
        raise ValueError('pose.position must contain three values [x, y, z]')
    if not isinstance(orientation, (list, tuple)) or len(orientation) != 4:
        raise ValueError('pose.orientation must contain four values [x, y, z, w]')
    position = [float(value) for value in position]
    orientation = [float(value) for value in orientation]
    if not all(math.isfinite(value) for value in (*position, *orientation)):
        raise ValueError('pose values must be finite')
    norm = math.sqrt(sum(value * value for value in orientation))
    if norm < 1e-9:
        raise ValueError('pose.orientation must be a non-zero quaternion')
    frame_id = str(payload.get('frame_id') or default_frame_id)
    return frame_id, position, [value / norm for value in orientation]


class CartesianIkBridge:
    """Publishes HMI Cartesian targets as TF and streams IK solutions from the action server."""

    def __init__(self, node, publish_event, target_frame, default_frame_id, action_name,
                 timeout_sec, action_client=None, tf_broadcaster=None):
        self.node = node
        self.publish_event = publish_event
        self.target_frame = target_frame
        self.default_frame_id = default_frame_id
        self.timeout_sec = timeout_sec
        if action_client is None:
            from rclpy.action import ActionClient
            action_client = ActionClient(node, FollowTcpTarget, action_name)
        if tf_broadcaster is None:
            from tf2_ros import TransformBroadcaster
            tf_broadcaster = TransformBroadcaster(node)
        self._client = action_client
        self._tf_broadcaster = tf_broadcaster
        self._lock = threading.Lock()
        self._goal_handle = None
        self._goal_pending = False
        self._last_target = 0.0

    @staticmethod
    def _rejected(code, message):
        return {'accepted': False, 'error': {'code': code, 'message': message}}

    def command(self, payload):
        try:
            frame_id, position, orientation = parse_target_pose(payload, self.default_frame_id)
        except (TypeError, ValueError) as exc:
            return self._rejected('invalid_cartesian_target', str(exc))

        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = frame_id
        transform.child_frame_id = self.target_frame
        (transform.transform.translation.x,
         transform.transform.translation.y,
         transform.transform.translation.z) = position
        (transform.transform.rotation.x,
         transform.transform.rotation.y,
         transform.transform.rotation.z,
         transform.transform.rotation.w) = orientation
        self._tf_broadcaster.sendTransform(transform)

        with self._lock:
            self._last_target = time.monotonic()
            if self._goal_handle is not None or self._goal_pending:
                return {'accepted': True}
            if not self._client.server_is_ready():
                return self._rejected('ik_unavailable', 'IK action server is not available')
            self._goal_pending = True

        goal = FollowTcpTarget.Goal()
        goal.target_frame = self.target_frame
        goal.continuous = True
        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response_callback)
        return {'accepted': True}

    def tick(self):
        with self._lock:
            goal_handle = self._goal_handle
            expired = time.monotonic() - self._last_target > self.timeout_sec
            if goal_handle is None or not expired:
                return
            self._goal_handle = None
        goal_handle.cancel_goal_async()

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            goal_handle, error = None, str(exc)
        else:
            error = 'IK goal rejected'
        with self._lock:
            self._goal_pending = False
            if goal_handle is None or not goal_handle.accepted:
                self.publish_event('ik_solution', {'final': True, 'success': False, 'message': error})
                return
            self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda result_future: self._result_callback(goal_handle, result_future)
        )

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.publish_event('ik_solution', {
            'final': False,
            'joint_names': list(feedback.joint_names),
            'positions': list(feedback.positions),
            'position_error': feedback.position_error,
            'orientation_error': feedback.orientation_error,
            'joint_delta_norm': feedback.joint_delta_norm,
        })

    def _result_callback(self, goal_handle, future):
        with self._lock:
            if self._goal_handle is goal_handle:
                self._goal_handle = None
        result = future.result().result
        self.publish_event('ik_solution', {
            'final': True,
            'success': result.success,
            'message': result.message,
            'joint_names': list(result.joint_names),
            'positions': list(result.positions),
        })
