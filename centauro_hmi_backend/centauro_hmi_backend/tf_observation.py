from dataclasses import dataclass
import math
import threading
import uuid

from action_msgs.msg import GoalStatus
from hmi_tfs_manager.action import ObserveTransform
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle


@dataclass
class Observation:
    reference_frame: str
    observed_frame: str
    request_id: object
    owner: object
    handle: ClientGoalHandle | None = None
    cancel_requested: bool = False


def transform_payload(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return {
        'stamp': {'sec': transform.header.stamp.sec, 'nanosec': transform.header.stamp.nanosec},
        'position': [translation.x, translation.y, translation.z],
        'orientation': [rotation.x, rotation.y, rotation.z, rotation.w],
    }


class TfObservationBridge:
    """Read-only action client; correlates feedback/results with each frame-pair request."""

    def __init__(self, node, publish_event, action_name, action_client=None):
        self._publish_event = publish_event
        self._client = action_client if action_client is not None else ActionClient(
            node, ObserveTransform, action_name)
        self._lock = threading.Lock()
        self._observations = {}

    @staticmethod
    def _rejected(code, message):
        return {'accepted': False, 'error': {'code': code, 'message': message}}

    def command(self, payload, request_id=None, owner=None):
        frames = [payload.get('reference_frame'), payload.get('observed_frame')]
        if any(not isinstance(frame, str) or not frame or frame.startswith('/') or
               any(char.isspace() for char in frame) for frame in frames):
            return self._rejected('invalid_tf_request', 'Frame names must be non-empty strings '
                                  'without whitespace or a leading slash')
        continuous = payload.get('continuous', False)
        timeout = payload.get('timeout_sec', 5.0)
        if not isinstance(continuous, bool):
            return self._rejected('invalid_tf_request', 'continuous must be a boolean')
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
            return self._rejected(
                'invalid_tf_request', 'timeout_sec must be a positive finite number')
        try:
            timeout = float(timeout)
        except OverflowError:
            return self._rejected('invalid_tf_request', 'timeout_sec is too large')
        if not math.isfinite(timeout) or timeout <= 0:
            return self._rejected(
                'invalid_tf_request', 'timeout_sec must be a positive finite number')
        if not self._client.server_is_ready():
            return self._rejected(
                'tf_unavailable', 'TF observation action server is not available')

        observation_id = str(uuid.uuid4())
        observation = Observation(frames[0], frames[1], request_id, owner)
        with self._lock:
            self._observations[observation_id] = observation
        goal = ObserveTransform.Goal()
        goal.reference_frame, goal.observed_frame = frames
        goal.continuous = continuous
        goal.timeout_sec = timeout
        try:
            future = self._client.send_goal_async(
                goal, feedback_callback=lambda msg: self._feedback(observation_id, msg))
        except RuntimeError as exc:
            with self._lock:
                self._observations.pop(observation_id, None)
            return self._rejected('tf_unavailable', str(exc))
        future.add_done_callback(lambda done: self._goal_response(observation_id, done))
        return {'accepted': True, 'result': {'observation_id': observation_id}}

    def cancel(self, payload, owner=None):
        observation_id = payload.get('observation_id')
        if not isinstance(observation_id, str) or not observation_id:
            return self._rejected(
                'invalid_tf_request', 'observation_id must be a non-empty string')
        with self._lock:
            observation = self._observations.get(observation_id)
            if observation is None or observation.owner is not owner:
                return self._rejected(
                    'unknown_tf_observation', 'No active observation for this client')
            observation.cancel_requested = True
            handle = observation.handle
        if handle is not None:
            self._cancel_goal(observation_id, handle)
        return {'accepted': True, 'result': {'observation_id': observation_id}}

    def cancel_owner(self, owner):
        with self._lock:
            ids = [key for key, value in self._observations.items() if value.owner is owner]
        for observation_id in ids:
            self.cancel({'observation_id': observation_id}, owner)

    def _emit(self, observation_id, observation, payload):
        self._publish_event('tf_transform', {
            'observation_id': observation_id,
            'request_id': observation.request_id,
            'reference_frame': observation.reference_frame,
            'observed_frame': observation.observed_frame,
            **payload,
        })

    def _fail(self, observation_id, message):
        with self._lock:
            observation = self._observations.pop(observation_id, None)
        if observation is not None:
            self._emit(observation_id, observation, {
                'final': True, 'success': False, 'status': 'failed', 'message': message,
            })

    def _cancel_goal(self, observation_id, handle):
        try:
            future = handle.cancel_goal_async()
        except RuntimeError as exc:
            self._cancel_error(observation_id, str(exc))
            return

        def completed(done):
            try:
                response = done.result()
                if not response.goals_canceling:
                    self._cancel_error(observation_id, 'Action server did not accept cancellation')
            except RuntimeError as exc:
                self._cancel_error(observation_id, str(exc))
        future.add_done_callback(completed)

    def _cancel_error(self, observation_id, message):
        with self._lock:
            observation = self._observations.get(observation_id)
        if observation is not None:
            self._emit(observation_id, observation, {
                'final': False, 'success': False, 'status': 'cancel_failed', 'message': message,
            })

    def _goal_response(self, observation_id, future):
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._fail(observation_id, 'TF observation goal rejected')
                return
            with self._lock:
                observation = self._observations[observation_id]
                observation.handle = handle
                cancel_requested = observation.cancel_requested
            handle.get_result_async().add_done_callback(
                lambda done: self._result(observation_id, done))
            if cancel_requested:
                self._cancel_goal(observation_id, handle)
        except RuntimeError as exc:
            self._fail(observation_id, str(exc))

    def _feedback(self, observation_id, msg):
        with self._lock:
            observation = self._observations.get(observation_id)
        if observation is not None:
            self._emit(observation_id, observation, {
                'final': False, 'success': True, **transform_payload(msg.feedback.transform),
            })

    def _result(self, observation_id, future):
        try:
            response = future.result()
            result = response.result
            payload = {
                'final': True,
                'success': result.success and response.status == GoalStatus.STATUS_SUCCEEDED,
                'status': {
                    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                    GoalStatus.STATUS_CANCELED: 'canceled',
                    GoalStatus.STATUS_ABORTED: 'aborted',
                }.get(response.status, 'failed'),
                'message': result.message,
            }
            if result.transform.header.frame_id:
                payload.update(transform_payload(result.transform))
        except RuntimeError as exc:
            self._fail(observation_id, str(exc))
            return
        with self._lock:
            observation = self._observations.pop(observation_id, None)
        if observation is not None:
            self._emit(observation_id, observation, payload)
