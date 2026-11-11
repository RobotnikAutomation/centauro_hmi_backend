import math
import threading
import time
import uuid

from manipulation_msgs.srv import SavePose


class PoseSavingBridge:
    def __init__(self, node, publish_event, service_name, timeout_sec):
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('pose_saving.timeout_sec must be positive and finite')
        self._publish_event = publish_event
        self._client = node.create_client(SavePose, service_name)
        self._timeout_sec = timeout_sec
        self._lock = threading.Lock()
        self._pending = {}
        self._timer = node.create_timer(0.1, self._expire)

    @staticmethod
    def _rejected(code, message):
        return {'accepted': False, 'error': {'code': code, 'message': message}}

    def command(self, payload, request_id=None):
        pose_name = payload.get('pose_name')
        if not isinstance(pose_name, str) or not pose_name.strip():
            return self._rejected('invalid_pose_name', 'pose_name must be a non-empty string')
        if not self._client.service_is_ready():
            return self._rejected('pose_saving_unavailable', 'Pose manager service is not available')
        save_id = str(uuid.uuid4())
        request = SavePose.Request()
        request.name = pose_name
        with self._lock:
            if self._pending:
                return self._rejected('pose_saving_busy', 'A pose save is already in progress')
            try:
                future = self._client.call_async(request)
            except RuntimeError as exc:
                return self._rejected('pose_saving_unavailable', str(exc))
            self._pending[save_id] = (
                future, time.monotonic() + self._timeout_sec, pose_name, request_id,
            )
        future.add_done_callback(lambda done: self._complete(save_id, done))
        return {'accepted': True, 'result': {'save_id': save_id}}

    def _emit(self, save_id, pending, success, error=None):
        payload = {
            'save_id': save_id, 'request_id': pending[3],
            'pose_name': pending[2], 'success': success,
        }
        if error is not None:
            payload['error'] = error
        self._publish_event('pose_saved', payload)

    def _complete(self, save_id, future):
        with self._lock:
            pending = self._pending.pop(save_id, None)
        if pending is None:
            return
        try:
            response = future.result()
            success = response is not None and response.success == 1
        except Exception as exc:
            self._emit(save_id, pending, False, {
                'code': 'pose_saving_failed', 'message': str(exc),
            })
            return
        self._emit(save_id, pending, success, None if success else {
            'code': 'pose_saving_failed', 'message': 'Pose manager could not save the pose',
        })

    def _expire(self):
        now = time.monotonic()
        with self._lock:
            expired = [(save_id, pending) for save_id, pending in self._pending.items()
                       if now >= pending[1]]
            for save_id, _ in expired:
                del self._pending[save_id]
        for save_id, pending in expired:
            self._client.remove_pending_request(pending[0])
            pending[0].cancel()
            self._emit(save_id, pending, False, {
                'code': 'pose_saving_timeout',
                'message': 'No response from pose manager; the pose may still have been saved',
            })