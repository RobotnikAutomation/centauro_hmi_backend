import math
import threading
import time
import uuid

from manipulation_msgs.srv import GetListOfPoses, GetPose, RemovePose, SavePose


class PoseSavingBridge:
    def __init__(self, node, publish_event, service_name, timeout_sec,
                 list_service_name='/robot/pose_saver/list_poses_names',
                 get_service_name='/robot/pose_saver/get_pose',
                 remove_service_name='/robot/pose_saver/remove_pose'):
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('pose_saving.timeout_sec must be positive and finite')
        self._publish_event = publish_event
        self._client = node.create_client(SavePose, service_name)
        self._list_client = node.create_client(GetListOfPoses, list_service_name)
        self._get_client = node.create_client(GetPose, get_service_name)
        self._remove_client = node.create_client(RemovePose, remove_service_name)
        self._timeout_sec = timeout_sec
        self._lock = threading.Lock()
        self._pending = {}
        self._pending_lists = {}
        self._pending_gets = {}
        self._pending_removals = {}
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

    def list_poses(self, request_id=None):
        if not self._list_client.service_is_ready():
            return self._rejected('pose_listing_unavailable', 'Pose list service is not available')
        list_id = str(uuid.uuid4())
        with self._lock:
            if self._pending_lists:
                return self._rejected('pose_listing_busy', 'A pose list request is already in progress')
            try:
                future = self._list_client.call_async(GetListOfPoses.Request())
            except RuntimeError as exc:
                return self._rejected('pose_listing_unavailable', str(exc))
            self._pending_lists[list_id] = (
                future, time.monotonic() + self._timeout_sec, request_id,
            )
        future.add_done_callback(lambda done: self._list_complete(list_id, done))
        return {'accepted': True, 'result': {'list_id': list_id}}

    def get_pose(self, payload, request_id=None):
        pose_name = payload.get('pose_name')
        if not isinstance(pose_name, str) or not pose_name.strip():
            return self._rejected(
                'invalid_pose_name', 'pose_name must be a non-empty string',
            )
        if not self._get_client.service_is_ready():
            return self._rejected(
                'pose_retrieval_unavailable', 'Pose retrieval service is not available',
            )
        get_id = str(uuid.uuid4())
        request = GetPose.Request()
        request.name = pose_name
        with self._lock:
            if self._pending_gets:
                return self._rejected(
                    'pose_retrieval_busy', 'A pose retrieval is already in progress',
                )
            try:
                future = self._get_client.call_async(request)
            except RuntimeError as exc:
                return self._rejected(
                    'pose_retrieval_unavailable', str(exc),
                )
            self._pending_gets[get_id] = (
                future, time.monotonic() + self._timeout_sec, pose_name, request_id,
            )
        future.add_done_callback(
            lambda done: self._get_complete(get_id, done),
        )
        return {'accepted': True, 'result': {'get_id': get_id}}

    def remove_pose(self, payload, request_id=None):
        pose_name = payload.get('pose_name')
        if not isinstance(pose_name, str) or not pose_name.strip():
            return self._rejected(
                'invalid_pose_name', 'pose_name must be a non-empty string',
            )
        if not self._remove_client.service_is_ready():
            return self._rejected(
                'pose_removal_unavailable', 'Pose removal service is not available',
            )
        remove_id = str(uuid.uuid4())
        request = RemovePose.Request()
        request.name = pose_name
        with self._lock:
            if self._pending_removals:
                return self._rejected(
                    'pose_removal_busy', 'A pose removal is already in progress',
                )
            try:
                future = self._remove_client.call_async(request)
            except RuntimeError as exc:
                return self._rejected(
                    'pose_removal_unavailable', str(exc),
                )
            self._pending_removals[remove_id] = (
                future, time.monotonic() + self._timeout_sec, pose_name, request_id,
            )
        future.add_done_callback(
            lambda done: self._remove_complete(remove_id, done),
        )
        return {'accepted': True, 'result': {'remove_id': remove_id}}

    @staticmethod
    def _pose_data(response):
        transform = response.tf_pose
        translation = transform.translation
        rotation = transform.rotation
        rpy = response.rpy_pose
        joint_state = response.joint_pose
        stamp = joint_state.header.stamp
        return {
            'tf_pose': {
                'translation': {
                    'x': float(translation.x),
                    'y': float(translation.y),
                    'z': float(translation.z),
                },
                'rotation': {
                    'x': float(rotation.x), 'y': float(rotation.y),
                    'z': float(rotation.z), 'w': float(rotation.w),
                },
            },
            'rpy_pose': {
                'x': float(rpy.x), 'y': float(rpy.y), 'z': float(rpy.z),
                'roll': float(rpy.roll), 'pitch': float(rpy.pitch),
                'yaw': float(rpy.yaw),
            },
            'joint_pose': {
                'header': {
                    'stamp': {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)},
                    'frame_id': joint_state.header.frame_id,
                },
                'name': list(joint_state.name),
                'position': [float(value) for value in joint_state.position],
                'velocity': [float(value) for value in joint_state.velocity],
                'effort': [float(value) for value in joint_state.effort],
            },
        }

    def _emit_list(self, list_id, pending, success, names, message, error=None):
        payload = {
            'list_id': list_id, 'request_id': pending[2], 'success': success,
            'names': names, 'message': message,
        }
        if error is not None:
            payload['error'] = error
        self._publish_event('poses_list', payload)

    def _list_complete(self, list_id, future):
        with self._lock:
            pending = self._pending_lists.pop(list_id, None)
        if pending is None:
            return
        try:
            response = future.result()
            success = response is not None and bool(response.success)
            names = list(response.names) if success else []
            message = response.message if response is not None else 'No pose list response'
        except Exception as exc:
            self._emit_list(list_id, pending, False, [], str(exc), {
                'code': 'pose_listing_failed', 'message': str(exc),
            })
            return
        self._emit_list(list_id, pending, success, names, message, None if success else {
            'code': 'pose_listing_failed', 'message': message,
        })

    def _emit_pose_retrieved(
        self, get_id, pending, success, message, pose=None, error=None,
    ):
        payload = {
            'get_id': get_id, 'request_id': pending[3], 'pose_name': pending[2],
            'success': success, 'message': message,
        }
        if pose is not None:
            payload['pose'] = pose
        if error is not None:
            payload['error'] = error
        self._publish_event('pose_retrieved', payload)

    def _get_complete(self, get_id, future):
        with self._lock:
            pending = self._pending_gets.pop(get_id, None)
        if pending is None:
            return
        try:
            response = future.result()
            success = response is not None and bool(response.success)
            message = response.message if response is not None else 'No pose response'
            pose = self._pose_data(response) if success else None
        except Exception as exc:
            self._emit_pose_retrieved(
                get_id, pending, False, str(exc), error={
                    'code': 'pose_retrieval_failed', 'message': str(exc),
                },
            )
            return
        self._emit_pose_retrieved(
            get_id, pending, success, message, pose, None if success else {
                'code': 'pose_retrieval_failed', 'message': message,
            },
        )

    def _emit_pose_removed(
        self, remove_id, pending, success, message, error=None,
    ):
        payload = {
            'remove_id': remove_id, 'request_id': pending[3], 'pose_name': pending[2],
            'success': success, 'message': message,
        }
        if error is not None:
            payload['error'] = error
        self._publish_event('pose_removed', payload)

    def _remove_complete(self, remove_id, future):
        with self._lock:
            pending = self._pending_removals.pop(remove_id, None)
        if pending is None:
            return
        try:
            response = future.result()
            success = response is not None and bool(response.success)
            message = (
                response.message if response is not None else 'No pose removal response'
            )
        except Exception as exc:
            self._emit_pose_removed(
                remove_id, pending, False, str(exc), {
                    'code': 'pose_removal_failed', 'message': str(exc),
                },
            )
            return
        self._emit_pose_removed(
            remove_id, pending, success, message, None if success else {
                'code': 'pose_removal_failed', 'message': message,
            },
        )

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
            expired_lists = [(list_id, pending) for list_id, pending in self._pending_lists.items()
                             if now >= pending[1]]
            for list_id, _ in expired_lists:
                del self._pending_lists[list_id]
            expired_gets = [
                (get_id, pending)
                for get_id, pending in self._pending_gets.items()
                if now >= pending[1]
            ]
            for get_id, _ in expired_gets:
                del self._pending_gets[get_id]
            expired_removals = [
                (remove_id, pending)
                for remove_id, pending in self._pending_removals.items()
                if now >= pending[1]
            ]
            for remove_id, _ in expired_removals:
                del self._pending_removals[remove_id]
        for save_id, pending in expired:
            self._client.remove_pending_request(pending[0])
            pending[0].cancel()
            self._emit(save_id, pending, False, {
                'code': 'pose_saving_timeout',
                'message': 'No response from pose manager; the pose may still have been saved',
            })
        for list_id, pending in expired_lists:
            self._list_client.remove_pending_request(pending[0])
            pending[0].cancel()
            self._emit_list(list_id, pending, False, [], 'No response from pose list service', {
                'code': 'pose_listing_timeout', 'message': 'No response from pose list service',
            })
        for get_id, pending in expired_gets:
            self._get_client.remove_pending_request(pending[0])
            pending[0].cancel()
            self._emit_pose_retrieved(
                get_id, pending, False, 'No response from pose retrieval service', error={
                    'code': 'pose_retrieval_timeout',
                    'message': 'No response from pose retrieval service',
                },
            )
        for remove_id, pending in expired_removals:
            self._remove_client.remove_pending_request(pending[0])
            pending[0].cancel()
            self._emit_pose_removed(
                remove_id, pending, False, 'No response from pose removal service', {
                    'code': 'pose_removal_timeout',
                    'message': 'No response from pose removal service',
                },
            )