import base64
import threading

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


FRONT_CAMERA_TOPIC = '/robot/front_rgbd_camera/color/image_raw'
DEFAULT_MOCK_ROSBAG_PATH = '/home/robot/teleop_ws/src/front_camera_10s'


class ImageStreamBridge:
    """Caches the latest frame per topic, dropping frames between snapshots."""

    def __init__(self, node, image_streams):
        self._lock = threading.Lock()
        self._images = {}
        self._image_sequences = {}
        self._published_image_sequences = {}
        self._image_subscriptions = []
        for stream_id, topic in self.parse_image_streams(image_streams):
            self._images[stream_id] = None
            self._image_sequences[stream_id] = 0
            self._published_image_sequences[stream_id] = 0
            self._image_subscriptions.append(node.create_subscription(
                Image,
                topic,
                lambda msg, key=stream_id, image_topic=topic: self._image_callback(
                    key, image_topic, msg
                ),
                qos_profile_sensor_data,
            ))

    @staticmethod
    def parse_image_streams(stream_specs):
        streams = []
        seen = set()
        for spec in stream_specs:
            stream_id, separator, topic = str(spec).partition('=')
            if not separator or not stream_id.strip() or not topic.strip():
                raise ValueError("image stream entries must use 'stream_name=/absolute/topic' format")
            stream_id, topic = stream_id.strip(), topic.strip()
            if stream_id in seen:
                raise ValueError(f'duplicate image stream name: {stream_id}')
            seen.add(stream_id)
            streams.append((stream_id, topic))
        return streams

    def _image_callback(self, stream_id, topic, msg):
        payload = {
            'stream_id': stream_id,
            'topic': topic,
            'encoding': 'base64',
            'format': 'ros_image',
            'data': base64.b64encode(bytes(msg.data)).decode('ascii'),
            'image_encoding': msg.encoding,
            'width': msg.width,
            'height': msg.height,
            'step': msg.step,
            'is_bigendian': bool(msg.is_bigendian),
            'timestamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
        }
        with self._lock:
            self._images[stream_id] = payload
            self._image_sequences[stream_id] += 1

    def _take_images_locked(self):
        images = []
        for stream_id, image in self._images.items():
            sequence = self._image_sequences[stream_id]
            if image is not None and sequence > self._published_image_sequences[stream_id]:
                frame = dict(image)
                frame['sequence'] = sequence
                images.append(frame)
                self._published_image_sequences[stream_id] = sequence
        return images

    def snapshot(self):
        with self._lock:
            return self._take_images_locked()
