import asyncio
import base64
import math
from pathlib import Path
import struct

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

from .protocol import encode, message


DEFAULT_SCANS_ROSBAG_PATH = '/home/robot/teleop_ws/src/scanners_point_cloud_10s'
DEFAULT_POINTCLOUD_ROSBAG_PATH = '/home/robot/teleop_ws/src/front_camera_point_cloud_4s'
POINT_CLOUD_SOURCES = ('none', 'scans', 'pointcloud')


def scan_to_point_cloud(scan: LaserScan) -> PointCloud2:
    """Project valid returns into the scan's own frame, without a TF transform."""
    data = bytearray()
    for index, distance in enumerate(scan.ranges):
        if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
            angle = scan.angle_min + index * scan.angle_increment
            data.extend(struct.pack('<fff', distance * math.cos(angle),
                                    distance * math.sin(angle), 0.0))
    return PointCloud2(
        header=scan.header, height=1, width=len(data) // 12,
        fields=[
            PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
            for name, offset in (('x', 0), ('y', 4), ('z', 8))
        ],
        is_bigendian=False, point_step=12, row_step=len(data),
        data=bytes(data), is_dense=True,
    )


def point_cloud_payload(cloud: PointCloud2, topic: str, source: str) -> dict:
    return {
        'stream_id': source,
        'source': source,
        'topic': topic,
        'frame_id': cloud.header.frame_id,
        'encoding': 'base64',
        'format': 'ros_pointcloud2',
        'data': base64.b64encode(bytes(cloud.data)).decode('ascii'),
        'height': cloud.height,
        'width': cloud.width,
        'fields': [
            {'name': field.name, 'offset': field.offset,
             'datatype': field.datatype, 'count': field.count}
            for field in cloud.fields
        ],
        'is_bigendian': bool(cloud.is_bigendian),
        'point_step': cloud.point_step,
        'row_step': cloud.row_step,
        'is_dense': bool(cloud.is_dense),
        'timestamp': {
            'sec': cloud.header.stamp.sec,
            'nanosec': cloud.header.stamp.nanosec,
        },
    }


class MockPointCloudReplay:
    """Stream one filtered bag at recorded speed with bounded, one-frame memory."""

    def __init__(self, source: str, path: str):
        if source not in ('scans', 'pointcloud'):
            raise ValueError('point cloud replay source must be scans or pointcloud')
        if not Path(path).exists():
            raise FileNotFoundError(f'Mock {source} rosbag not found: {path}')
        self.source = source
        self.path = path
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(
            rosbag2_py.StorageOptions(uri=path),
            rosbag2_py.ConverterOptions('cdr', 'cdr'),
        )
        self.message_type = LaserScan if source == 'scans' else PointCloud2
        type_name = f'sensor_msgs/msg/{self.message_type.__name__}'
        topics = [
            entry.topic_metadata.name
            for entry in self.reader.get_metadata().topics_with_message_count
            if entry.topic_metadata.type == type_name and entry.message_count > 0
        ]
        if not topics:
            raise ValueError(f'Mock {source} rosbag {path} has no {type_name} messages')
        self.reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

    def read_frame(self):
        if not self.reader.has_next():
            return None
        topic, data, timestamp = self.reader.read_next()
        msg = deserialize_message(data, self.message_type)
        cloud = scan_to_point_cloud(msg) if self.source == 'scans' else msg
        return timestamp, point_cloud_payload(cloud, topic, self.source)

    async def run(self, publish):
        clock = asyncio.get_running_loop()
        sequence = 0
        loop_index = 0
        while True:
            first_timestamp = None
            previous_timestamp = None
            interval = 0.001
            started = clock.time()
            while True:
                frame = await asyncio.to_thread(self.read_frame)
                if frame is None:
                    break
                timestamp, payload = frame
                if first_timestamp is None:
                    first_timestamp = timestamp
                if previous_timestamp is not None:
                    interval = max(0.001, (timestamp - previous_timestamp) / 1e9)
                previous_timestamp = timestamp
                await asyncio.sleep(max(0.0, started + (timestamp - first_timestamp) / 1e9
                                        - clock.time()))
                sequence += 1
                payload.update(sequence=sequence, loop_index=loop_index)
                await publish(encode(message('point_cloud', payload)))
            if first_timestamp is None:
                raise RuntimeError(f'Mock {self.source} rosbag {self.path} yielded no frames')
            # Retain a frame interval across the loop boundary, including one-frame bags.
            await asyncio.sleep(interval)
            await asyncio.to_thread(self.reader.seek, 0)
            loop_index += 1


def create_mock_point_cloud_replay(robot_type, source, scans_path, pointcloud_path):
    if source not in POINT_CLOUD_SOURCES:
        raise ValueError('mock.point_cloud_source must be none, scans or pointcloud')
    if robot_type != 'mock' or source == 'none':
        return None
    return MockPointCloudReplay(
        source, scans_path if source == 'scans' else pointcloud_path,
    )
