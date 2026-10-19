import time
import uuid

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster

from centauro_hmi_backend.tf_frames import TfFramesBridge


def transform(parent, child):
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.rotation.w = 1.0
    return msg


def make_bridge():
    bridge = TfFramesBridge.__new__(TfFramesBridge)
    bridge._buffer = Buffer()
    return bridge


def test_empty_tf_graph_has_empty_frame_list():
    assert make_bridge().snapshot() == {'frames': []}


def test_frames_include_roots_dynamic_static_and_disconnected_trees():
    bridge = make_bridge()
    bridge._buffer.set_transform_static(transform('world', 'base'), 'test')
    bridge._buffer.set_transform(transform('base', 'tool'), 'test')
    bridge._buffer.set_transform_static(transform('map', 'camera'), 'test')

    assert bridge.snapshot() == {
        'frames': ['base', 'camera', 'map', 'tool', 'world'],
    }

    bridge._buffer.set_transform(transform('base', 'tool'), 'test')
    assert bridge.snapshot()['frames'] == ['base', 'camera', 'map', 'tool', 'world']


def test_snapshot_reflects_tf_buffer_clearing():
    bridge = make_bridge()
    bridge._buffer.set_transform(transform('world', 'base'), 'test')
    bridge._buffer.clear()

    assert bridge.snapshot() == {'frames': []}


def test_listener_discovers_dynamic_and_preexisting_latched_static_tf():
    context = Context()
    rclpy.init(context=context)
    prefix = f'/hmi_tf_test_{uuid.uuid4().hex}'
    node = Node(
        'tf_frames_test',
        namespace=prefix,
        context=context,
        use_global_arguments=False,
        cli_args=['--ros-args', '-r', f'/tf:={prefix}/tf',
                  '-r', f'/tf_static:={prefix}/tf_static'],
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        static = StaticTransformBroadcaster(node)
        static.sendTransform(transform('world', 'base'))
        bridge = TfFramesBridge(node)
        dynamic = TransformBroadcaster(node)
        expected = {'frames': ['base', 'tool', 'world']}
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and bridge.snapshot() != expected:
            msg = transform('base', 'tool')
            msg.header.stamp = node.get_clock().now().to_msg()
            dynamic.sendTransform(msg)
            executor.spin_once(timeout_sec=0.05)

        assert bridge.snapshot() == expected
        bridge._listener.unregister()
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
