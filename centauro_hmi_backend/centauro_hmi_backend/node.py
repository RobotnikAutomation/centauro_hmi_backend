import asyncio
from concurrent.futures import TimeoutError as FutureTimeoutError
import json
import math
import os
import threading
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster

from .protocol import encode, message
from .camera import DEFAULT_MOCK_ROSBAG_PATH, FRONT_CAMERA_TOPIC, ImageStreamBridge
from .robots.mock_robot import tool_position
from .robot import create_robot
from .stats import InboundStats, compact_payload
from .transport import WebSocketTransport
from .robot_model import RobotModelProvider
from .real_status import RealStatusBridge
from .pad_config import load_pad_speed_limits
from .ik_bridge import CartesianIkBridge

PERIODIC_COMMANDS = {
    'teleoperation.deadman',
    'arm.joint_jog',
    'arm.cartesian_jog',
    'arm.cartesian_target',
    'teleoperation.freedrive',
    'base.joystick',
}


class HmiBackend(Node):
    def __init__(self):
        super().__init__('centauro_hmi_backend')
        self.declare_parameter('robot.type', 'mock')
        self.declare_parameter(
            'mock.camera_transport', 'websocket',
            ParameterDescriptor(read_only=True),
        )
        self.declare_parameter(
            'mock.rosbag_path', DEFAULT_MOCK_ROSBAG_PATH,
            ParameterDescriptor(read_only=True),
        )
        self.declare_parameter('websocket_host', '127.0.0.1')
        self.declare_parameter('websocket_port', 8765)
        self.declare_parameter('websocket_ping_timeout_sec', 1800.0)
        self.declare_parameter('telemetry_hz', 20.0)
        self.declare_parameter('base_cmd_vel_hz', 100.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('initial_speed_percentage', 25.0)
        self.declare_parameter('robot.joint_states_topic', '/robot/joint_states')
        self.declare_parameter('robot.base_cmd_vel_topic', '/robot/pad_teleop/cmd_vel')
        self.declare_parameter('robot.pad_model', os.environ.get('JOY_CONFIG') or 'ps5')
        self.declare_parameter('robot.teleoperation_node_name', '/robot/arm_teleoperation_node')
        self.declare_parameter(
            'robot.image_streams',
            ['front_camera=/robot/front_rgbd_camera/color/image_raw'],
        )
        self.declare_parameter('robot.battery_topic', '/robot/battery_estimator/data')
        self.declare_parameter('robot.base_status_topic', '/robot/robotnik_base_hw_monitor/status')
        self.declare_parameter('robot.robot_mode_topic', '/robot/arm/io_and_status_controller/robot_mode')
        self.declare_parameter(
            'robot.robot_program_running_topic',
            '/robot/arm/io_and_status_controller/robot_program_running',
        )
        self.declare_parameter('robot.safety_mode_topic', '/robot/arm/io_and_status_controller/safety_mode')
        self.declare_parameter('robot.tool_data_topic', '/robot/arm/io_and_status_controller/tool_data')
        self.declare_parameter('robot.safety_module_status_topic', '/robot/safety_module/status')
        self.declare_parameter('log_stats_period_sec', 5.0)
        self.declare_parameter('log_payloads', True)
        self.declare_parameter('log_payload_max_chars', 180)
        self.declare_parameter('robot_model_max_size_bytes', 50 * 1024 * 1024)
        self.declare_parameter('robot_model_chunk_size', 64 * 1024)
        self.declare_parameter('ik.target_frame', 'hmi_tcp_target')
        self.declare_parameter('ik.tcp_frame', 'robot_arm_tool0')
        self.declare_parameter('ik.base_frame', 'robot_arm_base_link')
        self.declare_parameter('ik.namespace', 'robot')
        robot_type = str(self.get_parameter('robot.type').value)
        camera_transport = str(self.get_parameter('mock.camera_transport').value)
        if camera_transport not in ('websocket', 'webrtc'):
            raise ValueError('mock.camera_transport must be websocket or webrtc')
        self.mock_camera = None
        if robot_type == 'mock' and camera_transport == 'websocket':
            self.mock_camera = ImageStreamBridge(
                self, [f'front_camera={FRONT_CAMERA_TOPIC}']
            )
        base_max_linear_speed, base_max_angular_speed = load_pad_speed_limits(
            self.get_parameter('robot.pad_model').value
        )
        self.robot = create_robot(
            robot_type,
            float(self.get_parameter('initial_speed_percentage').value),
            float(self.get_parameter('command_timeout_sec').value),
            node=self,
            joint_states_topic=str(self.get_parameter('robot.joint_states_topic').value),
            teleoperation_node_name=str(self.get_parameter('robot.teleoperation_node_name').value),
            base_cmd_vel_topic=str(self.get_parameter('robot.base_cmd_vel_topic').value),
            base_max_linear_speed=base_max_linear_speed,
            base_max_angular_speed=base_max_angular_speed,
        )
        self.real_status = None
        if robot_type == 'real':
            status_parameter_names = [parameter_name for _, _, parameter_name in RealStatusBridge.STATUS_TOPICS]
            self.real_status = RealStatusBridge(
                self,
                list(self.get_parameter('robot.image_streams').value),
                str(self.get_parameter('robot.battery_topic').value),
                {
                    parameter_name: str(self.get_parameter(parameter_name).value)
                    for parameter_name in status_parameter_names
                },
            )
        self.robot_model = None
        if robot_type == 'mock':
            self.robot_model = RobotModelProvider(
                get_package_share_directory('centauro_hmi_backend'),
                self.get_parameter('robot_model_max_size_bytes').value,
                self.get_parameter('robot_model_chunk_size').value,
            )
        self.command_pub = self.create_subscription(String, '/centauro/hmi/command', self.ros_command, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.state_pub = self.create_publisher(String, '/centauro/hmi/state', 10)
        self.event_pub = self.create_publisher(String, '/centauro/hmi/events', 10)
        self.planned_path_pub = self.create_publisher(Marker, '/centauro/hmi/planned_tool_path', 10)
        self.lock = threading.Lock()
        self.inbound_stats = InboundStats()
        self._last_teleoperation_status = None
        self._last_real_robot_status = None
        self.loop = asyncio.new_event_loop()
        ik_namespace = str(self.get_parameter('ik.namespace').value).strip('/')
        self.ik_bridge = CartesianIkBridge(
            self,
            self._publish_event,
            str(self.get_parameter('ik.target_frame').value),
            str(self.get_parameter('ik.base_frame').value),
            f'/{ik_namespace}/follow_tcp_target' if ik_namespace else '/follow_tcp_target',
            float(self.get_parameter('command_timeout_sec').value),
        )
        self._publisher_task = None
        self._shutting_down = False
        self.transport = WebSocketTransport(
            str(self.get_parameter('websocket_host').value),
            int(self.get_parameter('websocket_port').value),
            self.ws_command,
            self.ws_connected,
            self.ws_disconnected,
            ping_timeout_sec=float(self.get_parameter('websocket_ping_timeout_sec').value),
        )
        threading.Thread(target=self._run_loop, daemon=True).start()
        hz = float(self.get_parameter('telemetry_hz').value)
        self.create_timer(1.0 / hz, self.tick)
        base_cmd_vel_hz = float(self.get_parameter('base_cmd_vel_hz').value)
        if not math.isfinite(base_cmd_vel_hz) or base_cmd_vel_hz <= 0.0:
            raise ValueError('base_cmd_vel_hz must be finite and greater than zero')
        if getattr(self.robot, 'is_real', False):
            self.create_timer(1.0 / base_cmd_vel_hz, self.robot.publish_base_velocity)
        else:
            self._sim_base_broadcaster = TransformBroadcaster(self)
            self._last_sim_base_update = self.get_clock().now()
            self.create_timer(1.0 / base_cmd_vel_hz, self.publish_sim_base)
        stats_period = float(self.get_parameter('log_stats_period_sec').value)
        if stats_period > 0.0:
            self.create_timer(stats_period, self.log_inbound_stats)
        self.get_logger().info(f'Iniciando backend con robot {robot_type} en ws://{self.transport.host}:{self.transport.port}')

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.transport.start())
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return
        self.get_logger().info(f'WebSocket escuchando en ws://{self.transport.host}:{self.transport.port}')
        self._publisher_task = self.loop.create_task(self.publisher_loop())
        try:
            self.loop.run_forever()
        finally:
            self.loop.close()

    async def _shutdown_async(self):
        if self._publisher_task and not self._publisher_task.done():
            self._publisher_task.cancel()
            await asyncio.gather(self._publisher_task, return_exceptions=True)
        await self.transport.stop()

    async def ws_connected(self, websocket):
        self.get_logger().info(
            f'Cliente WebSocket conectado: {websocket.remote_address}; '
            f'clientes={len(self.transport.clients)}'
        )
        if self.real_status is not None:
            robot_status, _ = self.real_status.snapshot()
            if robot_status:
                await self.transport.send_to_client(
                    websocket, encode(message('robot_status', robot_status))
                )

    async def ws_disconnected(self, websocket):
        self.get_logger().info(
            f'Cliente WebSocket desconectado: {websocket.remote_address}; '
            f'clientes={len(self.transport.clients)}'
        )

    async def publisher_loop(self):
        while rclpy.ok():
            await asyncio.sleep(1.0 / float(self.get_parameter('telemetry_hz').value))
            with self.lock: payload = self.robot.snapshot()
            await self.transport.publish(encode(message('telemetry', payload)))
            if self.real_status is not None:
                robot_status, camera_streams = self.real_status.snapshot()
                if robot_status and robot_status != self._last_real_robot_status:
                    self._last_real_robot_status = robot_status
                    await self.transport.publish(encode(message('robot_status', robot_status)))
                for camera_payload in camera_streams:
                    await self.transport.publish(encode(message('tool_camera', camera_payload)))
            else:
                constraints = {
                    'directions': {'x+': True, 'x-': True, 'y+': True, 'y-': True, 'z+': True, 'z-': True},
                    'max_velocity_percentage': payload['teleoperation']['speed_percentage'],
                    'reason': 'none',
                }
                await self.transport.publish(encode(message('constraints', constraints)))
                await self.transport.publish(encode(message('robot_status', {'base': 'available', 'arm': 'available', 'tool': 'available', 'battery_percentage': 87.0, 'alarms': []})))
                if self.mock_camera is not None:
                    for camera_payload in self.mock_camera.snapshot():
                        await self.transport.publish(encode(message('tool_camera', camera_payload)))

    async def ws_command(self, websocket, raw, source='websocket'):
        name = None
        payload = None
        events = []
        model_chunks = None
        invalid = False
        try:
            data = json.loads(raw)
            if not isinstance(data, dict):
                raise TypeError('el mensaje debe ser un objeto JSON')
            payload = data.get('payload', {})
            if not isinstance(payload, dict):
                raise TypeError('payload debe ser un objeto JSON')
            name = payload.get('name') or data.get('name')
            if not name: raise ValueError('payload.name es obligatorio')
            if name in ('robot.model.get', 'robot.model.download') and not getattr(self.robot, 'supports_robot_model', True):
                command_result = {
                    'accepted': False,
                    'error': {'code': 'unsupported_command', 'message': 'robot model is not available from the real teleoperation adapter'},
                }
            elif name == 'robot.model.get':
                command_result = {'accepted': True, 'result': self.robot_model.manifest()}
            elif name == 'robot.model.download':
                manifest = self.robot_model.manifest()
                self.robot_model.bundle()
                command_result = {'accepted': True, 'result': manifest}
                model_chunks = self.robot_model.chunks()
            elif name == 'arm.cartesian_target':
                ik_bridge = getattr(self, 'ik_bridge', None)
                if ik_bridge is None:
                    command_result = {
                        'accepted': False,
                        'error': {'code': 'ik_unavailable', 'message': 'IK bridge is not configured'},
                    }
                else:
                    command_result = ik_bridge.command(payload)
            else:
                with self.lock:
                    command_result = self.robot.command(name, payload)
                    events = self.robot.take_events()
            response = None
            if name not in PERIODIC_COMMANDS:
                ack_payload = {'name': name, 'accepted': command_result['accepted']}
                if 'operation_id' in payload:
                    ack_payload['operation_id'] = payload['operation_id']
                if command_result.get('result') is not None:
                    ack_payload['result'] = command_result['result']
                if command_result.get('error') is not None:
                    ack_payload['error'] = command_result['error']
                response = message('ack', ack_payload, data.get('request_id'))
                invalid = not command_result['accepted']
            elif not command_result['accepted']:
                invalid = True
                response = message('error', command_result['error'], data.get('request_id'))
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            invalid = True
            response = message('error', {'code': 'invalid_command', 'message': str(exc)})
        self.inbound_stats.record(source, name, raw, payload, invalid)
        if websocket is not None and response is not None:
            await self.transport.send_to_client(websocket, encode(response))
        if websocket is not None and model_chunks is not None:
            for chunk in model_chunks:
                await self.transport.send_to_client(websocket, encode(message('robot_model_chunk', chunk)))
        for kind, event_payload in events:
            self._publish_event(kind, event_payload)

    def ros_command(self, msg):
        asyncio.run_coroutine_threadsafe(self.ws_command(None, msg.data, 'ros2'), self.loop)

    def log_inbound_stats(self):
        stats = self.inbound_stats.take_interval()
        msg_rate = stats['messages'] / stats['elapsed']
        byte_rate = stats['bytes'] / stats['elapsed']
        sources = ', '.join(f'{key}={value}' for key, value in sorted(stats['by_source'].items())) or 'ninguno'
        commands = ', '.join(
            f'{key}={value}' for key, value in sorted(stats['by_command'].items(), key=lambda item: (-item[1], item[0]))
        ) or 'ninguno'
        self.get_logger().info(
            f'RX {msg_rate:.1f} msg/s, {byte_rate:.0f} B/s | '
            f'intervalo={stats["messages"]}, total={stats["total_messages"]}, '
            f'errores={stats["errors"]}/{stats["total_errors"]}, '
            f'clientes={len(self.transport.clients)} | fuentes: {sources} | comandos: {commands}'
        )
        if bool(self.get_parameter('log_payloads').value) and stats['last_by_command']:
            max_chars = int(self.get_parameter('log_payload_max_chars').value)
            samples = '; '.join(
                f'{name}={compact_payload(payload, max_chars)}'
                for name, payload in sorted(stats['last_by_command'].items())
            )
            self.get_logger().info(f'Últimos mensajes RX del intervalo: {samples}')

    def publish_sim_base(self):
        now = self.get_clock().now()
        dt = max(0.0, (now - self._last_sim_base_update).nanoseconds / 1e9)
        self._last_sim_base_update = now
        with self.lock:
            self.robot.tick_base(dt)
            x, y, yaw = self.robot.base_pose
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'sim_base'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self._sim_base_broadcaster.sendTransform(transform)

    def tick(self):
        with self.lock:
            self.robot.tick(1.0 / float(self.get_parameter('telemetry_hz').value))
            snapshot = self.robot.snapshot()
            events = self.robot.take_events()
        self.ik_bridge.tick()
        if snapshot.get('feedback_available', True):
            joint = JointState()
            joint.header.stamp = self.get_clock().now().to_msg()
            joint.name, joint.position, joint.velocity = snapshot['joint_names'], snapshot['positions'], snapshot['velocities']
            self.joint_pub.publish(joint)
        state = String(); state.data = encode(message('telemetry', snapshot)); self.state_pub.publish(state)
        teleoperation = snapshot['teleoperation']
        if teleoperation != self._last_teleoperation_status:
            self._last_teleoperation_status = dict(teleoperation)
            self._publish_event('teleoperation_status', teleoperation)
        if self.robot.operation:
            self._publish_event('operation_status', {'operation_id': self.robot.operation.operation_id, 'status': self.robot.operation.status, 'progress': self.robot.operation.progress})
        for kind, event_payload in events:
            self._publish_event(kind, event_payload)

    def _publish_event(self, kind, payload):
        raw = encode(message(kind, payload))
        msg = String(); msg.data = raw; self.event_pub.publish(msg)
        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.transport.publish(raw), self.loop)
        if kind == 'planned_trajectory':
            self._publish_planned_path(payload)

    def _publish_planned_path(self, trajectory):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'planned_trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.015
        marker.color.r = 0.2
        marker.color.g = 0.9
        marker.color.b = 0.3
        marker.color.a = 0.9
        for point in trajectory['trajectory']['points']:
            x, y, z = tool_position(point['positions'])
            marker.points.append(Point(x=x, y=y, z=z))
        self.planned_path_pub.publish(marker)

    def destroy_node(self):
        if self._shutting_down:
            return
        self._shutting_down = True
        if self.loop.is_running():
            future = asyncio.run_coroutine_threadsafe(self._shutdown_async(), self.loop)
            try:
                future.result(timeout=2.0)
            except (FutureTimeoutError, RuntimeError):
                future.cancel()
            self.loop.call_soon_threadsafe(self.loop.stop)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HmiBackend()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
