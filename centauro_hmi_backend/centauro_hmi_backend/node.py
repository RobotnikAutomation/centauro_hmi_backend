import asyncio
from concurrent.futures import TimeoutError as FutureTimeoutError
import json
import threading
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from .protocol import encode, message, test_image_data
from .robots.mock_robot import tool_position
from .robot import create_robot
from .stats import InboundStats, compact_payload
from .transport import WebSocketTransport
from .robot_model import RobotModelProvider

PERIODIC_COMMANDS = {
    'teleoperation.deadman',
    'arm.joint_jog',
    'arm.cartesian_jog',
    'teleoperation.freedrive',
}


class HmiBackend(Node):
    def __init__(self):
        super().__init__('centauro_hmi_backend')
        self.declare_parameter('robot.type', 'mock')
        self.declare_parameter('websocket_host', '127.0.0.1')
        self.declare_parameter('websocket_port', 8765)
        self.declare_parameter('telemetry_hz', 20.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('initial_speed_percentage', 25.0)
        self.declare_parameter('log_stats_period_sec', 5.0)
        self.declare_parameter('log_payloads', True)
        self.declare_parameter('log_payload_max_chars', 180)
        self.declare_parameter('robot_model_max_size_bytes', 50 * 1024 * 1024)
        self.declare_parameter('robot_model_chunk_size', 64 * 1024)
        robot_type = str(self.get_parameter('robot.type').value)
        self.robot = create_robot(robot_type, float(self.get_parameter('initial_speed_percentage').value), float(self.get_parameter('command_timeout_sec').value))
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
        self.loop = asyncio.new_event_loop()
        self._publisher_task = None
        self._shutting_down = False
        self.transport = WebSocketTransport(
            str(self.get_parameter('websocket_host').value),
            int(self.get_parameter('websocket_port').value),
            self.ws_command,
            self.ws_connected,
            self.ws_disconnected,
        )
        threading.Thread(target=self._run_loop, daemon=True).start()
        hz = float(self.get_parameter('telemetry_hz').value)
        self.create_timer(1.0 / hz, self.tick)
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
            constraints = {'directions': {'x+': True, 'x-': True, 'y+': True, 'y-': True, 'z+': True, 'z-': True},
                           'max_velocity_percentage': payload['teleoperation']['speed_percentage'], 'reason': 'none'}
            await self.transport.publish(encode(message('constraints', constraints)))
            await self.transport.publish(encode(message('robot_status', {'base': 'available', 'arm': 'available', 'tool': 'available', 'battery_percentage': 87.0, 'alarms': []})))
            await self.transport.publish(encode(message('tool_camera', {'encoding': 'base64', 'format': 'png', 'data': test_image_data()})))

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
            if name == 'robot.model.get':
                command_result = {'accepted': True, 'result': self.robot_model.manifest()}
            elif name == 'robot.model.download':
                manifest = self.robot_model.manifest()
                self.robot_model.bundle()
                command_result = {'accepted': True, 'result': manifest}
                model_chunks = self.robot_model.chunks()
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

    def tick(self):
        with self.lock:
            self.robot.tick(1.0 / float(self.get_parameter('telemetry_hz').value))
            snapshot = self.robot.snapshot()
            events = self.robot.take_events()
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
