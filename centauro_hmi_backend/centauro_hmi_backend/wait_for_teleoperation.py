import time

import rclpy
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient


CHECK_FREQUENCY_HZ = 0.5


class TeleoperationNodeWaiter(Node):
    def __init__(self):
        super().__init__('wait_for_teleoperation_node')
        self.declare_parameter(
            'robot.teleoperation_node_name', '/robot/arm_teleoperation_node'
        )
        node_name = str(self.get_parameter('robot.teleoperation_node_name').value)
        self.parameter_client = AsyncParameterClient(self, node_name)
        self.ready = False
        self.request_pending = False
        self.last_wait_log = 0.0
        self.next_query_time = 0.0
        self.create_timer(1.0 / CHECK_FREQUENCY_HZ, self.check_teleoperation_node)
        self.get_logger().info(f'Waiting for teleoperation node {node_name}')

    def check_teleoperation_node(self):
        if self.ready or self.request_pending:
            return
        now = time.monotonic()
        if now < self.next_query_time:
            return
        if not self.parameter_client.services_are_ready():
            if now - self.last_wait_log >= 10.0:
                self.get_logger().info('Teleoperation node parameter services are not available yet')
                self.last_wait_log = now
            return
        try:
            future = self.parameter_client.get_parameters(['default_velocity_percentage'])
        except Exception as exc:
            self.get_logger().warning(f'Could not query teleoperation node parameters: {exc}')
            self.next_query_time = now + 1.0
            return
        self.request_pending = True
        future.add_done_callback(self.parameter_query_complete)

    def parameter_query_complete(self, future):
        self.request_pending = False
        try:
            response = future.result()
            if (len(response.values) != 1 or
                    response.values[0].type != ParameterType.PARAMETER_DOUBLE):
                self.get_logger().warning(
                    'Teleoperation node is running but default_velocity_percentage is not declared as a double yet'
                )
                self.next_query_time = time.monotonic() + 1.0
                return
        except Exception as exc:
            self.get_logger().warning(f'Teleoperation node parameter query failed: {exc}')
            self.next_query_time = time.monotonic() + 1.0
            return
        self.ready = True
        self.get_logger().info('Teleoperation node is ready; starting the HMI backend')


def main(args=None):
    rclpy.init(args=args)
    node = TeleoperationNodeWaiter()
    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=1.0 / CHECK_FREQUENCY_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.ready else 1