from .robots.mock_robot import MockRobot
from .robots.real_robot import RobotnikTeleoperationRobot


def create_robot(robot_type, speed_percentage, command_timeout_sec, node=None,
                 joint_states_topic='/robot/joint_states',
                 teleoperation_node_name='/robot/arm_teleoperation_node', parameter_client=None):
    """Create the configured robot adapter."""
    if robot_type == 'mock':
        return MockRobot(speed_percentage, command_timeout_sec)
    if robot_type == 'real':
        if node is None:
            raise RuntimeError('El adaptador real requiere el nodo ROS del backend')
        return RobotnikTeleoperationRobot(
            node,
            command_timeout_sec,
            joint_states_topic,
            teleoperation_node_name,
            parameter_client,
        )
    raise ValueError(f'Tipo de robot no soportado: {robot_type}')
