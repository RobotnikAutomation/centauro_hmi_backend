from .robots.mock_robot import MockRobot
from .robots.real_robot import RobotnikTeleoperationRobot


def create_robot(robot_type, speed_percentage, command_timeout_sec, node=None,
                 joint_states_topic='/robot/joint_states',
                 teleoperation_node_name='/robot/arm_teleoperation_node', parameter_client=None,
                 base_cmd_vel_topic='/robot/pad_teleop/cmd_vel', base_max_linear_speed=1.0,
                 base_max_angular_speed=1.5):
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
            base_cmd_vel_topic,
            base_max_linear_speed,
            base_max_angular_speed,
        )
    raise ValueError(f'Tipo de robot no soportado: {robot_type}')
