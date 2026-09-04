from .robots.mock_robot import MockRobot


def create_robot(robot_type, speed_percentage, command_timeout_sec):
    """Create the configured robot adapter."""
    if robot_type == 'mock':
        return MockRobot(speed_percentage, command_timeout_sec)
    if robot_type == 'real':
        raise RuntimeError('El adaptador de robot real todavía no está implementado')
    raise ValueError(f'Tipo de robot no soportado: {robot_type}')
