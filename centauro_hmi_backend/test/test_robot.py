import pytest

from centauro_hmi_backend.robots.mock_robot import MockRobot
from centauro_hmi_backend.robot import create_robot


def test_mock_robot_is_selected_by_configuration():
    assert isinstance(create_robot('mock', 25.0, 0.5), MockRobot)


def test_real_robot_is_explicitly_unavailable_until_implemented():
    with pytest.raises(RuntimeError, match='no está implementado'):
        create_robot('real', 25.0, 0.5)


def test_unknown_robot_type_is_rejected():
    with pytest.raises(ValueError, match='Tipo de robot no soportado'):
        create_robot('gazebo', 25.0, 0.5)
