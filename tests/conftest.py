"""
Conftest for mock imports and shared fixtures.
"""
import sys
from unittest.mock import MagicMock, Mock
import numpy as np


def pytest_configure(config):
    """Mock _core and M3_SerialManipulatorSimulatorFriendly before any test imports them."""
    # Create a mock _core module
    mock_core = MagicMock()

    # Create a mock M3_SerialManipulatorSimulatorFriendly class
    mock_manipulator = Mock()
    mock_manipulator.ActuationType = Mock()
    mock_manipulator.ActuationType.RX = 0
    mock_manipulator.ActuationType.RY = 1
    mock_manipulator.ActuationType.RZ = 2

    # The mock instance should support fkm, pose_jacobian, etc.
    def mock_fkm(q, idx=-1):
        from dqrobotics import DQ
        return DQ([1, 0, 0, 0, 0, 0, 0, 0])

    def mock_pose_jacobian(q, idx=-1):
        dof = len(q)
        return np.zeros((8, dof))

    def mock_get_lower_q_limit():
        return np.zeros(9)

    def mock_get_upper_q_limit():
        return np.ones(9)

    def mock_instance(*args, **kwargs):
        instance = Mock()
        instance.fkm = mock_fkm
        instance.pose_jacobian = mock_pose_jacobian
        instance.get_lower_q_limit = mock_get_lower_q_limit
        instance.get_upper_q_limit = mock_get_upper_q_limit
        instance.set_lower_q_limit = Mock()
        instance.set_upper_q_limit = Mock()
        return instance

    mock_manipulator.side_effect = mock_instance
    mock_core.M3_SerialManipulatorSimulatorFriendly = mock_manipulator

    sys.modules['marinholab.working.needlemanipulation._core'] = mock_core


# Also mock cv2 for PedriatricSimulator tests
sys.modules['cv2'] = MagicMock()