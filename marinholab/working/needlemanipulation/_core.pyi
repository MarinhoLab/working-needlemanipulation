"""Type stubs for marinholab.working.needlemanipulation._core C++ extension."""

from __future__ import annotations

import sys
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from dqrobotics import DQ
    from dqrobotics.robot_modeling import DQ_SerialManipulator
    from typing_extensions import Self

__all__ = [
    "M3_SerialManipulatorSimulatorFriendly",
    "ActuationType",
    "__version__",
]

__version__: str

class ActuationType:
    """Actuation type and axis of each joint.

    RZ: Revolution about the z-axis.
    RY: Revolution about the y-axis.
    RX: Revolution about the x-axis.
    TZ: Translation along the z-axis.
    TY: Translation along the y-axis.
    TX: Translation along the x-axis.
    """

    RZ: ActuationType
    RY: ActuationType
    RX: ActuationType
    TZ: ActuationType
    TY: ActuationType
    TX: ActuationType


class M3_SerialManipulatorSimulatorFriendly(DQ_SerialManipulator):
    """A serial manipulator with configurable joint offsets and actuation types.

    Supports both revolute (R) and prismatic (T) joints along any axis (X, Y, Z).
    Inherits from ``DQ_SerialManipulator`` and overrides the kinematic methods.
    """

    ActuationType = ActuationType

    def __init__(
        self,
        offset_before: list[DQ],
        offset_after: list[DQ],
        actuation_types: list[ActuationType],
    ) -> None:
        """Initialize the manipulator.

        Args:
            offset_before: Offset of each joint transformation before actuation.
            offset_after: Offset of each joint transformation after actuation.
            actuation_types: Actuation type and axis of each joint.
        """
        ...

    def raw_pose_jacobian(
        self,
        q_vec: np.ndarray,
        to_ith_link: int = ...,
    ) -> np.ndarray:
        """Retrieve the raw pose Jacobian.

        Args:
            q_vec: Joint configuration vector.
            to_ith_link: End link index (default: end-effector).

        Returns:
            The raw pose Jacobian matrix.
        """
        ...

    def raw_fkm(
        self,
        q_vec: np.ndarray,
        to_ith_link: int = ...,
    ) -> DQ:
        """Retrieve the raw forward kinematics.

        Args:
            q_vec: Joint configuration vector.
            to_ith_link: End link index (default: end-effector).

        Returns:
            The dual quaternion pose of the specified link.
        """
        ...

    def raw_pose_jacobian_derivative(
        self,
        q: np.ndarray,
        q_dot: np.ndarray,
        to_ith_link: int = ...,
    ) -> np.ndarray:
        """Retrieve the raw pose Jacobian derivative.

        Args:
            q: Joint configuration vector.
            q_dot: Joint velocity vector.
            to_ith_link: End link index (default: end-effector).

        Returns:
            The raw pose Jacobian derivative matrix.
        """
        ...