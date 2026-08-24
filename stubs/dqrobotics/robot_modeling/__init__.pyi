"""Stubs for `dqrobotics.robot_modeling` (see `stubs/dqrobotics/__init__.pyi`)."""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np
from numpy.typing import NDArray

from dqrobotics import DQ

__all__ = [
    "DQ_SerialManipulator",
    "DQ_Kinematics",
]


class DQ_SerialManipulator:
    """Base class for serial-manipulator kinematics models in dual quaternions.

    Subclasses (for example the C++-backed
    ``M3_SerialManipulatorSimulatorFriendly``) provide forward kinematics
    and Jacobians for a chain of actuated joints.
    """

    def fkm(self, q: NDArray[np.float64], idx: int = 0) -> DQ: ...
    def fkm_derivative(
        self, q: NDArray[np.float64], q_dot: NDArray[np.float64], idx: int = 0
    ) -> DQ: ...

    def pose_jacobian(
        self, q: NDArray[np.float64], idx: int = 0
    ) -> NDArray[np.float64]: ...
    def pose_jacobian_derivative(
        self,
        q: NDArray[np.float64],
        q_dot: NDArray[np.float64],
        idx: int = 0,
    ) -> NDArray[np.float64]: ...

    def set_lower_q_limit(self, lower_q_limit: NDArray[np.float64]) -> None: ...
    def get_lower_q_limit(self) -> NDArray[np.float64]: ...
    def set_upper_q_limit(self, upper_q_limit: NDArray[np.float64]) -> None: ...
    def get_upper_q_limit(self) -> NDArray[np.float64]: ...

    def set_effector(self, x_effector: DQ) -> None: ...
    def get_effector(self) -> DQ: ...
    def set_effector_target(self, xd_effector: DQ) -> None: ...
    def get_effector_target(self) -> DQ: ...

    def get_name(self) -> str: ...


class DQ_Kinematics:
    """A catalogue of closed-form Jacobians for common dual-quaternion tasks.

    All methods are stateless (static); each takes the pose Jacobian of the
    end-effector and returns the task Jacobian as a NumPy matrix.
    """

    @staticmethod
    def line_jacobian(
        Jx: NDArray[np.float64], x: DQ, primitive: DQ
    ) -> NDArray[np.float64]: ...

    @staticmethod
    def line_to_point_distance_jacobian(
        Jl: NDArray[np.float64], l: DQ, p: DQ
    ) -> NDArray[np.float64]: ...

    @staticmethod
    def rotation_jacobian(
        Jx: NDArray[np.float64],
    ) -> NDArray[np.float64]: ...

    @staticmethod
    def translation_jacobian(
        Jx: NDArray[np.float64], x: DQ,
    ) -> NDArray[np.float64]: ...

    @staticmethod
    def plane_jacobian(
        Jx: NDArray[np.float64], x: DQ, primitive: DQ,
    ) -> NDArray[np.float64]: ...

    @staticmethod
    def point_to_point_distance_jacobian(
        Jt: NDArray[np.float64], p1: DQ, p2: DQ,
    ) -> NDArray[np.float64]: ...

    @staticmethod
    def plane_to_point_distance_jacobian(
        Jpi: NDArray[np.float64], p: DQ,
    ) -> NDArray[np.float64]: ...

    @staticmethod
    def point_to_plane_distance_jacobian(
        Jt: NDArray[np.float64], p: DQ, plane: DQ,
    ) -> NDArray[np.float64]: ...

    @staticmethod
    def point_to_line_distance_jacobian(
        Jt: NDArray[np.float64], p: DQ, line: DQ,
    ) -> NDArray[np.float64]: ...
