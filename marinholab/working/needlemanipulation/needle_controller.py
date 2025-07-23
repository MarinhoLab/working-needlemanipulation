"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""
from typing import override
import numpy as np

from marinholab.working.needlemanipulation.icra2019_controller import ICRA19TaskSpaceController
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulator

from marinholab.working.needlemanipulation import needle_jacobian, needle_entry_error

class NeedleController(ICRA19TaskSpaceController):
    def __init__(self,
                 kinematics: DQ_SerialManipulator,
                 gain: float,
                 damping: float,
                 alpha: float,
                 rcm_constraints: list[tuple[DQ, float]],
                 relative_needle_pose: DQ,
                 vessel_position: DQ,
                 needle_radius: float):
        super().__init__(kinematics, gain, damping, alpha, rcm_constraints)

        self.relative_needle_pose = relative_needle_pose
        self.vessel_position = vessel_position
        self.needle_radius = needle_radius

    @override
    def compute_setpoint_control_signal(self, q, xd) -> np.array:
        """
        Get the control signal for the next step as the result of the constrained optimization.
        Joint limits are currently not considered.
        :param q: The current joint positions.
        :param xd: The desired pose.
        :return: The desired joint positions that should be sent to the robot.
        """
        DOF = len(q)
        if not is_unit(xd):
            raise Exception("ICRA19TaskSpaceController::compute_setpoint_control_signal::xd should be an unit dual "
                            "quaternion")

        H, f, W, w = self._get_optimization_parameters(q, xd)

        # The relative transformation of the needle is time-constant
        x = self.last_x
        Jx = self.last_Jx
        Jx_needle = haminus8(self.relative_needle_pose) * Jx
        x_needle = x * self.relative_needle_pose

        # VFI-related Jacobian
        J_needle = needle_jacobian(Jx_needle, x_needle, self.vessel_position)
        # VFI-related squared distance
        D_needle = needle_entry_error(x_needle, self.vessel_position, self.needle_radius)
        # VFI-related distance error
        D_tilde = 0 - D_needle

        # VFI
        W_needle = np.array(J_needle)
        w_needle = np.array([0.1 * D_tilde])

        if W is not None and w is not None:
            W = np.vstack((W, W_needle))
            w = np.vstack((w, w_needle))
        else:
            W = W_needle
            w = w_needle

        u = self.qp_solver.solve_quadratic_program(H, f, W, np.squeeze(w), None, None)

        return u