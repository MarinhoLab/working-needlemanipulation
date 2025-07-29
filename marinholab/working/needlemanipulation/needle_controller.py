"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""
import numpy as np

from marinholab.working.needlemanipulation.icra2019_controller import ICRA19TaskSpaceController
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulator

from marinholab.working.needlemanipulation import needle_jacobian, needle_w

class NeedleController(ICRA19TaskSpaceController):
    def __init__(self,
                 kinematics: DQ_SerialManipulator,
                 gain: float,
                 damping: float,
                 alpha: float,
                 rcm_constraints: list[tuple[DQ, float, int]],
                 relative_needle_pose: DQ,
                 vessel_positions: list[DQ],
                 needle_radius: float,
                 vfi_gain: float = 2.0,
                 **kwargs):
        super().__init__(kinematics, gain, damping, alpha, rcm_constraints, vfi_gain, **kwargs)

        if "vfi_gain_planes" in kwargs:
            self.vfi_gain_planes = kwargs["vfi_gain_planes"]
        if "vfi_gain_radius" in kwargs:
            self.vfi_gain_radius = kwargs["vfi_gain_radius"]
        if "vfi_gain_angles" in kwargs:
            self.vfi_gain_angles = kwargs["vfi_gain_angles"]
        if "d_safe_planes" in kwargs:
            self.d_safe_planes = kwargs["d_safe_planes"]
        if "d_safe_radius" in kwargs:
            self.d_safe_radius = kwargs["d_safe_radius"]
        if "vessel_normals" in kwargs:
            self.vessel_normals = kwargs["vessel_normals"]

        self.relative_needle_pose = relative_needle_pose
        self.vessel_positions = vessel_positions
        self.needle_radius = needle_radius

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
        Jx_needle = haminus8(self.relative_needle_pose) @ Jx
        x_needle = x * self.relative_needle_pose

        # VFI-related Jacobian
        W_needle = needle_jacobian(
            Jx_needle,
            x_needle,
            self.vessel_positions,
            self.vessel_normals if hasattr(self,"vessel_normals") else None)
        # VFI w
        w_needle = needle_w(
            x_needle=x_needle,
            ps_vessel=self.vessel_positions,
            ns_vessel=self.vessel_normals if hasattr(self,"vessel_normals") else None,
            needle_radius=self.needle_radius,
            vfi_gain_planes=self.vfi_gain_planes if hasattr(self,"vfi_gain_planes") else self.vfi_gain,
            vfi_gain_radius=self.vfi_gain_radius if hasattr(self,"vfi_gain_radius") else self.vfi_gain,
            vfi_gain_angles=self.vfi_gain_angles if hasattr(self, "vfi_gain_angles") else self.vfi_gain,
            d_safe_planes=self.d_safe_planes if hasattr(self,"d_safe_planes") else 0.0005,
            d_safe_radius=self.d_safe_radius if hasattr(self,"d_safe_radius") else 0.0005,
            verbose=self.verbose
        ).reshape((W_needle.shape[0],))

        if W is not None and w is not None:
            W = np.vstack((W, W_needle))
            w = np.hstack((w, w_needle))
        else:
            W = W_needle
            w = w_needle

        u = self.qp_solver.solve_quadratic_program(H, f, W, np.squeeze(w), None, None)

        return u