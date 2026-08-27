"""
Copyright (C) 2020-25 Murilo Marques Marinho (www.murilomarinho.info)
MIT License

Task-space controller with remote-centre-of-motion (RCM) and joint-limit
constraints, implemented as a quadratic program over joint velocities.
"""
from typing import Optional, Sequence, Tuple

from dqrobotics.robot_modeling import DQ_Kinematics, DQ_SerialManipulator
from dqrobotics.utils import DQ_Geometry
from dqrobotics import *
from marinholab.solvers.qpoases import Solver

import numpy as np

from marinholab.working.needlemanipulation._debug import (
    debug_rcm,
    normalize_verbose,
)


class ICRA19TaskSpaceController:
    """
    An implementation of the task-space controller described in:
     "A Unified Framework for the Teleoperation of Surgical Robots in Constrained Workspaces".
     Marinho, M. M; et al.
     In 2019 IEEE International Conference on Robotics and Automation (ICRA), pages 2721–2727, May 2019. IEEE
     http://doi.org/10.1109/ICRA.2019.8794363
    """

    def __init__(self,
                 kinematics: DQ_SerialManipulator,
                 gain: float,
                 damping: float,
                 alpha: float,
                 rcm_constraints: Sequence[Tuple[DQ, float, int]],
                 vfi_gain: float = 2.0,
                 **kwargs) -> None:
        """
        Initialize the controller.
        :param kinematics: A suitable DQ_SerialManipulator object.
        :param gain: A positive float. Controller proportional gain.
        :param damping: A positive float. Damping factor.
        :param alpha: A float between 0 and 1. Soft priority between translation and rotation.
        :param rcm_constraints: A list of tuples (p, r, ith), where p is the position of the constraint as a pure quaternion
        r is the radius of the constraint, and ith is the index of the joint this constraint relates to.
        :param vfi_gain: Violation Field Indicator gain applied to the RCM constraints.
        :param kwargs: Optional keyword arguments. Recognized keys: ``verbose``
            (bool or dict) — per-category debug output; ``True`` prints
            every category, ``False`` prints nothing, and a dict selects
            categories by name (e.g. ``{"rcm": True}``).
        """

        self.qp_solver = Solver()
        self.kinematics: DQ_SerialManipulator = kinematics
        self.gain: float = gain
        self.damping: float = damping
        self.alpha: float = alpha
        self.rcm_constraints: Sequence[Tuple[DQ, float, int]] = rcm_constraints
        self.vfi_gain: float = vfi_gain
        self.er_accumulated: np.ndarray = np.zeros((4,))
        self.et_accumulated: np.ndarray = np.zeros((4,))
        self.integral_gain: float = 0.0

        self.verbose = normalize_verbose(kwargs.get("verbose", False))

        # Pose / Jacobian / task error of the last control step.
        self.last_x: Optional[DQ] = None
        self.last_Jx: Optional[np.ndarray] = None
        self.last_error: Optional[np.ndarray] = None

    def get_last_robot_pose(self) -> DQ:
        """
        Retrieves the last recorded position of the robot.

        This method returns the most recently recorded pose of the end-effector of the robot.

        :return: The last recorded x-axis position of the robot.
        :rtype: DQ
        """
        assert self.last_x is not None, "No control step has been computed yet."
        return self.last_x

    def get_last_error(self) -> Optional[np.ndarray]:
        """Return the task error of the last control step, if any.

        :return: The stacked translation/rotation error vector of the last
            control step, or ``None`` before the first step.
        """
        return self.last_error


    @staticmethod
    def get_rcm_constraint(Jx: np.ndarray,
                           x: DQ,
                           primitive: DQ,
                           p: DQ,
                           d_safe: float,
                           eta_d: float,
                           ) -> Tuple[np.ndarray, np.ndarray]:
        """
        This static method computes the Remote Centre of Motion (RCM) constraint
        for the end-effector represented by x and its Jacobian Jx. It calculates the
        inequality matrix and vector that ensure the minimum safe squared distance (d_safe) between a line
        and a point is maintained during motion.

        :param Jx: The pose Jacobian of the robot.
        :param x: The current pose of the end-effector, compatible with Jx.
        :param primitive: The primitive in the end-effector in which the line is spanned. For instance i_, j_, or k_.
        :param p: The centre of the RCM constraint, represented as a pure quaternion.
        :param d_safe: The safe distance (float) to maintain between the line and the
            point, squared internally in the calculation.
        :param eta_d: VFI gain.
        :return: A tuple containing:
            - W (np.array): The inequality constraint matrix derived from the line-to-point
              distance Jacobian.
            - w (np.array): The inequality constraint vector determined by the distance
              error and safety distance.
        """

        # Get the line Jacobian for the primitive
        Jl = DQ_Kinematics.line_jacobian(Jx, x, primitive)

        # Get the line with respect to the base
        t = translation(x)
        r = rotation(x)
        l = Ad(r, primitive)
        l_dq = l + E_ * cross(t, l)

        # Get the line-to-point distance Jacobian
        Jl_p = DQ_Kinematics.line_to_point_distance_jacobian(Jl, l_dq, p)

        # Get the line-to-point square distance
        Dl_p = DQ_Geometry.point_to_line_squared_distance(p, l_dq)

        # Get the distance error
        D_safe = d_safe ** 2
        D_tilde = D_safe - Dl_p

        # The inequality matrix and vector
        W = np.array(Jl_p)
        w = np.array([eta_d * D_tilde])

        return W, w

    def _get_optimization_parameters(
        self,
        q: np.ndarray,
        xd: DQ,
    ) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
        """Assemble the QP cost (``H``, ``f``) and inequality constraints (``W``, ``w``).

        Computes the soft-priority blend of the translation and rotation
        task-space costs plus the damping term, and stacks the joint-limit
        and (if any) RCM inequality constraints. Also records the end
        effector pose, its pose Jacobian, and the task error for later use.

        Args:
            q: Current joint configuration vector.
            xd: Desired end-effector pose (a unit dual quaternion).

        Returns:
            A 4-tuple ``(H, f, W, w)`` where ``H`` and ``f`` are the QP cost
            matrix and linear term, and ``W``/``w`` are the stacked
            inequality matrix and right-hand side (``None`` when only the
            always-present joint-limit rows exist, i.e. ``W == W_jl``).
        """
        DOF = len(q)

        # Get current pose information
        x = self.kinematics.fkm(q)
        self.last_x = x

        # Calculate errors
        et = vec4(translation(x) - translation(xd))
        er = ICRA19TaskSpaceController._get_rotation_error(x, xd)

        self.et_accumulated = self.et_accumulated + et
        self.er_accumulated = self.er_accumulated + er

        self.last_error = np.vstack((er, et))

        # Get the Translation Jacobian and Rotation Jacobian
        Jx = self.kinematics.pose_jacobian(q)
        self.last_Jx = Jx
        rd = rotation(xd)
        Jr = DQ_Kinematics.rotation_jacobian(Jx)
        Nr = haminus4(rd) @ C4() @ Jr

        Jt = DQ_Kinematics.translation_jacobian(Jx, x)

        # Translation term
        Ht = Jt.transpose() @ Jt
        ft = self.gain * Jt.transpose() @ (et + self.integral_gain * self.et_accumulated)

        # Rotation term
        Hr = Nr.transpose() @ Nr
        fr = self.gain * Nr.transpose() @ (er + self.integral_gain * self.er_accumulated)

        # Damping term
        if isinstance(self.damping, np.ndarray):
            Hd = self.damping
        else:
            Hd = np.eye(DOF, DOF) * self.damping * self.damping

        # Combine terms using the soft priority
        H = self.alpha * Ht + (1.0 - self.alpha) * Hr + Hd
        f = self.alpha * ft + (1.0 - self.alpha) * fr

        # Joint limits
        lower_joint_limits = self.kinematics.get_lower_q_limit()
        upper_joint_limits = self.kinematics.get_upper_q_limit()
        W_jl = np.vstack((-1.0 * np.eye(DOF, DOF), np.eye(DOF, DOF)))
        w_jl = np.hstack((-1.0 * (lower_joint_limits - q), 1.0 * (upper_joint_limits - q)))

        # RCM constraints
        W: Optional[np.ndarray] = W_jl
        w: Optional[np.ndarray] = w_jl
        constraint_counter = 0

        if self.rcm_constraints is not None:
            for constraint in self.rcm_constraints:
                p, r, idx = constraint
                Jx_idx = self.kinematics.pose_jacobian(q, idx)
                x_idx = self.kinematics.fkm(q, idx)

                W_c_idx, w_c = self.get_rcm_constraint(Jx_idx, x_idx, k_, p, r, self.vfi_gain)

                debug_rcm(
                    self.verbose,
                    constraint_counter,
                    w_c[0],
                    self.vfi_gain,
                )
                constraint_counter += 1

                # Full matrix and vector
                W_c = np.zeros((1,DOF))
                w_c = np.zeros(1)
                # Add the current partial results
                W_c[0, 0:idx+1] = W_c_idx

                if W is None:
                    W = W_c
                    w = w_c
                else:
                    W = np.vstack((W, W_c))
                    w = np.hstack((w, w_c))

        return H, f, W, w

    def compute_setpoint_control_signal(self, q: np.ndarray, xd: DQ) -> np.ndarray:
        """Compute the control signal for the next step.

        Solves the constrained quadratic program built by
        :meth:`_get_optimization_parameters` for the joint velocity that
        drives the end effector toward the desired pose.

        Args:
            q: The current joint positions (vector of size ``DOF``).
            xd: The desired end-effector pose (a unit dual quaternion).

        Returns:
            The joint-velocity vector ``u`` to be applied for this step.

        Raises:
            Exception: If ``xd`` is not a unit dual quaternion.
        """
        DOF = len(q)
        if not is_unit(xd):
            raise Exception("ICRA19TaskSpaceController::compute_setpoint_control_signal::xd should be an unit dual "
                            "quaternion")

        H, f, W, w = self._get_optimization_parameters(q, xd)

        # Solve the quadratic program
        if W is not None and w is not None:
            u = self.qp_solver.solve_quadratic_program(H, f, W, np.squeeze(w), None, None)
        else:
            W = np.zeros((DOF, DOF))
            w = np.zeros(DOF)
            u = self.qp_solver.solve_quadratic_program(H, f, W, np.squeeze(w), None, None)

        return u

    @staticmethod
    def _get_rotation_error(x: DQ, xd: DQ) -> np.ndarray:
        """Return the 4-component rotation error between two poses.

        Uses the dual-quaternion invariant ``conj(r_x) * r_xd``: the error
        that is closer to the identity (i.e. whose dual part is smaller in
        norm) is selected, avoiding the 180° ambiguity.

        Args:
            x: Current end-effector pose.
            xd: Desired end-effector pose.

        Returns:
            The 4-component rotation error vector.
        """
        # Calculate error from invariant
        error_1 = vec4(conj(rotation(x))*rotation(xd) - 1)
        error_2 = vec4(conj(rotation(x))*rotation(xd) + 1)

        # Calculate 'distance' from invariant
        norm_1 = np.linalg.norm(error_1)
        norm_2 = np.linalg.norm(error_2)

        # Check the closest invariant and return the proper error
        if norm_1 < norm_2:
            return error_1
        else:
            return error_2