"""
Copyright (C) 2020-25 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""
from dqrobotics.robot_modeling import DQ_Kinematics, DQ_SerialManipulator
from dqrobotics.utils import DQ_Geometry
from dqrobotics import *
from dqrobotics.solvers import DQ_QuadprogSolver

import numpy as np


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
                 rcm_constraints: list[tuple[DQ, float]]):
        """
        Initialize the controller.
        :param kinematics: A suitable DQ_SerialManipulator object.
        :param gain: A positive float. Controller proportional gain.
        :param damping: A positive float. Damping factor.
        :param alpha: A float between 0 and 1. Soft priority between translation and rotation.
        :param rcm_constraints: A list of tuples (p, r), where p is the position of the constraint as a pure quaternion
        and r is the radius of the constraint.
        """

        self.qp_solver = DQ_QuadprogSolver()
        self.kinematics: DQ_SerialManipulator = kinematics
        self.gain: float = gain
        self.damping: float = damping
        self.alpha: float = alpha
        self.rcm_constraints: list[tuple[DQ, float]] = rcm_constraints

        self.last_x: np.array = None
        self.last_Jx: np.array = None
        self.last_error: np.array = None

    def get_last_robot_pose(self) -> DQ:
        """
        Retrieves the last recorded position of the robot.

        This method returns the most recently recorded pose of the end-effector of the robot.

        :return: The last recorded x-axis position of the robot.
        :rtype: DQ
        """
        return self.last_x

    def get_last_error(self) -> np.array:
        return self.last_error


    @staticmethod
    def get_rcm_constraint(Jx: np.array, x: DQ, primitive: DQ,
                           p: DQ, d_safe: float,
                           eta_d: float,
                           ) -> (np.array, np.array):
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

    def _get_optimization_parameters(self, q, xd):
        DOF = len(q)

        # Get current pose information
        x = self.kinematics.fkm(q)
        self.last_x = x

        # Calculate errors
        et = vec4(translation(x) - translation(xd))
        er = ICRA19TaskSpaceController._get_rotation_error(x, xd)

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
        ft = self.gain * Jt.transpose() @ et

        # Rotation term
        Hr = Nr.transpose() @ Nr
        fr = self.gain * Nr.transpose() @ er

        # Damping term
        Hd = np.eye(DOF, DOF) * self.damping * self.damping

        # Combine terms using the soft priority
        H = self.alpha * Ht + (1.0 - self.alpha) * Hr + Hd
        f = self.alpha * ft + (1.0 - self.alpha) * fr

        # RCM constraints
        W = None
        w = None
        if self.rcm_constraints is not None:
            for constraint in self.rcm_constraints:
                p, r = constraint
                W_c, w_c = self.get_rcm_constraint(Jx, x, k_, p, r, 0.1)
                if W is None:
                    W = W_c
                    w = w_c
                else:
                    W = np.vstack((W, W_c))
                    w = np.vstack((w, w_c))

        return H, f, W, w

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

        # Solve the quadratic program
        if W is not None:
            u = self.qp_solver.solve_quadratic_program(H, f, W, np.squeeze(w), None, None)
        else:
            W = np.zeros((DOF, DOF))
            w = np.zeros(DOF)
            u = self.qp_solver.solve_quadratic_program(H, f, W, np.squeeze(w), None, None)

        return u

    @staticmethod
    def _get_rotation_error(x, xd):
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