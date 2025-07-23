"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""

import numpy as np
from dqrobotics import *
from dqrobotics.utils import DQ_Geometry
from dqrobotics.robot_modeling import DQ_Kinematics


def needle_jacobian(Jx_needle, x_needle: DQ, p_vessel: DQ):
    """
    First idea, "needle" Jacobian. It is defined as J = [Jr Jpi]^T
    x: The pose of the centre of the needle
    Jx: The analytical Jacobian of the pose of the centre of the needle
    p_vessel: The position of the entry point in the vessel
    """
    p_needle = translation(x_needle)
    # static MatrixXd translation_jacobian (const MatrixXd& pose_jacobian, const DQ& pose);
    Jt_needle = DQ_Kinematics.translation_jacobian(Jx_needle, x_needle)
    # static MatrixXd point_to_point_distance_jacobian(const MatrixXd& translation_jacobian, const DQ& robot_point, const DQ& workspace_point);
    Jradius = DQ_Kinematics.point_to_point_distance_jacobian(Jt_needle, p_needle, p_vessel)
    # static MatrixXd plane_jacobian(const MatrixXd& pose_jacobian, const DQ& pose, const DQ& plane_normal);
    Jpi_needle = DQ_Kinematics.plane_jacobian(Jx_needle, x_needle, i_)
    # static MatrixXd plane_to_point_distance_jacobian(const MatrixXd& plane_jacobian, const DQ& workspace_point);
    Jpi_needle = DQ_Kinematics.plane_to_point_distance_jacobian(Jpi_needle, p_vessel)
    return np.vstack((Jradius, Jpi_needle))


def needle_entry_error(x_needle: DQ, p_vessel: DQ, needle_radius: float):
    """
    First idea, "needle" Jacobian. It is defined as J = [Jr Jpi]^T
    x_needle: The pose of the centre of the needle
    p_vessel: The position of the entry point in the vessel
    needle_radius: The radius of the needle
    """
    p_needle = translation(x_needle)
    # Just as a reminder, our Jacobians use the squared distance so keep that in mind
    current_radius_squared = DQ_Geometry.point_to_point_squared_distance(p_needle, p_vessel)
    needle_radius_squared = needle_radius ** 2
    radius_error = current_radius_squared - needle_radius_squared

    r_needle = rotation(x_needle)
    n_needle = r_needle * i_ * conj(r_needle)
    d_needle = dot(p_needle, n_needle)
    pi_needle = n_needle + E_ * d_needle
    current_plane_distance = DQ_Geometry.point_to_plane_distance(p_vessel, pi_needle)
    plane_error = current_plane_distance - 0
    return np.vstack((radius_error, plane_error))