"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""

import math
import numpy as np
from dqrobotics import *
from dqrobotics.utils import DQ_Geometry
from dqrobotics.robot_modeling import DQ_Kinematics
from termcolor import cprint

def rotation_axis_jacobian(primitive: DQ,
                           r: DQ,
                           Jr: np.ndarray):
    """
    https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8742769
    See Eq (26)
    Returns: The suitable Jacobian matrix.
    """
    return haminus4(k_ * conj(r)) @ Jr \
           + hamiplus4(r * k_) @ C4() @ Jr

def _l_normal_dot_product_Jacobian(normals: list[DQ],
                                   primitive: DQ,
                                   r: DQ,
                                   Jr: np.ndarray) -> np.ndarray:
    W = None

    for normal in normals:
        J = vec4(normal).T @ rotation_axis_jacobian(primitive, r, Jr)
        if W is None:
            W = J
        else:
            np.vstack((W, J))

    return W

def needle_jacobian(Jx_needle, x_needle: DQ, ps_vessel: list[DQ]):
    """
    First idea, "needle" Jacobian. It is defined as J = [Jr Jpi]^T
    x: The pose of the centre of the needle
    Jx: The analytical Jacobian of the pose of the centre of the needle
    p_vessel: The position of the entry point in the vessel
    """
    p_needle = translation(x_needle)
    # Radius constraint
    Jt_needle = DQ_Kinematics.translation_jacobian(Jx_needle, x_needle)
    # Plane constraint
    Jpi_needle = DQ_Kinematics.plane_jacobian(Jx_needle, x_needle, k_)
    # Two-point line rotation constraint


    W_needle = None

    for p_vessel in ps_vessel:
        Jradius = DQ_Kinematics.point_to_point_distance_jacobian(Jt_needle, p_needle, p_vessel)
        Jpi = DQ_Kinematics.plane_to_point_distance_jacobian(Jpi_needle, p_vessel)
        W = np.vstack((Jradius, -Jradius, Jpi, -Jpi))

        if W_needle is None:
            W_needle = W
        else:
            np.vstack((W_needle, W))

    return W_needle


def needle_w(x_needle: DQ,
             ps_vessel: list[DQ],
             needle_radius: float,
             vfi_gain_planes: float,
             vfi_gain_radius: float,
             d_safe_planes: float,
             d_safe_radius: float,
             verbose: bool):
    """
    First idea, "needle" Jacobian. It is defined as J = [Jr Jpi]^T
    x_needle: The pose of the centre of the needle
    p_vessel: The position of the entry point in the vessel
    needle_radius: The radius of the needle
    """
    p_needle = translation(x_needle)
    w_needle = None

    for p_vessel in ps_vessel:
        # Just as a reminder, our Jacobians use the squared distance so keep that in mind
        current_radius_squared = DQ_Geometry.point_to_point_squared_distance(p_needle, p_vessel)
        needle_radius_squared = needle_radius ** 2

        radius_safe_delta = d_safe_radius ** 2
        radius_error_one = (needle_radius_squared + radius_safe_delta) - current_radius_squared
        radius_error_two = current_radius_squared - (needle_radius_squared - radius_safe_delta)

        if verbose:
            print(f"Upper radius {math.sqrt((needle_radius_squared + radius_safe_delta))}")
            if radius_error_one < 0:
                cprint(f"     ↑↑↑Constraint violation: {math.sqrt(-radius_error_one)}", "red")
            print(f"Current radius {math.sqrt(current_radius_squared)}")
            print(f"Lower radius {math.sqrt((needle_radius_squared - radius_safe_delta))}")
            if radius_error_two < 0:
                cprint(f"     ↑↑↑Constraint violation: {math.sqrt(-radius_error_two)}", "red")

        r_needle = rotation(x_needle)
        n_needle = r_needle * k_ * conj(r_needle)
        d_needle = dot(p_needle, n_needle)
        pi_needle = n_needle + E_ * d_needle

        current_plane_distance = DQ_Geometry.point_to_plane_distance(p_vessel, pi_needle)

        plane_error_one = d_safe_planes - current_plane_distance
        plane_error_two = current_plane_distance - (-d_safe_planes)

        if verbose:
            print(f"Upper plane {d_safe_planes - current_plane_distance}")
            if plane_error_one < 0:
                cprint(f"     ↑↑↑Constraint violation: {plane_error_one}", "red")
            print(f"Current plane {current_plane_distance}")
            print(f"Lower plane {current_plane_distance - (-d_safe_planes)}")
            if plane_error_two < 0:
                cprint(f"     ↑↑↑Constraint violation: {plane_error_two}", "red")

        w = np.vstack((vfi_gain_radius * radius_error_one,
                      vfi_gain_radius * radius_error_two,
                      2.0 * vfi_gain_planes * plane_error_one,
                      2.0 * vfi_gain_planes * plane_error_two))
        if w_needle is None:
            w_needle = w
        else:
            np.vstack((w_needle, w))

    return w_needle