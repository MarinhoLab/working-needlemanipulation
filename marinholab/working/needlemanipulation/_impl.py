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


def needle_jacobian(Jx_needle, x_needle: DQ, p_vessel: DQ):
    """
    First idea, "needle" Jacobian. It is defined as J = [Jr Jpi]^T
    x: The pose of the centre of the needle
    Jx: The analytical Jacobian of the pose of the centre of the needle
    p_vessel: The position of the entry point in the vessel
    """
    p_needle = translation(x_needle)
    Jt_needle = DQ_Kinematics.translation_jacobian(Jx_needle, x_needle)
    Jradius = DQ_Kinematics.point_to_point_distance_jacobian(Jt_needle, p_needle, p_vessel)
    Jpi_needle = DQ_Kinematics.plane_jacobian(Jx_needle, x_needle, k_)
    Jpi_needle = DQ_Kinematics.plane_to_point_distance_jacobian(Jpi_needle, p_vessel)
    return np.vstack((Jradius, -Jradius, Jpi_needle, -Jpi_needle))


def needle_w(x_needle: DQ, p_vessel: DQ, needle_radius: float, vfi_gain: float, verbose: bool):
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

    radius_safe_delta = 0.0005 ** 2
    radius_error_one = (needle_radius_squared + radius_safe_delta) - current_radius_squared
    radius_error_two = current_radius_squared - (needle_radius_squared - radius_safe_delta)

    if verbose:
        print(f"Upper radius {math.sqrt((needle_radius_squared + radius_safe_delta))}")
        if radius_error_one < 0:
            cprint(f"     Constraint violation: {radius_error_one}", "red")
        print(f"Current radius {math.sqrt(current_radius_squared)}")
        print(f"Lower radius {math.sqrt((needle_radius_squared - radius_safe_delta))}")
        if radius_error_two < 0:
            cprint(f"     Constraint violation: {radius_error_two}", "red")

    r_needle = rotation(x_needle)
    n_needle = r_needle * k_ * conj(r_needle)
    d_needle = dot(p_needle, n_needle)
    pi_needle = n_needle + E_ * d_needle

    current_plane_distance = DQ_Geometry.point_to_plane_distance(p_vessel, pi_needle)

    plane_error_one = radius_safe_delta - current_plane_distance
    plane_error_two = current_plane_distance - (-radius_safe_delta)

    if verbose:
        print(f"Upper plane {radius_safe_delta - current_plane_distance}")
        if plane_error_one < 0:
            cprint(f"     Constraint violation: {plane_error_one}", "red")
        print(f"Current plane {current_plane_distance}")
        print(f"Lower plane {current_plane_distance - (-radius_safe_delta)}")
        if plane_error_two < 0:
            cprint(f"     Constraint violation: {plane_error_two}", "red")

    # Adjusting VFI gain to be twice as much for the first-order distances.
    return np.vstack((vfi_gain * radius_error_one,
                      vfi_gain * radius_error_two,
                      2.0 * vfi_gain * plane_error_one,
                      2.0 * vfi_gain * plane_error_two))