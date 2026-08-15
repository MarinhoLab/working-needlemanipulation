"""
Copyright (C) 2025-26 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from dqrobotics import *
from dqrobotics._dqrobotics import DQ
from dqrobotics.utils import DQ_Geometry
from dqrobotics.robot_modeling import DQ_Kinematics
from termcolor import cprint

if TYPE_CHECKING:
    from numpy.typing import NDArray


def rotation_axis_jacobian(
    primitive: DQ,
    r: DQ,
    Jr: NDArray[np.floating],
) -> NDArray[np.floating]:
    """Compute the rotation axis Jacobian.

    See https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8742769 Eq (26).

    Args:
        primitive: The primitive dual quaternion.
        r: The rotation dual quaternion.
        Jr: The rotation Jacobian matrix.

    Returns:
        The suitable Jacobian matrix.
    """
    return haminus4(primitive * conj(r)) @ Jr \
           + hamiplus4(r * primitive) @ C4() @ Jr


def normal_dot_product_jacobian(
    normal: DQ,
    primitive: DQ,
    r: DQ,
    Jr: NDArray[np.floating],
) -> NDArray[np.floating]:
    """Compute the normal dot product Jacobian.

    To keep the needle 'disk' vertical.

    Args:
        normal: The normal vector as a dual quaternion.
        primitive: The primitive dual quaternion.
        r: The rotation dual quaternion.
        Jr: The rotation Jacobian matrix.

    Returns:
        The normal dot product Jacobian matrix.
    """
    J_normal = vec4(normal).T @ rotation_axis_jacobian(primitive, r, Jr)
    return J_normal

def cone_jacobian():
    """
    Constrain the point to be inside the cone.
    Keep the angle. Possibly very similar to this: normal_dot_product_jacobian

    The negative z-axis points forward from the needle tip.

    dot(a,b) = ||a|| ||b|| cos φ
    Impose  ||a||=||b||=1
    dot(a,b) = cos φ
    φ = acos( dot(a,b) )

    We can simplify the Jacobian using the dot product.

    d_min <= dot(a,b) <= d_max

    Point-to-line distance, where the d_safe depends on the distance to p

    We combine these two distances to make sure that the point is within the code.
    D_p_line = point_to_line_distance( (p1,n1), needle_tip)
    This one will be applicable anyway.

    D_safe = point_to_plane_distance( (p1,n1), needle_tip)
    This D_safe will need to be slightly changed according to the desired angle.
    45 degrees = 1 => tangent of the desired angle

    tan(point_to_point_distance) -> Need a new Jacobian
    d/dt(tan(t)) = sec2(t)

    """
    pass


def needle_jacobian(
    Jx_needle: NDArray[np.floating],
    x_needle: DQ,
    ps_vessel: list[DQ],
    ns_vessel: list[DQ],
    planes_active: bool = True,
    spheres_active: bool = True,
    driving_angle_active: bool = True,
    insertion_angle_active: bool = False,
    needle_radius: float | None = None,
) -> NDArray[np.floating] | None:
    """Compute the "needle" Jacobian.

    It is defined as J = [Jr Jpi]^T.

    Args:
        Jx_needle: The analytical Jacobian of the pose of the centre of the needle.
        x_needle: The pose of the centre of the needle.
        ps_vessel: The positions of the entry points in the vessels.
        ns_vessel: The normals of the entry points in the vessels.
        planes_active: Whether to include plane constraints.
        spheres_active: Whether to include sphere constraints.
        driving_angle_active: Whether to include driving angle constraints.
        insertion_angle_active: Whether to include insertion angle constraints.
        needle_radius: The needle radius. If None, the insertion constraint
            will not be calculated.

    Returns:
        The needle Jacobian matrix, or None if no constraints were active.
    """
    p_needle = translation(x_needle)
    r_needle = rotation(x_needle)

    Jr_needle = DQ_Kinematics.rotation_jacobian(Jx_needle)

    # Radius constraint
    Jt_needle = DQ_Kinematics.translation_jacobian(Jx_needle, x_needle)
    # Plane constraint
    Jpi_needle = DQ_Kinematics.plane_jacobian(Jx_needle, x_needle, k_)

    W_needle = None

    if spheres_active:
        for p_vessel in ps_vessel:
            Jradius = DQ_Kinematics.point_to_point_distance_jacobian(Jt_needle, p_needle, p_vessel)
            W = np.vstack((Jradius, -Jradius))

            # Stack vertically
            W_needle = (np.vstack((W_needle, W)) if W_needle is not None else W)

    if planes_active:

        for p_vessel in ps_vessel:
            Jpi = DQ_Kinematics.plane_to_point_distance_jacobian(Jpi_needle, p_vessel)
            W = np.vstack((Jpi, -Jpi))

            # Stack vertically
            W_needle = (np.vstack((W_needle, W)) if W_needle is not None else W)

    if driving_angle_active:

        if ns_vessel is not None:
            for n_vessel in ns_vessel:
                J_normal = normal_dot_product_jacobian(n_vessel, k_, r_needle, Jr_needle)
                W = np.vstack((J_normal, -J_normal))

                # Stack vertically
                W_needle = (np.vstack((W_needle, W)) if W_needle is not None else W)

    if insertion_angle_active:

        if needle_radius is not None:

            # x_needle_tip = x_needle * (1 + 0.5 * E_ * i_ * needle_radius)
            # r_needle_tip = rotation(x_needle_tip)

            J_normal = normal_dot_product_jacobian(ns_vessel[0], -j_, r_needle, Jr_needle)
            W = np.vstack((J_normal, -J_normal))

            # Stack vertically
            W_needle = (np.vstack((W_needle, W)) if W_needle is not None else W)

    return W_needle


def needle_w(
    x_needle: DQ,
    ps_vessel: list[DQ],
    ns_vessel: list[DQ],
    needle_radius: float,
    vfi_gain_planes: float,
    vfi_gain_radius: float,
    vfi_gain_angles: float,
    vfi_gain_needle_insertion_angles: float,
    d_safe_planes: float | None,
    d_safe_radius: float | None,
    d_safe_angles: float | None,
    d_safe_needle_insertion_angles: float | None = None,
    verbose: bool = True,
) -> NDArray[np.floating] | None:
    """Compute the needle violation field indicator vector.

    Args:
        x_needle: The pose of the centre of the needle.
        ps_vessel: The positions of the entry points in the vessels.
        ns_vessel: If not None, the first item is the insertion point and
            the second is the extraction point.
        needle_radius: The radius of the needle.
        vfi_gain_planes: VFI gain for plane constraints.
        vfi_gain_radius: VFI gain for radius constraints.
        vfi_gain_angles: VFI gain for angle constraints.
        vfi_gain_needle_insertion_angles: VFI gain for insertion angle constraints.
        d_safe_planes: Safe distance for plane constraints, or None to disable.
        d_safe_radius: Safe distance for radius constraints, or None to disable.
        d_safe_angles: Safe distance for angle constraints, or None to disable.
        d_safe_needle_insertion_angles: Safe distance for insertion angle constraints,
            or None to disable.
        verbose: Whether to print constraint diagnostics.

    Returns:
        The needle VFI vector, or None if no constraints were active.
    """
    p_needle = translation(x_needle)
    r_needle = rotation(x_needle)
    w_needle = None

    for p_vessel in ps_vessel:
        # Just as a reminder, our Jacobians use the squared distance so keep that in mind
        current_radius_squared = DQ_Geometry.point_to_point_squared_distance(p_needle, p_vessel)
        needle_radius_squared = needle_radius ** 2

        n_needle = r_needle * k_ * conj(r_needle)
        d_needle = dot(p_needle, n_needle)
        pi_needle = n_needle + E_ * d_needle

        current_plane_distance = DQ_Geometry.point_to_plane_distance(p_vessel, pi_needle)

        w = None

        # Plane distance calculation
        if d_safe_planes is not None:
            plane_error_one = d_safe_planes - current_plane_distance
            plane_error_two = current_plane_distance - (-d_safe_planes)

            # Add plane constraints
            plane_w = np.array([2.0 * vfi_gain_planes * plane_error_one,
                                2.0 * vfi_gain_planes * plane_error_two])
            w = np.vstack((w, plane_w)) if w is not None else plane_w

            if verbose:
                print(f"Upper plane {d_safe_planes - current_plane_distance}")
                if plane_error_one < 0:
                    cprint(f"     ↑↑↑Constraint violation: {plane_error_one}", "red")
                print(f"Current plane {current_plane_distance}")
                print(f"Lower plane {current_plane_distance - (-d_safe_planes)}")
                if plane_error_two < 0:
                    cprint(f"     ↑↑↑Constraint violation: {plane_error_two}", "red")

        # Radius distance calculation
        if d_safe_radius is not None:

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

            # Add radius constraints
            radius_w = np.array([vfi_gain_radius * radius_error_one,
                                 vfi_gain_radius * radius_error_two])
            w = np.vstack((w, radius_w)) if w is not None else radius_w



        w_needle = (np.vstack((w_needle, w)) if w_needle is not None else w)

    # Needle insertion angle to allow the tip to match the required angle in the XX-point checklist
    if ns_vessel is not None and d_safe_needle_insertion_angles is not None:
        x_needle_tip = x_needle * (1 + 0.5 * E_ * i_ * needle_radius)
        r_needle_tip = rotation(x_needle_tip)
        # Our needle is defined such that the y-axis points backwards from the tip
        # So we get the andle about the -y
        lmy = Ad(r_needle_tip, -j_)
        # Check the angle with the first vessel because it's an insertion
        current_dot = dot(ns_vessel[0], lmy).q[0]
        max_dot = math.cos(math.pi / 2 - d_safe_needle_insertion_angles)  # Positive
        min_dot = math.cos(math.pi / 2 + d_safe_needle_insertion_angles)  # Negative

        dot_error_one = max_dot - current_dot
        dot_error_two = current_dot - min_dot

        if verbose:
            print(f"Upper dot {dot_error_one}")
            if dot_error_one < 0:
                cprint(f"     ↑↑↑Constraint violation needle insertion: {dot_error_one}", "red")
            print(f"Current dot {current_dot}")
            print(f"Lower dot {dot_error_two}")
            if dot_error_two < 0:
                cprint(f"     ↑↑↑Constraint violation needle insertion: {dot_error_two}", "red")

        # Times two because it's not quadratic
        w = np.vstack((2.0 * vfi_gain_needle_insertion_angles * dot_error_one,
                       2.0 * vfi_gain_needle_insertion_angles * dot_error_two))

        w_needle = (np.vstack((w_needle, w)) if w_needle is not None else w)

    # Needle passing angle to keep the disc straight up
    if ns_vessel is not None and d_safe_angles is not None:
        for n_vessel in ns_vessel:
            lz = Ad(r_needle, k_)
            current_dot = dot(n_vessel, lz).q[0]
            max_dot = math.cos(math.pi/2 - d_safe_angles) # Positive
            min_dot = math.cos(math.pi/2 + d_safe_angles) # Negative

            dot_error_one = max_dot - current_dot
            dot_error_two = current_dot - min_dot

            if verbose:
                print(f"Upper dot {dot_error_one}")
                if dot_error_one < 0:
                    cprint(f"     ↑↑↑Constraint violation: {dot_error_one}", "red")
                print(f"Current dot {current_dot}")
                print(f"Lower dot {dot_error_two}")
                if dot_error_two < 0:
                    cprint(f"     ↑↑↑Constraint violation: {dot_error_two}", "red")

            # Times two because it's not quadratic
            w = np.vstack((2.0 * vfi_gain_angles * dot_error_one,
                           2.0 * vfi_gain_angles * dot_error_two))

            w_needle = (np.vstack((w_needle, w)) if w_needle is not None else w)

    return w_needle