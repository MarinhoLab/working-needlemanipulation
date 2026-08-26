"""
Copyright (C) 2025 Murilo Marques Marinho
(www.murilomarinho.info)

MIT License

Needle manipulation constraint implementation.

This module implements the Violation Field Indicator (VFI) constraint
machinery used by :class:`NeedleController`. It computes, for a needle pose
relative to one or more vessel primitives, the inequality matrix ``W``
(:func:`needle_jacobian`) and the right-hand-side vector ``w``
(:func:`needle_w`) that bound the joint velocities in the quadratic program
solved at each control step.

When ``insertion_constraints`` is enabled an additional angular insertion
constraint (the needle must approach the vessel within a depth-dependent
angle band) and a tip point-to-line insertion constraint are appended.
"""

import math

import numpy as np
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_Kinematics
from dqrobotics.utils import DQ_Geometry

from marinholab.working.needlemanipulation._debug import (
    Verbose,
    debug_insertion,
    debug_orientation,
    debug_plane,
    debug_radius,
    normalize_verbose,
)


def rotation_axis_jacobian(
    primitive: DQ,
    r: DQ,
    Jr: np.ndarray,
) -> np.ndarray:
    """Compute the Jacobian of a rotated axis. See Eq. (26) in IEEE 8742769."""
    return (
        haminus4(primitive * conj(r)) @ Jr
        + hamiplus4(r * primitive) @ C4() @ Jr
    )


def phi_z(normal: DQ, r: DQ) -> float:
    """Return cos(angle(normal, Ad(r, k_)))."""
    return float(dot(normal, Ad(r, k_)).q[0])


def J_phi_z(normal: DQ, r: DQ, Jr: np.ndarray) -> np.ndarray:
    """Compute the Jacobian of phi_z = <normal, Ad(r, k_)>."""
    J_p_z = rotation_axis_jacobian(k_, r, Jr)
    return np.asarray(vec4(normal).T @ J_p_z, dtype=np.float64)


def dot_product_safe(
    h: float,
    phi_min: float,
    phi_max: float,
    h_min: float,
    h_max: float,
) -> float:
    """
    Compute cos(phi_safe), where the maximum permitted angle increases
    from phi_min to phi_max as h increases from h_min to h_max.
    """
    if h_max <= h_min:
        raise ValueError("h_max must be greater than h_min.")

    h_n = (h - h_min) / (h_max - h_min)

    if h_n <= 0.0:
        phi_safe = phi_min
    elif h_n >= 1.0:
        phi_safe = phi_max
    else:
        phi_safe = phi_min + h_n * (phi_max - phi_min)

    return float(math.cos(phi_safe))


def J_dot_product_safe(
    h: float,
    J_h: np.ndarray,
    phi_min: float,
    phi_max: float,
    h_min: float,
    h_max: float,
) -> np.ndarray:
    """Compute the Jacobian of cos(phi_safe(h))."""
    if h_max <= h_min:
        raise ValueError("h_max must be greater than h_min.")

    J_h = np.asarray(J_h, dtype=np.float64)
    h_n = (h - h_min) / (h_max - h_min)

    if h_n <= 0.0:
        phi_safe = phi_min
        J_phi_safe = np.zeros_like(J_h, dtype=np.float64)
    elif h_n >= 1.0:
        phi_safe = phi_max
        J_phi_safe = np.zeros_like(J_h, dtype=np.float64)
    else:
        phi_safe = phi_min + h_n * (phi_max - phi_min)
        J_phi_safe = (
            (phi_max - phi_min) / (h_max - h_min) * J_h
        )

    return np.asarray(
        -math.sin(phi_safe) * J_phi_safe,
        dtype=np.float64,
    )


def phi_z_constraint_W(
    normal: DQ,
    r: DQ,
    Jr: np.ndarray,
    h: float,
    J_h: np.ndarray,
    phi_min: float,
    phi_max: float,
    h_min: float,
    h_max: float,
) -> np.ndarray:
    """
    Enforce angle(normal, p_z) <= phi_safe(h), equivalently
    phi_z >= dot_product_safe(h), using W @ q_dot <= w.
    """
    J_safe = J_dot_product_safe(
        h, J_h, phi_min, phi_max, h_min, h_max
    )
    J_current = J_phi_z(normal, r, Jr)
    return np.asarray(J_safe - J_current, dtype=np.float64)


def phi_z_constraint_w(
    normal: DQ,
    r: DQ,
    h: float,
    phi_min: float,
    phi_max: float,
    h_min: float,
    h_max: float,
    vfi_gain: float,
) -> float:
    """Compute w for phi_z >= dot_product_safe(h)."""
    safe_value = dot_product_safe(
        h, phi_min, phi_max, h_min, h_max
    )
    current_value = phi_z(normal, r)
    return float(vfi_gain * (current_value - safe_value))


def insertion_D(line: DQ, needle_pose: DQ) -> float:
    """Compute the squared point-to-line distance."""
    return float(
        DQ_Geometry.point_to_line_squared_distance(
            translation(needle_pose), line
        )
    )


def insertion_D_safe(angle: float, plane: DQ, needle_pose: DQ) -> float:
    """Compute D_safe = tan(angle)^2 * d_plane^2."""
    d_plane = DQ_Geometry.point_to_plane_distance(
        translation(needle_pose), plane
    )
    return float(math.tan(angle) ** 2 * d_plane ** 2)


def insertion_J_D_safe(
    Jt_needle: np.ndarray,
    angle: float,
    plane: DQ,
    needle_pose: DQ,
) -> np.ndarray:
    """Compute J_D_safe = 2*tan(angle)^2*d_plane*J_d_plane."""
    needle_t = translation(needle_pose)
    d_plane = DQ_Geometry.point_to_plane_distance(needle_t, plane)
    J_d_plane = np.asarray(
        DQ_Kinematics.point_to_plane_distance_jacobian(
            Jt_needle, needle_t, plane
        ),
        dtype=np.float64,
    )
    return np.asarray(
        2.0 * math.tan(angle) ** 2 * d_plane * J_d_plane,
        dtype=np.float64,
    )


def insertion_W(
    Jt_needle: np.ndarray,
    line: DQ,
    angle: float,
    plane: DQ,
    needle_pose: DQ,
) -> np.ndarray:
    """Compute W = J_D - J_D_safe for D <= D_safe."""
    J_D_safe = insertion_J_D_safe(
        Jt_needle, angle, plane, needle_pose
    )
    J_D = np.asarray(
        DQ_Kinematics.point_to_line_distance_jacobian(
            Jt_needle, translation(needle_pose), line
        ),
        dtype=np.float64,
    )
    return np.asarray(J_D - J_D_safe, dtype=np.float64)


def insertion_w(
    line: DQ,
    plane: DQ,
    angle: float,
    needle_pose: DQ,
    vfi_gain: float,
) -> float:
    """Compute w = gain * (D_safe - D)."""
    return float(
        vfi_gain
        * (
            insertion_D_safe(angle, plane, needle_pose)
            - insertion_D(line, needle_pose)
        )
    )


def normal_dot_product_jacobian(
    normal: DQ,
    primitive: DQ,
    r: DQ,
    Jr: np.ndarray,
) -> np.ndarray:
    """Compute the Jacobian of a normal/rotated-primitive dot product."""
    return np.asarray(
        vec4(normal).T @ rotation_axis_jacobian(primitive, r, Jr),
        dtype=np.float64,
    )


def needle_jacobian(
    Jx_needle: np.ndarray,
    x_needle: DQ,
    ps_vessel: list[DQ],
    ns_vessel: list[DQ] | None,
    insertion_constraints: bool = False,
    needle_offset: DQ = DQ([1.0]),
) -> np.ndarray:
    """Construct the needle inequality constraint matrix ``W``.

    Builds one row of ``W`` (in the ``W @ q_dot <= w`` formulation) per
    constraint enforced on the needle, where the needle pose is the end
    effector pose transformed by the (fixed) relative needle pose. The
    returned matrix has shape ``(n_constraints, DOF)``.

    Per vessel in ``ps_vessel`` a point-to-point radius constraint (inside/
    outside the needle radius band) and a plane constraint (the needle's own
    plane vs. the vessel point) are added. When ``ns_vessel`` is provided, an
    orientation constraint (needle axis vs. the vessel normal) is added for
    every vessel normal — except, when ``insertion_constraints`` is set, the
    first normal is instead handled by the depth-dependent angular insertion
    constraint below.

    Args:
        Jx_needle: Pose Jacobian of the needle frame, ``8 x DOF``.
        x_needle: Dual-quaternion pose of the needle frame.
        ps_vessel: Vessel positions as pure dual quaternions.
        ns_vessel: Vessel normals as pure dual quaternions, or ``None`` to
            skip orientation constraints.
        insertion_constraints: If ``True`` append the insertion point-to-line
            and angular (phi_z) constraints for the first vessel.
        needle_offset: Dual quaternion relating the needle tip to the needle
            frame (defaults to the identity).

    Returns:
        The stacked constraint Jacobian ``W``, shape ``(n_constraints, DOF)``
        (``(0, DOF)`` when no constraints are active).

    Raises:
        ValueError: If ``insertion_constraints`` is ``True`` but
            ``ps_vessel`` or ``ns_vessel`` is empty.
    """
    p_needle = translation(x_needle)
    r_needle = rotation(x_needle)

    Jr_needle = np.asarray(
        DQ_Kinematics.rotation_jacobian(Jx_needle), dtype=np.float64
    )
    Jt_needle = np.asarray(
        DQ_Kinematics.translation_jacobian(Jx_needle, x_needle),
        dtype=np.float64,
    )
    Jpi_needle = np.asarray(
        DQ_Kinematics.plane_jacobian(Jx_needle, x_needle, k_),
        dtype=np.float64,
    )

    W_needle = None

    for p_vessel in ps_vessel:
        J_radius = np.asarray(
            DQ_Kinematics.point_to_point_distance_jacobian(
                Jt_needle, p_needle, p_vessel
            ),
            dtype=np.float64,
        )
        J_plane = np.asarray(
            DQ_Kinematics.plane_to_point_distance_jacobian(
                Jpi_needle, p_vessel
            ),
            dtype=np.float64,
        )
        W = np.vstack((J_radius, -J_radius, J_plane, -J_plane))
        W_needle = np.vstack((W_needle, W)) if W_needle is not None else W

    if ns_vessel is not None:
        for i, n_vessel in enumerate(ns_vessel):
            # Replaced by the depth-dependent orientation constraint.
            if insertion_constraints and i == 0:
                continue

            J_normal = normal_dot_product_jacobian(
                n_vessel, k_, r_needle, Jr_needle
            )
            W = np.vstack((J_normal, -J_normal))
            W_needle = np.vstack((W_needle, W)) if W_needle is not None else W

    if insertion_constraints:
        if not ps_vessel or not ns_vessel:
            raise ValueError(
                "ps_vessel and ns_vessel must be non-empty when "
                "insertion constraints are enabled."
            )

        x_needle_tip = x_needle * needle_offset
        Jx_needle_tip = haminus8(needle_offset) @ Jx_needle
        Jt_needle_tip = np.asarray(
            DQ_Kinematics.translation_jacobian(
                Jx_needle_tip, x_needle_tip
            ),
            dtype=np.float64,
        )

        n_vessel = ns_vessel[0]
        p_vessel = ps_vessel[0]
        line = n_vessel + E_ * cross(p_vessel, n_vessel)
        plane = n_vessel + E_ * dot(p_vessel, n_vessel)

        insertion_angle = 2.0 * math.pi / 3.0
        phi_min = math.pi / 6.0
        phi_max = math.pi / 2.0
        h_min = 0.0
        h_max = 0.05

        W_insertion = insertion_W(
            Jt_needle_tip,
            line,
            insertion_angle,
            plane,
            x_needle_tip,
        )

        needle_tip_t = translation(x_needle_tip)
        h = float(
            DQ_Geometry.point_to_plane_distance(needle_tip_t, plane)
        )
        J_h = np.asarray(
            DQ_Kinematics.point_to_plane_distance_jacobian(
                Jt_needle_tip, needle_tip_t, plane
            ),
            dtype=np.float64,
        )

        W_phi = phi_z_constraint_W(
            n_vessel,
            r_needle,
            Jr_needle,
            h,
            J_h,
            phi_min,
            phi_max,
            h_min,
            h_max,
        )

        W = np.vstack((
            np.asarray(W_insertion, dtype=np.float64).reshape(1, -1),
            np.asarray(W_phi, dtype=np.float64).reshape(1, -1),
        ))
        W_needle = np.vstack((W_needle, W)) if W_needle is not None else W

    if W_needle is None:
        return np.empty((0, Jx_needle.shape[1]), dtype=np.float64)

    return np.asarray(W_needle, dtype=np.float64)


def needle_w(
    x_needle: DQ,
    ps_vessel: list[DQ],
    ns_vessel: list[DQ] | None,
    needle_radius: float,
    vfi_gain_planes: float,
    vfi_gain_radius: float,
    vfi_gain_angles: float,
    d_safe_planes: float,
    d_safe_radius: float,
    d_safe_angles: float,
    verbose: Verbose,
    insertion_constraints: bool = False,
    needle_offset: DQ = DQ([1.0]),
) -> np.ndarray:
    """Construct the needle inequality constraint right-hand side ``w``.

    Mirrors :func:`needle_jacobian` row-for-row: for each constraint row in
    ``W`` this returns the corresponding signed margin in ``w``, so that the
    QP enforces ``W @ q_dot <= w``. All rows are scaled by their VFI gain.

    Args:
        x_needle: Dual-quaternion pose of the needle frame.
        ps_vessel: Vessel positions as pure dual quaternions.
        ns_vessel: Vessel normals as pure dual quaternions, or ``None`` to
            skip orientation constraints.
        needle_radius: Physical needle radius (m).
        vfi_gain_planes: VFI gain for plane-distance constraints.
        vfi_gain_radius: VFI gain for radius constraints.
        vfi_gain_angles: VFI gain for orientation (dot-product) constraints.
        d_safe_planes: Safe plane-distance margin (m).
        d_safe_radius: Safe radius margin (m).
        d_safe_angles: Safe orientation margin (rad).
        verbose: Debug-output setting. ``True`` prints the per-constraint
            margins and violations, ``False`` prints nothing, and a dict
            selects individual categories by name (``"radius"``,
            ``"plane"``, ``"orientation"``, ``"insertion"``).
        insertion_constraints: If ``True`` append the insertion point-to-line
            and angular (phi_z) constraint rows for the first vessel.
        needle_offset: Dual quaternion relating the needle tip to the needle
            frame (defaults to the identity).

    Returns:
        The constraint right-hand-side vector ``w``, shape
        ``(n_constraints, 1)`` (``(0, 1)`` when no constraints are active).

    Raises:
        ValueError: If ``insertion_constraints`` is ``True`` but
            ``ps_vessel`` or ``ns_vessel`` is empty.
    """
    p_needle = translation(x_needle)
    r_needle = rotation(x_needle)
    verbose = normalize_verbose(verbose)
    w_needle = None

    for vessel_index, p_vessel in enumerate(ps_vessel):
        current_radius_squared = float(
            DQ_Geometry.point_to_point_squared_distance(
                p_needle,
                p_vessel,
            )
        )

        lower_radius = max(
            0.0,
            needle_radius - d_safe_radius,
        )
        upper_radius = needle_radius + d_safe_radius

        lower_radius_squared = lower_radius ** 2
        upper_radius_squared = upper_radius ** 2

        radius_error_one = (
                upper_radius_squared - current_radius_squared
        )
        radius_error_two = (
                current_radius_squared - lower_radius_squared
        )

        n_needle = r_needle * k_ * conj(r_needle)
        d_needle = dot(p_needle, n_needle)
        pi_needle = n_needle + E_ * d_needle

        current_plane_distance = float(
            DQ_Geometry.point_to_plane_distance(
                p_vessel,
                pi_needle,
            )
        )

        plane_error_one = (
                d_safe_planes - current_plane_distance
        )
        plane_error_two = (
                current_plane_distance + d_safe_planes
        )

        debug_radius(
            verbose,
            vessel_index,
            lower_radius,
            math.sqrt(current_radius_squared),
            upper_radius,
            radius_error_one,
            radius_error_two,
        )
        debug_plane(
            verbose,
            vessel_index,
            current_plane_distance,
            plane_error_one,
            plane_error_two,
        )

        w = np.array(
            [
                vfi_gain_radius * radius_error_one,
                vfi_gain_radius * radius_error_two,
                2.0 * vfi_gain_planes * plane_error_one,
                2.0 * vfi_gain_planes * plane_error_two,
            ],
            dtype=np.float64,
        ).reshape(-1, 1)
        w_needle = np.vstack((w_needle, w)) if w_needle is not None else w

    if ns_vessel is not None:
        for normal_index, n_vessel in enumerate(ns_vessel):
            # Must match the skipped rows in needle_jacobian().
            if insertion_constraints and normal_index == 0:
                continue

            current_dot = float(dot(n_vessel, Ad(r_needle, k_)).q[0])
            max_dot = math.cos(math.pi / 2.0 - d_safe_angles)
            min_dot = math.cos(math.pi / 2.0 + d_safe_angles)
            dot_error_one = max_dot - current_dot
            dot_error_two = current_dot - min_dot

            debug_orientation(
                verbose,
                normal_index,
                current_dot,
                dot_error_one,
                dot_error_two,
            )

            w = np.array(
                [
                    2.0 * vfi_gain_angles * dot_error_one,
                    2.0 * vfi_gain_angles * dot_error_two,
                ],
                dtype=np.float64,
            ).reshape(-1, 1)

            if np.any(w < 0.0):
                cprint(
                    f"     Constraint violation for dot product: {w}",
                    "red",
                )

            w_needle = np.vstack((w_needle, w)) if w_needle is not None else w

    if insertion_constraints:
        if not ps_vessel or not ns_vessel:
            raise ValueError(
                "ps_vessel and ns_vessel must be non-empty when "
                "insertion constraints are enabled."
            )

        x_needle_tip = x_needle * needle_offset
        n_vessel = ns_vessel[0]
        p_vessel = ps_vessel[0]
        line = n_vessel + E_ * cross(p_vessel, n_vessel)
        plane = n_vessel + E_ * dot(p_vessel, n_vessel)

        insertion_angle = 2.0 * math.pi / 3.0
        insertion_vfi_gain = 1.0
        phi_vfi_gain = 1.0
        phi_min = math.pi / 6.0
        phi_max = math.pi / 2.0
        h_min = 0.0
        h_max = 0.05

        w_insertion = insertion_w(
            line,
            plane,
            insertion_angle,
            x_needle_tip,
            insertion_vfi_gain,
        )

        h = float(
            DQ_Geometry.point_to_plane_distance(
                translation(x_needle_tip), plane
            )
        )
        w_phi = phi_z_constraint_w(
            n_vessel,
            r_needle,
            h,
            phi_min,
            phi_max,
            h_min,
            h_max,
            phi_vfi_gain,
        )

        current_dot = phi_z(n_vessel, r_needle)
        safe_dot = dot_product_safe(
            h, phi_min, phi_max, h_min, h_max
        )
        current_angle = math.acos(
            float(np.clip(current_dot, -1.0, 1.0))
        )
        safe_angle = math.acos(
            float(np.clip(safe_dot, -1.0, 1.0))
        )
        h_n = float(
            np.clip((h - h_min) / (h_max - h_min), 0.0, 1.0)
        )

        debug_insertion(
            verbose,
            0,
            h,
            h_n,
            math.degrees(current_angle),
            math.degrees(safe_angle),
            current_dot,
            safe_dot,
            current_dot - safe_dot,
            w_insertion,
        )

        w = np.array(
            [w_insertion, w_phi],
            dtype=np.float64,
        ).reshape(-1, 1)
        w_needle = np.vstack((w_needle, w)) if w_needle is not None else w

    if w_needle is None:
        return np.empty((0, 1), dtype=np.float64)

    return np.asarray(w_needle, dtype=np.float64)
