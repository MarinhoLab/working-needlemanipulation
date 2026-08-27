"""Finite-difference validation of the dual-quaternion task Jacobians.

These tests exercise the pure-Python ``dqrobotics`` kinematics (no compiled
``_core`` required) and confirm, by central finite differences, that each
distance / plane Jacobian used by the needlemanipulation controllers
differentiates the quantity its name and sign convention imply:

* point-to-point and line-based distances use the **squared** distance
  (matching the RCM / vessel geometry built from ``*_squared_distance``);
* plane-based constraints use the **signed plain** distance.

A fixed, well-conditioned 4-DOF arm and a handful of off-manifold fixed
primitives are used so every Jacobian row is non-degenerate.
"""
from __future__ import annotations

import numpy as np
import pytest

from dqrobotics import DQ, Ad, E_, cross, dot, haminus8, k_, rotation, translation
from dqrobotics.robot_modeling import DQ_Kinematics, DQ_SerialManipulatorDH
from dqrobotics.utils import DQ_Geometry

# 5x4 DH parameter matrix (a, alpha, d, theta, joint_type=1) for a 4-DOF arm.
_DH_PARAMS = np.array(
    [
        [0.0, 0.3, 0.0, 0.35],
        [0.0, 0.0, 1.5707963267948966, 1.5707963267948966],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [1, 1, 1, 1],
    ]
)
_Q0 = np.array([0.6, -0.4, 1.1, 0.25])
_DOF = _Q0.size
_FD_TOL = 2e-6
_EPS = 1e-6

# Fixed primitives, placed off the arm's reach so the distances are
# non-degenerate. A point is a pure quaternion; a line is a unit axis plus a
# perpendicular moment; a plane is a unit normal plus a signed offset.
_P_POINT = DQ([1.0, 0.15, -0.2])
_LINE_AXIS = DQ([0.2, 0.1, 0.0]).normalize()
_LINE_T0 = DQ([0.0, 0.05, 0.06])
_P_LINE = _LINE_AXIS + E_ * cross(_LINE_T0, _LINE_AXIS)
_P_PLANE_N = DQ([0.3, 0.4, 0.3]).normalize()
_P_PLANE_P = DQ([0.1, -0.05, 0.15])
_P_PLANE = _P_PLANE_N + E_ * dot(_P_PLANE_P, _P_PLANE_N)


def _finite_difference(scalar_fn, q: np.ndarray, eps: float = _EPS) -> np.ndarray:
    """Central finite-difference Jacobian of ``scalar_fn(q) -> float``."""
    jac = np.zeros(q.size)
    for i in range(q.size):
        qp = q.copy()
        qm = q.copy()
        qp[i] += eps
        qm[i] -= eps
        jac[i] = (scalar_fn(qp) - scalar_fn(qm)) / (2.0 * eps)
    return jac


def _row(jac) -> np.ndarray:
    """Flatten a Jacobian returned by ``DQ_Kinematics`` to a 1-D row."""
    return np.asarray(jac, dtype=float).ravel()


def _frame_plane(q: np.ndarray, robot, idx=None) -> DQ:
    """Plane attached to a moving frame: unit normal + signed offset."""
    xx = robot.fkm(q) if idx is None else robot.fkm(q, idx)
    n = Ad(rotation(xx), k_)
    t = translation(xx)
    return n + E_ * dot(t, n)


def _frame_line(q: np.ndarray, robot, idx=None) -> DQ:
    """Line attached to a moving frame: the frame's z-axis through its origin."""
    xx = robot.fkm(q) if idx is None else robot.fkm(q, idx)
    axis = Ad(rotation(xx), k_)
    t = translation(xx)
    return axis + E_ * cross(t, axis)


@pytest.fixture(scope="module")
def robot() -> DQ_SerialManipulatorDH:
    r = DQ_SerialManipulatorDH(_DH_PARAMS)
    r.set_lower_q_limit(np.full(_DOF, -2.0))
    r.set_upper_q_limit(np.full(_DOF, 2.0))
    return r


@pytest.fixture(scope="module")
def base(robot) -> dict:
    x = robot.fkm(_Q0)
    Jx = robot.pose_jacobian(_Q0)
    return {
        "x": x,
        "Jx": Jx,
        "Jt": DQ_Kinematics.translation_jacobian(Jx, x),
    }


def test_point_to_point_squared_distance_jacobian(base, robot):
    x, Jt = base["x"], base["Jt"]
    closed = _row(
        DQ_Kinematics.point_to_point_distance_jacobian(Jt, translation(x), _P_POINT)
    )
    fd = _finite_difference(
        lambda qq: DQ_Geometry.point_to_point_squared_distance(
            translation(robot.fkm(qq)), _P_POINT
        )
    , _Q0)
    assert closed.shape == fd.shape == (_DOF,)
    assert np.max(np.abs(closed - fd)) < _FD_TOL


def test_point_to_line_squared_distance_jacobian(base, robot):
    x, Jt = base["x"], base["Jt"]
    closed = _row(
        DQ_Kinematics.point_to_line_distance_jacobian(Jt, translation(x), _P_LINE)
    )
    fd = _finite_difference(
        lambda qq: DQ_Geometry.point_to_line_squared_distance(
            translation(robot.fkm(qq)), _P_LINE
        )
    , _Q0)
    assert closed.shape == fd.shape == (_DOF,)
    assert np.max(np.abs(closed - fd)) < _FD_TOL


def test_line_to_point_squared_distance_jacobian_moving_line(base, robot):
    # The RCM configuration: the line is attached to the frame and moves with
    # the robot, while the point is fixed.
    x, Jx = base["x"], base["Jx"]
    Jl = DQ_Kinematics.line_jacobian(Jx, x, k_)
    line0 = _frame_line(_Q0, robot)
    closed = _row(
        DQ_Kinematics.line_to_point_distance_jacobian(Jl, line0, _P_POINT)
    )
    fd = _finite_difference(
        lambda qq: DQ_Geometry.point_to_line_squared_distance(
            _P_POINT, _frame_line(qq, robot)
        )
    , _Q0)
    assert closed.shape == fd.shape == (_DOF,)
    assert np.max(np.abs(closed - fd)) < _FD_TOL


def test_point_to_plane_signed_distance_jacobian(base, robot):
    x, Jt = base["x"], base["Jt"]
    closed = _row(
        DQ_Kinematics.point_to_plane_distance_jacobian(Jt, translation(x), _P_PLANE)
    )
    fd = _finite_difference(
        lambda qq: DQ_Geometry.point_to_plane_distance(
            translation(robot.fkm(qq)), _P_PLANE
        )
    , _Q0)
    assert closed.shape == fd.shape == (_DOF,)
    assert np.max(np.abs(closed - fd)) < _FD_TOL


def test_plane_to_point_signed_distance_jacobian_moving_plane(base, robot):
    # The plane is attached to the frame (moves with the robot); the point is
    # fixed. This mirrors the vessel-plane constraint in _impl.py.
    x, Jx = base["x"], base["Jx"]
    Jpi = DQ_Kinematics.plane_jacobian(Jx, x, k_)
    closed = _row(
        DQ_Kinematics.plane_to_point_distance_jacobian(Jpi, _P_POINT)
    )
    fd = _finite_difference(
        lambda qq: DQ_Geometry.point_to_plane_distance(_P_POINT, _frame_plane(qq, robot))
    , _Q0)
    assert closed.shape == fd.shape == (_DOF,)
    assert np.max(np.abs(closed - fd)) < _FD_TOL


def test_haminus8_tool_frame_jacobian_transform(robot):
    # With a fixed tool offset A, the pose Jacobian of the offset frame must
    # equal haminus8(A) @ J of the base frame (the transform used to offset
    # the needle tip). Verified to machine precision.
    robot.set_effector(DQ([1.0]))
    x_id = robot.fkm(_Q0)
    J_id = robot.pose_jacobian(_Q0)
    A = DQ([1.0]) + E_ * DQ([0.2, 0.1, 0.05, 0.0])
    robot.set_effector(A)
    x_off = robot.fkm(_Q0)
    J_off = robot.pose_jacobian(_Q0)
    robot.set_effector(DQ([1.0]))

    # Pose consistency: x_off == x_id * A.
    assert np.allclose(
        np.asarray(x_off.vec4()), np.asarray((x_id * A).vec4()), atol=1e-12
    )
    # Jacobian transform consistency.
    err = np.max(np.abs(np.asarray(J_off) - np.asarray(haminus8(A) @ J_id)))
    assert err < 1e-9


def test_get_rcm_constraint_matches_geometry(robot):
    # Cross-check the controller's RCM margin against the closed-form geometry
    # so a future change to get_rcm_constraint cannot silently desync the QP
    # right-hand side from the VFI margin.
    from marinholab.working.needlemanipulation.icra2019_controller import (
        ICRA19TaskSpaceController,
    )

    idx = 2
    Jx_idx = robot.pose_jacobian(_Q0, idx)
    x_idx = robot.fkm(_Q0, idx)
    r_safe = 0.5
    eta = 2.0
    _, w = ICRA19TaskSpaceController.get_rcm_constraint(
        Jx_idx, x_idx, k_, _P_POINT, r_safe, eta
    )
    # get_rcm_constraint spans the line along the primitive (k_) through the
    # frame origin, which is exactly _frame_line(..., idx) with the z-axis.
    line = _frame_line(_Q0, robot, idx)
    Dl_p = DQ_Geometry.point_to_line_squared_distance(_P_POINT, line)
    assert np.isclose(w[0], eta * (r_safe ** 2 - Dl_p))
    # With this geometry the margin is non-zero, i.e. a real (non-void) bound.
    assert abs(w[0]) > 1e-6
