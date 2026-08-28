"""Finite-difference validation of the ``_impl`` VFI constraint Jacobians.

Complements :mod:`tests.test_finite_difference` (which checks the raw
``dqrobotics`` distance/plane Jacobians) by checking the **wrapper** Jacobians
defined in :mod:`marinholab.working.needlemanipulation._impl`, i.e. the
functions that build the ``NeedleController`` VFI rows. Each one is validated
by central finite differences against the scalar (or vector) quantity its
docstring/name says it differentiates:

* :func:`_impl.rotation_axis_jacobian` -- ``d/dq Ad(r(q), a)`` (vector valued);
* :func:`_impl.J_phi_z` -- ``d/dq <n, Ad(r(q), k_)>``;
* :func:`_impl.normal_dot_product_jacobian` -- ``d/dq <n, Ad(r(q), a)>``;
* :func:`_impl.J_dot_product_safe` -- ``d/dh cos(phi_safe(h)) * Jh``
  (zero in the saturated regions);
* :func:`_impl.phi_z_constraint_W` -- ``d/dq [dot_product_safe(h(q)) - phi_z(q)]``;
* :func:`_impl.insertion_J_D_safe` -- ``d/dq tan^2(angle) * d_plane(q)^2``;
* :func:`_impl.insertion_W` -- ``d/dq [D(q) - D_safe(q)]`` (point-to-line
  squared minus the safe radius band);
* :func:`_impl.needle_jacobian` -- every stacked row (radius / plane /
  normal / insertion / angular) is the FD gradient of its matching
  constraint scalar, with the correct sign and row order.

Pure-Python ``dqrobotics`` only (no compiled ``_core`` required). A fixed,
well-conditioned 4-DOF arm and off-manifold vessel primitives keep every row
non-degenerate.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from dqrobotics import DQ, Ad, E_, cross, dot, k_, rotation, translation
from dqrobotics.robot_modeling import DQ_Kinematics, DQ_SerialManipulatorDH
from dqrobotics.utils import DQ_Geometry

from marinholab.working.needlemanipulation import _impl

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
_Q0 = np.array([0.5, -0.3, 0.9, 0.2])
_DOF = _Q0.size
_FD_TOL = 2e-6
_EPS = 1e-6

# Depth-dependent angular (phi_z) insertion constraint constants. These mirror
# the values hard-coded in ``_impl.needle_jacobian`` / ``needle_w``; the
# reference finite difference below depends on them, so they must stay in sync.
_INSERTION_ANGLE = 2.0 * math.pi / 3.0
_PHI_MIN = math.pi / 6.0
_PHI_MAX = math.pi / 2.0
_H_MIN = 0.0
_H_MAX = 0.05

# Vessel-0 geometry, built at module scope. ``_impl`` places the angular
# insertion constraint in the linear (non-saturated) region only for
# ``h_min < h < h_max`` where ``h`` is the tip's signed distance to the vessel
# plane. We therefore put the needle tip (the frame position at ``_Q0``)
# exactly at ``h_mid = (h_min + h_max) / 2`` above the plane, plus a lateral
# (in-plane) offset so the tip is NOT on the vessel line. This keeps both the
# point-to-line and plane gradients non-degenerate and, crucially, keeps the
# depth-dependent ``phi_z`` row differentiable.
_ROBOT = DQ_SerialManipulatorDH(_DH_PARAMS)
_TIP0 = translation(_ROBOT.fkm(_Q0))
_N1 = DQ([0.3, 0.7, 0.6]).normalize()
# Unit vector perpendicular to _N1 (Gram-Schmidt of a fixed reference axis).
_M0 = DQ([1.0, 0.0, 0.0])
_M = (_M0 - _N1 * dot(_M0, _N1)).normalize()
_H_MID = (_H_MIN + _H_MAX) / 2.0
_LAT_OFFSET = 0.03
_P1 = _TIP0 - _H_MID * _N1 + _LAT_OFFSET * _M
_N2 = DQ([-0.4, 0.2, 0.9]).normalize()
_P2 = DQ([-0.06, 0.09, 0.04])

# A fixed unit axis for the rotated-axis Jacobian.
_A = DQ([0.3, 0.6, 0.7]).normalize()

# Vessel-0 primitives span the insertion line/plane (same construction as
# ``_impl.needle_jacobian``: ``n + E_ * cross(p, n)`` / ``n + E_ * dot(p, n)``).
_INS_LINE = _N1 + E_ * cross(_P1, _N1)
_INS_PLANE = _N1 + E_ * dot(_P1, _N1)


def _pose(q: np.ndarray, robot) -> tuple[DQ, np.ndarray, np.ndarray, np.ndarray]:
    """Return (x, Jx, Jr, Jt) at joint configuration ``q``."""
    x = robot.fkm(q)
    Jx = robot.pose_jacobian(q)
    Jr = np.asarray(DQ_Kinematics.rotation_jacobian(Jx), dtype=np.float64)
    Jt = np.asarray(
        DQ_Kinematics.translation_jacobian(Jx, x), dtype=np.float64
    )
    return x, Jx, Jr, Jt


def _fd_scalar(fn, q: np.ndarray) -> np.ndarray:
    """Central finite-difference Jacobian of ``fn(q) -> float``."""
    jac = np.zeros(q.size)
    for i in range(q.size):
        qp = q.copy()
        qm = q.copy()
        qp[i] += _EPS
        qm[i] -= _EPS
        jac[i] = (fn(qp) - fn(qm)) / (2.0 * _EPS)
    return jac


def _fd_vector(fn, q: np.ndarray) -> np.ndarray:
    """Central finite-difference Jacobian of ``fn(q) -> (m,)``; returns (m, n)."""
    cols = []
    for i in range(q.size):
        qp = q.copy()
        qm = q.copy()
        qp[i] += _EPS
        qm[i] -= _EPS
        cols.append((np.asarray(fn(qp)) - np.asarray(fn(qm))) / (2.0 * _EPS))
    return np.column_stack(cols)


def _row(jac) -> np.ndarray:
    """Flatten a (possibly (1, n)) Jacobian to a 1-D row."""
    return np.asarray(jac, dtype=float).ravel()


def _h(q: np.ndarray, robot) -> float:
    """Signed distance from the needle tip (== frame, identity offset) to the
    insertion plane -- the depth ``h`` of the angular insertion constraint."""
    return float(
        DQ_Geometry.point_to_plane_distance(translation(robot.fkm(q)), _INS_PLANE)
    )


@pytest.fixture(scope="module")
def robot() -> DQ_SerialManipulatorDH:
    r = DQ_SerialManipulatorDH(_DH_PARAMS)
    r.set_lower_q_limit(np.full(_DOF, -2.0))
    r.set_upper_q_limit(np.full(_DOF, 2.0))
    return r


@pytest.fixture(scope="module")
def pose0(robot):
    x, Jx, Jr, Jt = _pose(_Q0, robot)
    return {"x": x, "Jx": Jx, "Jr": Jr, "Jt": Jt, "r": rotation(x)}


# --- rotated / orientation Jacobians ---------------------------------------


def test_rotation_axis_jacobian(pose0, robot):
    # rotation_axis_jacobian(a, r, Jr) must equal d/dq of Ad(r(q), a)
    # (vector-valued, 4x4). ``a`` is a fixed unit axis.
    x, Jr, r = pose0["x"], pose0["Jr"], pose0["r"]
    closed = _impl.rotation_axis_jacobian(_A, r, Jr)
    fd = _fd_vector(lambda qq: Ad(rotation(robot.fkm(qq)), _A).vec4(), _Q0)
    assert np.asarray(closed).shape == fd.shape == (_DOF, _DOF)
    assert np.max(np.abs(np.asarray(closed) - fd)) < _FD_TOL


def test_J_phi_z(pose0, robot):
    # J_phi_z(n, r, Jr) must equal d/dq of phi_z = <n, Ad(r(q), k_)>.
    Jr, r = pose0["Jr"], pose0["r"]
    closed = _row(_impl.J_phi_z(_N1, r, Jr))
    fd = _fd_scalar(
        lambda qq: _impl.phi_z(_N1, rotation(robot.fkm(qq))), _Q0
    )
    assert closed.shape == fd.shape == (_DOF,)
    assert np.max(np.abs(closed - fd)) < _FD_TOL


def test_normal_dot_product_jacobian(pose0, robot):
    # Must equal d/dq of <n, Ad(r(q), a)> for a fixed unit axis a.
    Jr, r = pose0["Jr"], pose0["r"]
    closed = _row(_impl.normal_dot_product_jacobian(_N2, _A, r, Jr))
    fd = _fd_scalar(
        lambda qq: float(dot(_N2, Ad(rotation(robot.fkm(qq)), _A)).q[0]), _Q0
    )
    assert closed.shape == fd.shape == (_DOF,)
    assert np.max(np.abs(closed - fd)) < _FD_TOL


# --- safe-angle (phi_safe) chain rule --------------------------------------


def test_J_dot_product_safe_chain_rule():
    # Interior: J_dot_product_safe == d/dh[dot_product_safe(h)] * Jh.
    h = 0.025
    Jh = np.arange(4.0)
    dh = _fd_scalar(
        lambda qq: _impl.dot_product_safe(qq[0], _PHI_MIN, _PHI_MAX, _H_MIN, _H_MAX),
        np.array([h]),
    )[0]
    closed = _row(_impl.J_dot_product_safe(h, Jh, _PHI_MIN, _PHI_MAX, _H_MIN, _H_MAX))
    assert closed.shape == Jh.shape == (4,)
    assert np.max(np.abs(closed - dh * Jh)) < 1e-7


def test_J_dot_product_safe_saturated_regions():
    # Outside [h_min, h_max] the safe angle is constant, so the Jacobian is 0.
    Jh = np.arange(4.0)
    assert np.allclose(
        _impl.J_dot_product_safe(_H_MAX + 1e-9, Jh, _PHI_MIN, _PHI_MAX, _H_MIN, _H_MAX),
        0.0,
    )
    assert np.allclose(
        _impl.J_dot_product_safe(_H_MIN - 1e-9, Jh, _PHI_MIN, _PHI_MAX, _H_MIN, _H_MAX),
        0.0,
    )


# --- angular insertion constraint ------------------------------------------


def test_phi_z_constraint_W(pose0, robot):
    # phi_z_constraint_W == d/dq [dot_product_safe(h(q)) - phi_z(q)], where
    # h(q) is the tip's signed distance to the insertion plane.
    Jr, r = pose0["Jr"], pose0["r"]
    Jx, Jt = pose0["Jx"], pose0["Jt"]
    h = _h(_Q0, robot)
    Jh = np.asarray(
        DQ_Kinematics.point_to_plane_distance_jacobian(Jt, translation(pose0["x"]), _INS_PLANE),
        dtype=np.float64,
    )
    closed = _row(
        _impl.phi_z_constraint_W(
            _N1, r, Jr, h, Jh, _PHI_MIN, _PHI_MAX, _H_MIN, _H_MAX
        )
    )
    fd = _fd_scalar(
        lambda qq: (
            _impl.dot_product_safe(_h(qq, robot), _PHI_MIN, _PHI_MAX, _H_MIN, _H_MAX)
            - _impl.phi_z(_N1, rotation(robot.fkm(qq)))
        ),
        _Q0,
    )
    assert closed.shape == fd.shape == (_DOF,)
    assert np.max(np.abs(closed - fd)) < _FD_TOL
    # h must lie in the linear (non-saturated) region for the FD to match.
    assert _H_MIN < h < _H_MAX


# --- point-to-line insertion constraint ------------------------------------


def test_insertion_J_D_safe(pose0, robot):
    # insertion_J_D_safe == d/dq of tan^2(angle) * d_plane(q)^2.
    Jt, x = pose0["Jt"], pose0["x"]
    closed = _row(_impl.insertion_J_D_safe(Jt, _INSERTION_ANGLE, _INS_PLANE, x))
    fd = _fd_scalar(
        lambda qq: _impl.insertion_D_safe(
            _INSERTION_ANGLE, _INS_PLANE, robot.fkm(qq)
        ),
        _Q0,
    )
    assert closed.shape == fd.shape == (_DOF,)
    assert np.max(np.abs(closed - fd)) < _FD_TOL


def test_insertion_W(pose0, robot):
    # insertion_W == d/dq of [D(q) - D_safe(q)] (squared point-to-line
    # distance minus the safe radius band).
    Jt, x = pose0["Jt"], pose0["x"]
    closed = _row(_impl.insertion_W(Jt, _INS_LINE, _INSERTION_ANGLE, _INS_PLANE, x))
    fd = _fd_scalar(
        lambda qq: (
            _impl.insertion_D(_INS_LINE, robot.fkm(qq))
            - _impl.insertion_D_safe(_INSERTION_ANGLE, _INS_PLANE, robot.fkm(qq))
        ),
        _Q0,
    )
    assert closed.shape == fd.shape == (_DOF,)
    assert np.max(np.abs(closed - fd)) < _FD_TOL


# --- composite needle_jacobian ---------------------------------------------


def _plane_of_needle(q: np.ndarray, robot) -> DQ:
    """The needle's own plane (unit k_ through the tip), as in ``needle_w``."""
    x = robot.fkm(q)
    n = Ad(rotation(x), k_)
    t = translation(x)
    return n + E_ * dot(t, n)


def test_needle_jacobian_radius_plane_rows(pose0, robot):
    # With two vessels and no normals (``ns_vessel=None``), ``W`` has exactly
    # the 8 base rows -- per vessel v: [+J(radius^2), -J(radius^2),
    # +J(plane d), -J(plane d)] -- matching the row order in ``needle_w``.
    ps = [_P1, _P2]
    W = _impl.needle_jacobian(pose0["Jx"], pose0["x"], ps, None)
    assert W.shape == (8, _DOF)

    ref = []
    for p_v in ps:
        j_rad = _fd_scalar(
            lambda qq, p=p_v: float(
                DQ_Geometry.point_to_point_squared_distance(
                    translation(robot.fkm(qq)), p
                )
            ),
            _Q0,
        )
        j_pl = _fd_scalar(
            lambda qq, p=p_v: float(
                DQ_Geometry.point_to_plane_distance(p, _plane_of_needle(qq, robot))
            ),
            _Q0,
        )
        ref += [j_rad, -j_rad, j_pl, -j_pl]
    ref = np.vstack(ref)
    assert np.max(np.abs(W - ref)) < _FD_TOL


def test_needle_jacobian_normal_rows(pose0, robot):
    # With one normal (no insertion), the orientation rows are
    # [+J(<n, Ad(r, k_)>), -J(<n, Ad(r, k_)>)] for that normal.
    ps = [_P1]
    ns = [_N2]
    W = _impl.needle_jacobian(pose0["Jx"], pose0["x"], ps, ns)
    assert W.shape == (6, _DOF)
    j_norm = _fd_scalar(
        lambda qq: float(dot(_N2, Ad(rotation(robot.fkm(qq)), k_)).q[0]), _Q0
    )
    assert np.max(np.abs(W[4] - j_norm)) < _FD_TOL
    assert np.max(np.abs(W[5] - (-j_norm))) < _FD_TOL


def test_needle_jacobian_insertion_rows(pose0, robot):
    # With insertion enabled, the first normal's row is replaced by the
    # point-to-line and angular (phi_z) insertion rows. Identity offset means
    # the tip equals the frame, so these rows match ``insertion_W`` /
    # ``phi_z_constraint_W`` evaluated at the frame pose.
    ps = [_P1]
    ns = [_N1]
    W = _impl.needle_jacobian(
        pose0["Jx"], pose0["x"], ps, ns, insertion_constraints=True
    )
    assert W.shape == (6, _DOF)

    # Row 4: d/dq [D(q) - D_safe(q)].
    fd_ins = _fd_scalar(
        lambda qq: (
            _impl.insertion_D(_INS_LINE, robot.fkm(qq))
            - _impl.insertion_D_safe(_INSERTION_ANGLE, _INS_PLANE, robot.fkm(qq))
        ),
        _Q0,
    )
    assert np.max(np.abs(W[4] - fd_ins)) < _FD_TOL

    # Row 5: d/dq [dot_product_safe(h(q)) - phi_z(q)].
    fd_phi = _fd_scalar(
        lambda qq: (
            _impl.dot_product_safe(_h(qq, robot), _PHI_MIN, _PHI_MAX, _H_MIN, _H_MAX)
            - _impl.phi_z(_N1, rotation(robot.fkm(qq)))
        ),
        _Q0,
    )
    assert np.max(np.abs(W[5] - fd_phi)) < _FD_TOL

    # The radius/plane rows (0..3) still match the (unsigned) base rows.
    j_rad = _fd_scalar(
        lambda qq: float(
            DQ_Geometry.point_to_point_squared_distance(
                translation(robot.fkm(qq)), _P1
            )
        ),
        _Q0,
    )
    j_pl = _fd_scalar(
        lambda qq: float(
            DQ_Geometry.point_to_plane_distance(_P1, _plane_of_needle(qq, robot))
        ),
        _Q0,
    )
    assert np.max(np.abs(W[0] - j_rad)) < _FD_TOL
    assert np.max(np.abs(W[1] - (-j_rad))) < _FD_TOL
    assert np.max(np.abs(W[2] - j_pl)) < _FD_TOL
    assert np.max(np.abs(W[3] - (-j_pl))) < _FD_TOL
