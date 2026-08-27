"""Regression tests for the bug fixes in this change.

These target the three concrete bugs:

1. The RCM constraint margin (right-hand side) being dropped when the RCM row
   is stacked into the full QP — it must equal the margin from
   ``get_rcm_constraint``.
2. ``M3_SerialManipulatorSimulatorFriendly`` default joint limits being
   uninitialized (garbage) memory — they must be finite and ordered.
3. ``example_plot`` reading a ``"diameter"`` key that the loader never
   produces — it must read ``"radius"``.

The ``_core``-dependent tests are skipped when the compiled extension is not
available (see :mod:`tests.conftest`).
"""
from __future__ import annotations

import numpy as np
import pytest

from dqrobotics import DQ, k_
from marinholab.working.needlemanipulation import (
    M3_SerialManipulatorSimulatorFriendly,
    ICRA19TaskSpaceController,
)
from marinholab.working.needlemanipulation.example_load_from_file import (
    get_information_from_file,
    example_plot,
)

# The RCM joint index and the example's documented joint limits (radians).
_RC_IDX = 6
_LOWER = [-85, -85, 5, -265, -85, -355, -170, -30, -30]
_UPPER = [85, 85, 120, 0, 85, 355, 170, 30, 30]


@pytest.fixture(scope="module")
def loaded():
    """(robot, rcm1, rcm2) built from the bundled left_robot.yaml."""
    from importlib.resources import files

    return get_information_from_file(
        files("marinholab.working.needlemanipulation")
        .joinpath("left_robot.yaml")
        .read_text()
    )


def test_rcm_margin_is_stacked_into_qp_rhs(loaded):
    robot, r1, _ = loaded
    robot.set_lower_q_limit(list(_LOWER))
    robot.set_upper_q_limit(list(_UPPER))

    rcm = [(r1["position"], r1["radius"], _RC_IDX)]
    ctrl = ICRA19TaskSpaceController(
        robot, gain=10.0, damping=0.01, alpha=0.999, rcm_constraints=rcm
    )

    q = np.zeros(9)
    xd = robot.fkm(q)
    _, _, W, w = ctrl._get_optimization_parameters(q, xd)

    # The RCM row is the row after the 2*DOF joint-limit rows.
    dof = len(q)
    rcm_rhs = w[2 * dof]
    Jx_idx = robot.pose_jacobian(q, _RC_IDX)
    x_idx = robot.fkm(q, _RC_IDX)
    _, w_ref = ctrl.get_rcm_constraint(
        Jx_idx, x_idx, k_, r1["position"], r1["radius"], ctrl.vfi_gain
    )

    # The stacked right-hand side must carry the VFI margin (item 1).
    assert np.isclose(rcm_rhs, w_ref[0])
    assert abs(w_ref[0]) > 1e-9  # a real (non-void) bound, not a dropped zero


def test_m3_default_limits_are_finite_and_ordered(loaded):
    robot, _, _ = loaded
    lo = np.asarray(robot.get_lower_q_limit(), dtype=float)
    up = np.asarray(robot.get_upper_q_limit(), dtype=float)

    assert lo.shape == up.shape == (9,)
    assert np.all(np.isfinite(lo)) and np.all(np.isfinite(up)), (
        "default joint limits are not finite (uninitialized memory?)"
    )
    assert np.all(lo < up), "default joint limits are not ordered"


def test_m3_default_limits_via_direct_construction(core_available):
    # Also exercise the C++ constructor directly (bypasses the YAML loader).
    pytest.importorskip("marinholab.working.needlemanipulation._core")
    if not core_available:
        pytest.skip("compiled _core extension not available")

    n = 5
    rx = M3_SerialManipulatorSimulatorFriendly.ActuationType.RX
    ob = [DQ([1.0, 0.0, 0.0, 0.0]) for _ in range(n)]
    oa = [DQ([1.0, 0.0, 0.0, 0.0]) for _ in range(n)]
    robot = M3_SerialManipulatorSimulatorFriendly(ob, oa, [rx] * n)

    lo = np.asarray(robot.get_lower_q_limit(), dtype=float)
    up = np.asarray(robot.get_upper_q_limit(), dtype=float)
    assert lo.shape == up.shape == (n,)
    assert np.all(np.isfinite(lo)) and np.all(np.isfinite(up))
    assert np.all(lo < up)


def test_example_plot_reads_radius_key(loaded):
    robot, r1, r2 = loaded
    # The loader only produces "position" and "radius"; example_plot must not
    # reference a missing "diameter" key (item 4).
    assert set(r1.keys()) == {"position", "radius"}
    assert set(r2.keys()) == {"position", "radius"}
    assert "diameter" not in r1 and "diameter" not in r2
    # The attribute is reachable on the module (no NameError / ImportError).
    assert callable(example_plot)


def test_example_plot_source_uses_radius_key():
    import inspect

    src = inspect.getsource(example_plot)
    assert "radius" in src
    assert "diameter" not in src
