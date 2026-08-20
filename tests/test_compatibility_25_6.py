"""
Regression tests for PyPI 25.6.0.66 compatibility issues.

Covers:
1. The package root re-exports the dqrobotics namespace (25.6.0.66 did
   ``from dqrobotics import *``; the PEP 561 cleanup accidentally removed it).
2. ``NeedleController.compute_setpoint_control_signal`` no longer raises
   ``TypeError: needle_w() missing 1 required positional argument:
   'vfi_gain_needle_insertion_angles'``.
3. ``needle_w`` no longer crashes with an ``np.vstack`` shape mismatch when
   angle constraints are active together with vessel plane/radius constraints.
4. ``icra2019_controller`` uses the concrete ``DQ_QuadprogSolver`` (25.6.0.66
   did; a later change aliased the abstract ``DQ_QuadraticProgrammingSolver``
   under its name, which makes ``solve_quadratic_program`` unusable).
5. ``example_kinematics_from_coppeliasim`` imports without the CoppeliaSim
   interface (not shipped in released dqrobotics wheels) and only fails inside
   ``main()`` with an actionable message.
6. ``requires-python`` matches the oldest Python that has a dqrobotics build
   providing the submodules used by this package (3.10).
"""
import importlib.util
import os
import re

import numpy as np
import pytest

# conftest.py mocks marinholab.working.needlemanipulation._core at configure
# time, so the package (and its submodules) can be imported without the
# compiled extension.
import marinholab.working.needlemanipulation as nm
from marinholab.working.needlemanipulation import icra2019_controller, needle_controller
from dqrobotics import DQ


def _load_impl_module():
    """Load _impl.py directly, bypassing the package __init__.py."""
    spec = importlib.util.spec_from_file_location(
        "_impl_compat",
        os.path.join(os.path.dirname(__file__), '..',
                     'marinholab', 'working', 'needlemanipulation', '_impl.py'),
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


impl = _load_impl_module()

# Shared valid DQ fixtures (see test_impl.py for the same convention).
_X_NEEDLE = impl.DQ([1, 0, 0, 0, 0.1, 0.2, 0.3, 0.0]).normalize()
_PS_VESSEL = [0.5 * impl.k_]
_NS_VESSEL = [impl.k_]


class _FakeKinematics:
    """Minimal kinematics stand-in: identity pose, zero Jacobian, +/-1 limits."""

    DOF = 3

    def fkm(self, q, idx=-1):
        return DQ([1, 0, 0, 0, 0, 0, 0, 0])

    def pose_jacobian(self, q, idx=-1):
        return np.zeros((8, self.DOF))

    def get_lower_q_limit(self):
        return np.full(self.DOF, -1.0)

    def get_upper_q_limit(self):
        return np.full(self.DOF, 1.0)


@pytest.fixture
def q():
    return [0.0, 0.0, 0.0]


class TestDqroboticsNamespaceReexport:
    """25.6.0.66 exposed every dqrobotics symbol at the package root."""

    @pytest.mark.parametrize("name", ["DQ", "k_", "conj", "haminus8", "vec8", "is_unit", "E_"])
    def test_symbol_available(self, name):
        assert hasattr(nm, name), f"{name} missing from package root"

    @pytest.mark.parametrize("name", ["DQ", "k_", "conj", "haminus8"])
    def test_symbol_in_all(self, name):
        assert name in nm.__all__, f"{name} missing from __all__"

    def test_core_names_still_exported(self):
        for name in ("M3_SerialManipulatorSimulatorFriendly", "needle_jacobian",
                     "needle_w", "ICRA19TaskSpaceController", "NeedleController"):
            assert name in nm.__all__


class TestNeedleControllerNeedleWCall:
    """compute_setpoint_control_signal must call needle_w with the full signature."""

    def test_no_typeerror_when_computing_signal(self, q):
        c = nm.NeedleController(
            kinematics=_FakeKinematics(),
            gain=10.0,
            damping=0.01,
            alpha=0.999,
            rcm_constraints=None,
            relative_needle_pose=DQ([1]),
            vessel_positions=_PS_VESSEL,
            needle_radius=0.003,
            vfi_gain=2.0,
        )
        # This raised TypeError (missing vfi_gain_needle_insertion_angles)
        # before the fix.
        u = c.compute_setpoint_control_signal(q, c.kinematics.fkm(q))
        assert u is not None
        assert len(u) == _FakeKinematics.DOF

    def test_needle_w_accepts_controller_arguments(self):
        # Directly exercise the exact keyword set the controller passes, to
        # catch future required-positional regressions.
        w = impl.needle_w(
            x_needle=_X_NEEDLE,
            ps_vessel=_PS_VESSEL,
            ns_vessel=None,
            needle_radius=0.003,
            vfi_gain_planes=2.0,
            vfi_gain_radius=2.0,
            vfi_gain_angles=2.0,
            vfi_gain_needle_insertion_angles=2.0,
            d_safe_planes=None,
            d_safe_radius=None,
            d_safe_angles=None,
            d_safe_needle_insertion_angles=None,
            verbose=False,
        )
        assert w is None  # no active constraints


class TestNeedleWCombinedConstraints:
    """Angle constraints must stack cleanly with vessel plane/radius constraints."""

    def _call(self, **overrides):
        params = dict(
            x_needle=_X_NEEDLE,
            ps_vessel=_PS_VESSEL,
            ns_vessel=_NS_VESSEL,
            needle_radius=0.003,
            vfi_gain_planes=2.0,
            vfi_gain_radius=2.0,
            vfi_gain_angles=2.0,
            vfi_gain_needle_insertion_angles=2.0,
            d_safe_planes=0.01,
            d_safe_radius=0.01,
            d_safe_angles=0.5,
            d_safe_needle_insertion_angles=0.5,
            verbose=False,
        )
        params.update(overrides)
        return impl.needle_w(**params)

    def test_all_constraint_families_stack(self):
        # Previously raised:
        # ValueError: all the input array dimensions except for the
        # concatenation axis must match exactly
        w = self._call()
        assert w is not None
        # 4 constraint pairs (plane + radius + insertion angle + driving angle),
        # each contributing 2 rows (upper/lower).
        assert w.size == 8
        # The reshape the controller performs must be consistent with the
        # matching Jacobian's row count.
        W = impl.needle_jacobian(np.zeros((8, 3)), _X_NEEDLE, _PS_VESSEL, _NS_VESSEL,
                                 planes_active=True, spheres_active=True,
                                 driving_angle_active=True, insertion_angle_active=True,
                                 needle_radius=0.003)
        assert w.size == W.shape[0]
        assert w.reshape((W.shape[0],)).shape == (W.shape[0],)

    def test_angle_plus_planes(self):
        w = self._call(d_safe_radius=None)
        # 3 pairs (plane + insertion angle + driving angle)
        assert w.size == 6

    def test_angle_plus_radius(self):
        w = self._call(d_safe_planes=None, d_safe_needle_insertion_angles=None)
        # 2 pairs (radius + driving angle)
        assert w.size == 4


class TestConcreteQuadprogSolver:
    """The controller must use the concrete DQ_QuadprogSolver."""

    def test_not_abstract_base(self):
        from dqrobotics.solvers import DQ_QuadraticProgrammingSolver
        assert icra2019_controller.DQ_QuadprogSolver is not DQ_QuadraticProgrammingSolver

    def test_solve_works_end_to_end(self, q):
        # With the abstract class aliased as DQ_QuadprogSolver this raised
        # "Tried to call pure virtual function" / a pybind11 TypeError.
        c = nm.ICRA19TaskSpaceController(
            kinematics=_FakeKinematics(),
            gain=10.0,
            damping=0.01,
            alpha=0.999,
            rcm_constraints=None,
        )
        u = c.compute_setpoint_control_signal(q, c.kinematics.fkm(q))
        assert u is not None
        assert len(u) == _FakeKinematics.DOF


class TestCoppeliaSimExampleGuard:
    """example_kinematics_from_coppeliasim must import even without the interface."""

    @pytest.fixture
    def example_module(self):
        spec = importlib.util.spec_from_file_location(
            "example_kinematics_from_coppeliasim_compat",
            os.path.join(os.path.dirname(__file__), '..',
                         'marinholab', 'working', 'needlemanipulation',
                         'example_kinematics_from_coppeliasim.py'),
        )
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod

    def test_module_imports_without_interface(self, example_module):
        # dqrobotics releases (checked on linux x86_64/aarch64 and win_amd64)
        # do not compile the CoppeliaSim interface, so this must not raise.
        assert example_module.DQ_CoppeliaSimInterfaceZMQ is None or callable(
            example_module.DQ_CoppeliaSimInterfaceZMQ)

    def test_main_gives_actionable_error_when_unavailable(self, example_module):
        if example_module.DQ_CoppeliaSimInterfaceZMQ is None:
            with pytest.raises(ImportError, match="CoppeliaSim"):
                example_module.main()
        else:
            pytest.skip("CoppeliaSim interface available in this environment")


class TestRequiresPython:
    """requires-python must not promise versions no dqrobotics build supports."""

    def test_lower_bound_is_310_or_newer(self):
        here = os.path.join(os.path.dirname(__file__), '..', 'pyproject.toml')
        with open(here, "r") as f:
            content = f.read()
        m = re.search(r'requires-python\s*=\s*">=\s*(\d+)\.(\d+)', content)
        assert m is not None, "requires-python not found in pyproject.toml"
        # dqrobotics dropped cp39 wheels in the release line that provides the
        # submodules used here (robot_control, solvers, interfaces), so the
        # lower bound must be at least 3.10.
        major, minor = int(m.group(1)), int(m.group(2))
        assert (major, minor) >= (3, 10), "requires-python too permissive (needs >=3.10)"
