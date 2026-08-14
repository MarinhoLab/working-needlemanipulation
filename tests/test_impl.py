"""
Tests for _impl.py functions: needle_jacobian, needle_w, rotation_axis_jacobian, normal_dot_product_jacobian.
These functions depend only on dqrobotics (no C++ extension).
"""
import sys
import os
import importlib.util
import numpy as np
import pytest


def _load_impl_module():
    """Load _impl.py directly, bypassing the package __init__.py which requires _core."""
    spec = importlib.util.spec_from_file_location(
        "_impl",
        os.path.join(os.path.dirname(__file__), '..',
                     'marinholab', 'working', 'needlemanipulation', '_impl.py')
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


impl = _load_impl_module()


class TestRotationAxisJacobian:
    """Tests for rotation_axis_jacobian."""

    def test_returns_array(self):
        primitive = impl.k_
        r = impl.DQ([1, 0, 0, 0, 0, 0, 0, 0])
        Jr = np.random.rand(4, 6)
        result = impl.rotation_axis_jacobian(primitive, r, Jr)
        assert isinstance(result, np.ndarray)

    def test_output_shape(self):
        primitive = impl.k_
        r = impl.DQ([1, 0, 0, 0, 0, 0, 0, 0])
        Jr = np.random.rand(4, 9)
        result = impl.rotation_axis_jacobian(primitive, r, Jr)
        # haminus4 is 4x4, so output is 4 rows × dof columns
        assert result.shape[0] == 4
        assert result.shape[1] == 9


class TestNormalDotProductJacobian:
    """Tests for normal_dot_product_jacobian."""

    def test_returns_array(self):
        normal = impl.k_
        primitive = impl.k_
        r = impl.DQ([1, 0, 0, 0, 0, 0, 0, 0])
        Jr = np.random.rand(4, 6)
        result = impl.normal_dot_product_jacobian(normal, primitive, r, Jr)
        assert isinstance(result, np.ndarray)

    def test_output_shape(self):
        normal = impl.k_
        primitive = impl.k_
        r = impl.DQ([1, 0, 0, 0, 0, 0, 0, 0])
        Jr = np.random.rand(4, 9)
        result = impl.normal_dot_product_jacobian(normal, primitive, r, Jr)
        # vec4(normal) is (4,) and haminus4 result is (4, 9), so 1D @ 2D = 1D(9)
        assert result.shape[0] == 9
        # It's a 1D array, not 2D
        assert result.ndim == 1


class TestConeJacobian:
    """Tests for cone_jacobian (placeholder function)."""

    def test_returns_none(self):
        result = impl.cone_jacobian()
        assert result is None


class TestNeedleJacobian:
    """Tests for needle_jacobian function."""

    def _make_jacobian(self, dof=9):
        """Create a dummy Jacobian matrix."""
        return np.random.rand(8, dof)

    def _make_needle_pose(self):
        """Create a valid needle pose DQ."""
        x = impl.DQ([1, 0, 0, 0, 0.1, 0.2, 0.3, 0.0])
        return x.normalize()

    def _make_vessel_positions(self):
        """Create vessel positions as pure quaternions (using k_ axis)."""
        return [0.5 * impl.k_]

    def _make_vessel_positions_multi(self):
        """Create multiple vessel positions."""
        return [0.5 * impl.k_, 0.6 * impl.k_ + 0.1 * impl.i_]

    def test_spheres_active(self):
        Jx = self._make_jacobian()
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions()

        W = impl.needle_jacobian(
            Jx, x_needle, ps_vessel, [],
            planes_active=False, spheres_active=True,
            driving_angle_active=False, insertion_angle_active=False,
        )
        assert W is not None
        # Each sphere produces 2 constraints (radius and -radius)
        assert W.shape[0] == 2
        assert W.shape[1] == 9

    def test_planes_active(self):
        Jx = self._make_jacobian()
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions()

        W = impl.needle_jacobian(
            Jx, x_needle, ps_vessel, [],
            planes_active=True, spheres_active=False,
            driving_angle_active=False, insertion_angle_active=False,
        )
        assert W is not None
        # Each plane produces 2 constraints
        assert W.shape[0] == 2
        assert W.shape[1] == 9

    def test_both_active(self):
        Jx = self._make_jacobian()
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions()

        W = impl.needle_jacobian(
            Jx, x_needle, ps_vessel, [],
            planes_active=True, spheres_active=True,
            driving_angle_active=False, insertion_angle_active=False,
        )
        assert W is not None
        # 2 spheres + 2 planes = 4
        assert W.shape[0] == 4

    def test_all_inactive(self):
        Jx = self._make_jacobian()
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions()

        W = impl.needle_jacobian(
            Jx, x_needle, ps_vessel, [],
            planes_active=False, spheres_active=False,
            driving_angle_active=False, insertion_angle_active=False,
        )
        # When all constraints are inactive, W is None
        assert W is None

    def test_driving_angle_active(self):
        Jx = self._make_jacobian()
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions()
        ns_vessel = [impl.k_]

        W = impl.needle_jacobian(
            Jx, x_needle, ps_vessel, ns_vessel,
            planes_active=False, spheres_active=False,
            driving_angle_active=True, insertion_angle_active=False,
        )
        assert W is not None
        # Each normal produces 2 constraints
        assert W.shape[0] == 2

    def test_driving_angle_inactive_when_ns_vessel_none(self):
        Jx = self._make_jacobian()
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions()

        W = impl.needle_jacobian(
            Jx, x_needle, ps_vessel, None,
            planes_active=False, spheres_active=False,
            driving_angle_active=True, insertion_angle_active=False,
        )
        # ns_vessel is None so driving angle should be skipped
        assert W is None

    def test_insertion_angle_active(self):
        Jx = self._make_jacobian()
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions()
        ns_vessel = [impl.k_]

        W = impl.needle_jacobian(
            Jx, x_needle, ps_vessel, ns_vessel,
            planes_active=False, spheres_active=False,
            driving_angle_active=False, insertion_angle_active=True,
            needle_radius=0.003,
        )
        assert W is not None
        assert W.shape[0] == 2

    def test_multiple_vessels(self):
        Jx = self._make_jacobian()
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions_multi()

        W = impl.needle_jacobian(
            Jx, x_needle, ps_vessel, [],
            planes_active=True, spheres_active=True,
            driving_angle_active=False, insertion_angle_active=False,
        )
        # 2 vessels × (2 sphere + 2 plane) = 8 constraints
        assert W.shape[0] == 8
        assert W.shape[1] == 9


class TestNeedleW:
    """Tests for needle_w function."""

    def _make_needle_pose(self):
        x = impl.DQ([1, 0, 0, 0, 0.1, 0.2, 0.3, 0.0])
        return x.normalize()

    def _make_vessel_positions(self):
        return [0.5 * impl.k_]

    def test_basic_call(self):
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions()

        w = impl.needle_w(
            x_needle=x_needle,
            ps_vessel=ps_vessel,
            ns_vessel=None,
            needle_radius=0.003,
            vfi_gain_planes=2.0,
            vfi_gain_radius=2.0,
            vfi_gain_angles=2.0,
            vfi_gain_needle_insertion_angles=2.0,
            d_safe_planes=0.001,
            d_safe_radius=0.001,
            d_safe_angles=None,
            d_safe_needle_insertion_angles=None,
            verbose=False,
        )
        # With planes and radius active, should get constraints
        assert w is not None
        # vstack of plane_w (2,) and radius_w (2,) produces (2, 2)
        assert w.shape == (2, 2)

    def test_vfi_constraints_produced_regression(self):
        """Regression: VFI constraints were silently not produced due to backwards ternary.

        The original code had:
            w = np.vstack((..., w)) if w is not None else w
        which kept w as None when w was None. The fix uses intermediate arrays:
            plane_w = np.array([...])
            w = np.vstack((w, plane_w)) if w is not None else plane_w
        """
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions()

        # Test that each constraint type independently produces output
        w_planes_only = impl.needle_w(
            x_needle=x_needle, ps_vessel=ps_vessel, ns_vessel=None,
            needle_radius=0.003, vfi_gain_planes=2.0, vfi_gain_radius=2.0,
            vfi_gain_angles=2.0, vfi_gain_needle_insertion_angles=2.0,
            d_safe_planes=0.001, d_safe_radius=None,
            d_safe_angles=None, d_safe_needle_insertion_angles=None,
            verbose=False,
        )
        assert w_planes_only is not None, "Plane constraints not produced"

        w_radius_only = impl.needle_w(
            x_needle=x_needle, ps_vessel=ps_vessel, ns_vessel=None,
            needle_radius=0.003, vfi_gain_planes=2.0, vfi_gain_radius=2.0,
            vfi_gain_angles=2.0, vfi_gain_needle_insertion_angles=2.0,
            d_safe_planes=None, d_safe_radius=0.001,
            d_safe_angles=None, d_safe_needle_insertion_angles=None,
            verbose=False,
        )
        assert w_radius_only is not None, "Radius constraints not produced"

    def test_all_safe_distances_none(self):
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions()

        w = impl.needle_w(
            x_needle=x_needle,
            ps_vessel=ps_vessel,
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
        # When all safe distances are None, no constraints → None
        assert w is None

    def test_with_angles(self):
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions()
        ns_vessel = [impl.k_]

        w = impl.needle_w(
            x_needle=x_needle,
            ps_vessel=ps_vessel,
            ns_vessel=ns_vessel,
            needle_radius=0.003,
            vfi_gain_planes=2.0,
            vfi_gain_radius=2.0,
            vfi_gain_angles=2.0,
            vfi_gain_needle_insertion_angles=2.0,
            d_safe_planes=None,
            d_safe_radius=None,
            d_safe_angles=0.1,
            d_safe_needle_insertion_angles=None,
            verbose=False,
        )
        # 2 angle constraints
        assert w is not None
        assert w.shape[0] == 2

    def test_with_needle_insertion_angles(self):
        x_needle = self._make_needle_pose()
        ps_vessel = self._make_vessel_positions()
        ns_vessel = [impl.k_]

        w = impl.needle_w(
            x_needle=x_needle,
            ps_vessel=ps_vessel,
            ns_vessel=ns_vessel,
            needle_radius=0.003,
            vfi_gain_planes=2.0,
            vfi_gain_radius=2.0,
            vfi_gain_angles=2.0,
            vfi_gain_needle_insertion_angles=2.0,
            d_safe_planes=None,
            d_safe_radius=None,
            d_safe_angles=None,
            d_safe_needle_insertion_angles=0.1,
            verbose=False,
        )
        assert w is not None
        assert w.shape[0] == 2


class TestRedundantComputationRemoved:
    """Verify the redundant r_needle computation was removed."""

    def test_no_redundant_rotation_call(self):
        """The needle_w function should not recompute r_needle inside the loop.
        This is verified by checking that the source code no longer contains
        the redundant rotation(x_needle) inside the for loop."""
        source_file = os.path.join(
            os.path.dirname(__file__), '..',
            'marinholab', 'working', 'needlemanipulation', '_impl.py'
        )
        with open(source_file) as f:
            source = f.read()

        # Count rotation(x_needle) calls in needle_w
        # We can check that after the initial r_needle = rotation(x_needle),
        # there's no duplicate inside the for loop body
        needle_w_start = source.index("def needle_w(")
        next_def = source.index("\ndef ", needle_w_start + 10) if "\ndef " in source[needle_w_start + 10:] else len(source)
        needle_w_source = source[needle_w_start:next_def]

        rotation_calls = needle_w_source.count("rotation(x_needle)")
        # Should be exactly 1 (the initial assignment)
        assert rotation_calls == 1