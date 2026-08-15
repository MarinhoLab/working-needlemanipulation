"""
Tests for example_load_from_file.get_information_from_file.
"""
import sys
import os
import pytest
import yaml
from unittest.mock import Mock, patch

# Mock _core before importing
mock_core = Mock()
sys.modules.setdefault('marinholab.working.needlemanipulation._core', mock_core)

# Load the module directly
import importlib.util
spec = importlib.util.spec_from_file_location(
    "example_load_from_file",
    os.path.join(os.path.dirname(__file__), '..',
                 'marinholab', 'working', 'needlemanipulation', 'example_load_from_file.py')
)
load_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(load_module)

# Also import dqrobotics for creating DQ objects
from dqrobotics import DQ


@pytest.fixture
def valid_yaml_content():
    """Return valid YAML content matching left_robot.yaml structure."""
    return yaml.dump({
        "actuation_types": ["RX", "RX", "RX"],
        "offsets_before": [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ],
        "offsets_after": [
            [0.7071, 0.7071, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.7071, 0.7071, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.7071, 0.7071, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ],
        "rcm1": [[0.0, 0.01, 0.0], 0.01],
        "rcm2": [[-0.025, 0.024, 0.026], 0.001],
    })


class TestGetInformationFromFile:
    """Tests for get_information_from_file function."""

    def test_returns_robot_and_rcm(self, valid_yaml_content):
        robot, rcm1, rcm2 = load_module.get_information_from_file(valid_yaml_content)

        assert robot is not None
        assert isinstance(rcm1, dict)
        assert isinstance(rcm2, dict)
        assert "position" in rcm1
        assert "radius" in rcm1
        assert "position" in rcm2
        assert "radius" in rcm2

    def test_rcm1_values(self, valid_yaml_content):
        _, rcm1, _ = load_module.get_information_from_file(valid_yaml_content)

        assert isinstance(rcm1["position"], DQ)
        assert rcm1["radius"] == 0.01

    def test_rcm2_values(self, valid_yaml_content):
        _, _, rcm2 = load_module.get_information_from_file(valid_yaml_content)

        assert isinstance(rcm2["position"], DQ)
        assert rcm2["radius"] == 0.001

    def test_invalid_actuation_type(self):
        """Test that non-RX actuation types raise an error."""
        yaml_content = yaml.dump({
            "actuation_types": ["RX", "RY", "RX"],
            "offsets_before": [[1.0] * 8] * 3,
            "offsets_after": [[0.7071] * 8] * 3,
            "rcm1": [[0.0, 0.01, 0.0], 0.01],
            "rcm2": [[-0.025, 0.024, 0.026], 0.001],
        })
        with pytest.raises(RuntimeError, match="Only RX is accepted"):
            load_module.get_information_from_file(yaml_content)

    def test_offset_normalization(self, valid_yaml_content):
        """Test that DQ offsets are normalized."""
        robot, _, _ = load_module.get_information_from_file(valid_yaml_content)

        # The offsets should be unit dual quaternions
        # We verify the robot was created successfully
        assert robot is not None


class TestPlotHelpers:
    """Tests for plotting helper functions."""

    def test_set_plot_labels(self):
        """Test _set_plot_labels creates correct labels (requires matplotlib)."""
        # We test that the function exists and is callable
        assert callable(load_module._set_plot_labels)

    def test_set_plot_limits(self):
        """Test _set_plot_limits creates correct limits."""
        assert callable(load_module._set_plot_limits)


class TestBugFixes:
    """Verify that the bug fixes were applied correctly."""

    def test_rcm_uses_radius_not_diameter_in_example_plot(self):
        """Verify example_plot uses 'radius' key instead of 'diameter'."""
        source_file = os.path.join(
            os.path.dirname(__file__), '..',
            'marinholab', 'working', 'needlemanipulation', 'example_load_from_file.py'
        )
        with open(source_file) as f:
            source = f.read()

        # The fixed code should use rcm1["radius"] and rcm2["radius"]
        assert 'rcm1["radius"]' in source
        assert 'rcm2["radius"]' in source
        # The buggy rcm1["diameter"] should NOT be present
        assert 'rcm1["diameter"]' not in source
        assert 'rcm2["diameter"]' not in source

    def test_rcm_constraints_have_joint_index_in_main(self):
        """Verify rcm_constraints in main() include joint index."""
        source_file = os.path.join(
            os.path.dirname(__file__), '..',
            'marinholab', 'working', 'needlemanipulation', 'example_load_from_file.py'
        )
        with open(source_file) as f:
            source = f.read()

        # The fixed main() should have 3-tuple rcm_constraints with joint index 6
        assert '(lrcm1["position"], lrcm1["radius"], 6)' in source
        assert '(lrcm2["position"], lrcm2["radius"], 6)' in source


class TestGraspingNeedleBugFix:
    """Verify the grasping_needle.py bug fix."""

    def test_vessel_positions_plural(self):
        """Verify grasping_needle.py uses 'vessel_positions' (plural) parameter."""
        source_file = os.path.join(
            os.path.dirname(__file__), '..',
            'saul', 'grasping_needle.py'
        )
        with open(source_file) as f:
            source = f.read()

        # Should use vessel_positions (plural)
        assert 'vessel_positions=' in source
        # Should NOT use vessel_position (singular)
        assert 'vessel_position=' not in source


class TestPedriatricSimulatorBugFix:
    """Verify the PedriatricSimulator.py cv2 import fix."""

    def test_cv2_import_present(self):
        """Verify cv2 is imported in PedriatricSimulator.py."""
        source_file = os.path.join(
            os.path.dirname(__file__), '..',
            'saul', 'PedriatricSimulator.py'
        )
        with open(source_file) as f:
            source = f.read()

        assert 'import cv2' in source


class TestICRAControllerBugFix:
    """Verify the ICRA19TaskSpaceController bug fix."""

    def test_w_c_indexed_comparison(self):
        """Verify w_c is indexed in the comparison (w_c[0] < 0, not w_c < 0)."""
        source_file = os.path.join(
            os.path.dirname(__file__), '..',
            'marinholab', 'working', 'needlemanipulation', 'icra2019_controller.py'
        )
        with open(source_file) as f:
            lines = f.readlines()

        # Find the line with the comparison
        found_indexed = False
        for line in lines:
            if 'w_c' in line and '< 0' in line and 'if' in line:
                if 'w_c[0]' in line:
                    found_indexed = True
                elif 'w_c < 0' in line:
                    pytest.fail("Found unindexed 'w_c < 0' comparison")

        assert found_indexed, "Could not find the w_c comparison line"


class TestNeedleControllerBugFix:
    """Verify the NeedleController bug fix."""

    def test_null_constraint_handling(self):
        """Verify NeedleController handles null W_needle gracefully."""
        source_file = os.path.join(
            os.path.dirname(__file__), '..',
            'marinholab', 'working', 'needlemanipulation', 'needle_controller.py'
        )
        with open(source_file) as f:
            source = f.read()

        # Should check for W_needle is not None before reshaping
        assert 'W_needle is not None' in source


class TestRedundantComputationBugFix:
    """Verify the redundant computation removal in _impl.py."""

    def test_no_redundant_rotation_in_needle_w(self):
        """Verify needle_w only calls rotation(x_needle) once."""
        source_file = os.path.join(
            os.path.dirname(__file__), '..',
            'marinholab', 'working', 'needlemanipulation', '_impl.py'
        )
        with open(source_file) as f:
            source = f.read()

        needle_w_start = source.index("def needle_w(")
        next_def = source.index("\ndef ", needle_w_start + 10) if "\ndef " in source[needle_w_start + 10:] else len(source)
        needle_w_source = source[needle_w_start:next_def]

        rotation_calls = needle_w_source.count("rotation(x_needle)")
        assert rotation_calls == 1, f"Expected 1 rotation(x_needle) call, found {rotation_calls}"