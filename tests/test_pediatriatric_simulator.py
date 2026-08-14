"""
Tests for PedriatricSimulator helper functions (LoopInfo, flag functions).
"""
import sys
import os
import pytest
import numpy as np

# Import the module directly (bypasses package __init__.py)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'saul'))
import importlib.util
spec = importlib.util.spec_from_file_location(
    "PedriatricSimulator",
    os.path.join(os.path.dirname(__file__), '..', 'saul', 'PedriatricSimulator.py')
)
ps_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ps_module)


class TestLoopInfo:
    """Tests for the LoopInfo class."""

    def test_parse_active_loop(self):
        info = ps_module.LoopInfo("1 0.1 0.2 0.3 5 10")
        assert info.is_active is True
        assert info.centroid == pytest.approx([0.1, 0.2, 0.3])
        assert info.start_index == 5
        assert info.end_index == 10

    def test_parse_inactive_loop(self):
        info = ps_module.LoopInfo("0 -1.0 -2.0 -3.0 0 0")
        assert info.is_active is False
        assert info.centroid == pytest.approx([-1.0, -2.0, -3.0])
        assert info.start_index == 0
        assert info.end_index == 0

    def test_str_representation(self):
        info = ps_module.LoopInfo("1 1.0 2.0 3.0 4 5")
        s = str(info)
        assert "is_Active: True" in s
        assert "1.0" in s
        assert "2.0" in s
        assert "3.0" in s
        assert "start_index: 4" in s
        assert "end_index: 5" in s


class TestFlagFunctions:
    """Tests for thread segment flag helper functions."""

    def test_is_grabbed_by_left_tool(self):
        assert ps_module.is_grabbed_by_left_tool(1)
        assert ps_module.is_grabbed_by_left_tool(3)
        assert not ps_module.is_grabbed_by_left_tool(0)
        assert not ps_module.is_grabbed_by_left_tool(2)

    def test_is_grabbed_by_right_tool(self):
        assert ps_module.is_grabbed_by_right_tool(2)
        assert ps_module.is_grabbed_by_right_tool(3)
        assert not ps_module.is_grabbed_by_right_tool(0)
        assert not ps_module.is_grabbed_by_right_tool(1)

    def test_is_passing_through_left_tube(self):
        assert ps_module.is_passing_through_left_tube(4)
        assert not ps_module.is_passing_through_left_tube(0)

    def test_is_passing_through_right_tube(self):
        assert ps_module.is_passing_through_right_tube(8)
        assert not ps_module.is_passing_through_right_tube(0)

    def test_is_self_colliding(self):
        assert ps_module.is_self_colliding(16)
        assert not ps_module.is_self_colliding(0)

    def test_is_touching_left_tool_left_side(self):
        assert ps_module.is_touching_left_tool_left_side(32)
        assert not ps_module.is_touching_left_tool_left_side(0)

    def test_is_touching_left_tool_right_side(self):
        assert ps_module.is_touching_left_tool_right_side(64)
        assert not ps_module.is_touching_left_tool_right_side(0)

    def test_combined_flags(self):
        # Flag with both left grab and right grab
        combined = 1 | 2  # 3
        assert ps_module.is_grabbed_by_left_tool(combined)
        assert ps_module.is_grabbed_by_right_tool(combined)


class TestDetectNumLoopsFromFlags:
    """Tests for detect_num_loops_from_flags."""

    def test_no_flags(self):
        result = ps_module.detect_num_loops_from_flags([])
        assert result == 0

    def test_single_flag(self):
        result = ps_module.detect_num_loops_from_flags([32])
        assert result == 0

    def test_no_transition(self):
        # All left side - no loop
        result = ps_module.detect_num_loops_from_flags([32, 32, 32])
        assert result == 0

    def test_one_transition(self):
        # One transition: left → right = 1 → (1+1)//2 = 1
        result = ps_module.detect_num_loops_from_flags([32, 64])
        assert result == 1

    def test_two_transitions(self):
        # Two transitions: left → right → left = 1 loop
        result = ps_module.detect_num_loops_from_flags([32, 64, 32])
        assert result == 1

    def test_no_tool_contact_flags(self):
        # Flags that don't have tool contact → empty filtered_flags
        result = ps_module.detect_num_loops_from_flags([1, 2, 4, 8])
        assert result == 0


class TestUnitaryThreadLength:
    """Tests for unitary thread length functions."""

    def test_get_unitary_thread_start_length_no_tube(self):
        # No segment passes through tube
        flags = [0, 0, 0]
        result = ps_module.get_unitary_thread_start_length_from_flags(flags)
        assert result == 0

    def test_get_unitary_thread_start_length_no_grab(self):
        # Passes through tube but never grabbed
        flags = [4, 0, 0]
        result = ps_module.get_unitary_thread_start_length_from_flags(flags)
        assert result == 0

    def test_get_unitary_thread_start_length_valid(self):
        # Tube at index 0, grabbed at index 1 (within range(n) where n=len-1=2)
        flags = [4, 1, 0]
        result = ps_module.get_unitary_thread_start_length_from_flags(flags)
        n = len(flags) - 1  # 2
        expected = abs(0 - 1) / n  # 0.5
        assert result == pytest.approx(expected)

    def test_get_unitary_thread_end_length_no_tube(self):
        flags = [0, 0, 0]
        result = ps_module.get_unitary_thread_end_length_from_flags(flags)
        assert result == 0

    def test_get_unitary_thread_end_length_valid(self):
        # Tube at last index
        flags = [0, 0, 4]
        n = len(flags)  # 3
        result = ps_module.get_unitary_thread_end_length_from_flags(flags)
        expected = 1.0 - (2 + 0.5) / n  # 1.0 - 2.5/3 = 0.1667
        assert result == pytest.approx(expected)