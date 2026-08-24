"""Stubs for `dqrobotics.utils` (see `stubs/dqrobotics/__init__.pyi`)."""

from __future__ import annotations

import numpy as np

from dqrobotics import DQ

__all__ = [
    "DQ_Geometry",
]


class DQ_Geometry:
    """Stateless geometric distances between dual-quaternion primitives."""

    @staticmethod
    def point_to_point_squared_distance(p1: DQ, p2: DQ) -> float: ...

    @staticmethod
    def point_to_point_distance(p1: DQ, p2: DQ) -> float: ...

    @staticmethod
    def point_to_plane_distance(p: DQ, plane: DQ) -> float: ...

    @staticmethod
    def point_to_line_squared_distance(p: DQ, line: DQ) -> float: ...
