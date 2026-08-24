"""Stubs for `dqrobotics.solvers` (see `stubs/dqrobotics/__init__.pyi`)."""

from __future__ import annotations

from typing import Optional

import numpy as np
from numpy.typing import NDArray

__all__ = [
    "DQ_QuadraticProgrammingSolver",
    "DQ_QuadprogSolver",
]


class DQ_QuadraticProgrammingSolver:
    """Abstract base for quadratic-programming solvers.

    Solves::

        min_x 0.5 * x' H x + f' x
        s.t.  A x <= b,  Aeq x = beq
    """

    def solve_quadratic_program(
        self,
        H: NDArray[np.float64],
        f: NDArray[np.float64],
        A: Optional[NDArray[np.float64]],
        b: Optional[NDArray[np.float64]],
        Aeq: Optional[NDArray[np.float64]],
        beq: Optional[NDArray[np.float64]],
    ) -> NDArray[np.float64]: ...


class DQ_QuadprogSolver(DQ_QuadraticProgrammingSolver):
    """Concrete QP solver backed by the ``quadprog`` package.

    Not instantiated when ``quadprog`` is not installed (the import is
    guarded by a bare ``try/except`` in ``dqrobotics.solvers``).
    """

    def __init__(self) -> None: ...

    def set_equality_constraints_tolerance(self, tolerance: float) -> None: ...

    def get_equality_constraints_tolerance(self) -> float: ...
