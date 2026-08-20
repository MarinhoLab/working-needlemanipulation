"""
Copyright (C) 2025-26 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License

Needle manipulation package providing task-space controllers for surgical robotics.
"""

# Re-export the full dqrobotics namespace at the package root for backward
# compatibility with releases up to 25.6.0.66, which exposed every dqrobotics
# symbol (DQ, k_, conj, haminus8, ...) via ``from dqrobotics import *``.
from dqrobotics import *  # noqa: F403
import dqrobotics as _dqrobotics
from dqrobotics.robot_modeling import DQ_SerialManipulator
from marinholab.working.needlemanipulation._core import (
    M3_SerialManipulatorSimulatorFriendly,
)
from marinholab.working.needlemanipulation._impl import (
    needle_jacobian,
    needle_w,
)
from marinholab.working.needlemanipulation.icra2019_controller import (
    ICRA19TaskSpaceController,
)
from marinholab.working.needlemanipulation.needle_controller import (
    NeedleController,
)

__all__ = [
    "DQ_SerialManipulator",
    "M3_SerialManipulatorSimulatorFriendly",
    "needle_jacobian",
    "needle_w",
    "ICRA19TaskSpaceController",
    "NeedleController",
]
# Expose the dqrobotics symbols pulled in by the star import above so that
# ``from marinholab.working.needlemanipulation import *`` keeps working as it
# did in 25.6.0.66.
__all__ += [n for n in dir(_dqrobotics) if not n.startswith("_")]
