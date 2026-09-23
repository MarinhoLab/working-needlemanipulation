from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulator

from marinholab.sas.core.modeling import (
    ActuationType,
    SerialManipulatorSimulatorFriendly,
)
from marinholab.working.needlemanipulation._impl import needle_jacobian, needle_w
from marinholab.working.needlemanipulation.icra2019_controller import ICRA19TaskSpaceController
from marinholab.working.needlemanipulation.needle_controller import NeedleController

# Backward-compatible alias: the model moved from this repo (where it lived as
# ``M3_SerialManipulatorSimulatorFriendly`` in ``namespace DQ_robotics``) to
# ``marinholab.sas.core.modeling.SerialManipulatorSimulatorFriendly``. Existing
# code that does ``from marinholab.working.needlemanipulation import
# M3_SerialManipulatorSimulatorFriendly`` keeps working via this alias.
M3_SerialManipulatorSimulatorFriendly = SerialManipulatorSimulatorFriendly

__all__ = [
    "SerialManipulatorSimulatorFriendly",
    "M3_SerialManipulatorSimulatorFriendly",
    "ActuationType",
    "needle_jacobian",
    "needle_w",
    "ICRA19TaskSpaceController",
    "NeedleController",
]
