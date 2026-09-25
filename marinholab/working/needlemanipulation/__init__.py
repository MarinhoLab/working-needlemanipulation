from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulator

from marinholab.sas.core.modeling import (
    ActuationType,
    SerialManipulatorSimulatorFriendly,
)
from marinholab.working.needlemanipulation._impl import needle_jacobian, needle_w
from marinholab.sas.core.papers.icra2019 import Controller
from marinholab.working.needlemanipulation.needle_controller import NeedleController

# Backward-compatible alias: the model moved from this repo (where it lived as
# ``M3_SerialManipulatorSimulatorFriendly`` in ``namespace DQ_robotics``) to
# ``marinholab.sas.core.modeling.SerialManipulatorSimulatorFriendly``. Existing
# code that does ``from marinholab.working.needlemanipulation import
# M3_SerialManipulatorSimulatorFriendly`` keeps working via this alias.
M3_SerialManipulatorSimulatorFriendly = SerialManipulatorSimulatorFriendly

# Backward-compatible alias: the ICRA 2019 task-space controller moved from
# this repo (``icra2019_controller.ICRA19TaskSpaceController``) to
# ``marinholab.sas.core.papers.icra2019.Controller``. Existing code that does
# ``from marinholab.working.needlemanipulation import
# ICRA19TaskSpaceController`` keeps working via this alias.
ICRA19TaskSpaceController = Controller

__all__ = [
    "SerialManipulatorSimulatorFriendly",
    "M3_SerialManipulatorSimulatorFriendly",
    "ActuationType",
    "needle_jacobian",
    "needle_w",
    "Controller",
    "ICRA19TaskSpaceController",
    "NeedleController",
]
