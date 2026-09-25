# Working Project: Needle Manipulation

A University of Manchester (UoM) and University of Tokyo (UTokyo)
collaboration, on dual-quaternion (DQ) control for needle
insertion. Built on [DQRobotics](https://github.com/dqrobotics), it provides a
QP-based task-space controller with remote-centre-of-motion (RCM) and
joint-limit constraints, extended with violation-field constraints that keep a
needle within a safe band around vessel primitives (with optional angular
insertion constraints).

## Installation

```
pip install marinholab-working-needlemanipulation
```

This is a pure-Python package. The serial-manipulator kinematics model
(`SerialManipulatorSimulatorFriendly`, re-exported here as
`M3_SerialManipulatorSimulatorFriendly` for backward compatibility) is provided
by the `marinholab-sas-core` dependency, so no C++ toolchain or git submodules
are needed to build it. See [`AGENTS.md`](AGENTS.md).

## Usage

The public API is exposed at the package root:

```python
from marinholab.working.needlemanipulation import (
    Controller,                        # QP task-space control (RCM + joint limits)
    NeedleController,                  # adds vessel VFI + insertion constraints
    M3_SerialManipulatorSimulatorFriendly,  # serial-manipulator model
)
```

`Controller` is the ICRA 2019 task-space controller and is re-exported from
`marinholab.sas.core.papers.icra2019` (it now ships in the
`marinholab-sas-core` dependency). `ICRA19TaskSpaceController` is kept as a
backward-compatible alias for it.

Three examples are installed as commands:

- `needlemanipulation_example` — build a small robot and (if `matplotlib` is
  available) plot it in 3D.
- `needlemanipulation_example_create_needle_controller` — build a
  `NeedleController` from the bundled `left_robot.yaml`.
- `needlemanipulation_example_load_from_file` — run the base controller on the
  bundled model for a short trajectory (records an MP4 when `matplotlib` is
  available).

The live end-to-end insertion simulations against the pediatric simulator live
in [`saul/`](saul/) and require a running simulator instance; they are not part
of the installed package.

## License

See [`LICENSE`](LICENSE) for details.
