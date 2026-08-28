"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
MIT License

Needle manipulation controller. Extends
:class:`ICRA19TaskSpaceController` with VFI constraints that keep the needle
inside the safe volume around one or more vessel primitives and (optionally)
within a depth-dependent angular insertion band.
"""
import math

import numpy as np

from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulator

from marinholab.working.needlemanipulation._impl import needle_jacobian, needle_w
from marinholab.working.needlemanipulation.icra2019_controller import ICRA19TaskSpaceController


class NeedleController(ICRA19TaskSpaceController):
    """A task-space controller for needle insertion with vessel VFI constraints.

    Inherits the QP-based task-space control of
    :class:`ICRA19TaskSpaceController` and stacks an additional set of VFI
    inequality constraints (see :mod:`marinholab.working.needlemanipulation._impl`)
    onto the joint-limit and RCM constraints before solving.
    """

    def __init__(
        self,
        kinematics: DQ_SerialManipulator,
        gain: float,
        damping: float,
        alpha: float,
        rcm_constraints: list[tuple[DQ, float, int]],
        relative_needle_pose: DQ,
        vessel_positions: list[DQ],
        needle_radius: float,
        vfi_gain: float = 2.0,
        insertion_constraints: bool = False,
        **kwargs,
    ) -> None:
        """Initialize the needle controller.

        Args:
            kinematics: The serial-manipulator model of the robot.
            gain: Proportional task-space gain.
            damping: Damping factor (scalar or a ``DOF x DOF`` matrix).
            alpha: Soft priority between translation (``alpha``) and rotation
                (``1 - alpha``); typically close to 1.
            rcm_constraints: List of ``(position, radius, joint_index)`` RCM
                constraints, as in the parent class.
            relative_needle_pose: Fixed dual-quaternion pose of the needle
                expressed in the end-effector frame.
            vessel_positions: Vessel primitives as pure dual quaternions
                (typically points on the vessel wall).
            needle_radius: Physical needle radius (m).
            vfi_gain: Default VFI gain applied to constraints that do not
                override it via the ``vfi_gain_*`` kwargs.
            insertion_constraints: If ``True``, enable the tip point-to-line
                insertion constraint and the depth-dependent angular
                insertion constraint on the first vessel.
            kwargs: Optional keyword arguments, passed to the parent class
                and additionally used to override per-category VFI gains and
                safety margins. Recognized keys:

                * ``verbose`` (bool or dict) — per-category debug output;
                  ``True`` prints every category, ``False`` prints nothing,
                  and a dict selects categories by name (``"radius"``,
                  ``"plane"``, ``"orientation"``, ``"insertion"``,
                  ``"rcm"``).
                * ``vessel_normals`` (list[DQ]) — vessel normals as pure DQs.
                * ``vfi_gain_planes``, ``vfi_gain_radius``,
                  ``vfi_gain_angles`` (float) — per-category VFI gains.
                * ``d_safe_planes``, ``d_safe_radius`` (float) — safe
                  distance margins (m).
                * ``d_safe_angles`` (float) — safe angular margin (rad).
        """
        super().__init__(
            kinematics,
            gain,
            damping,
            alpha,
            rcm_constraints,
            vfi_gain,
            **kwargs,
        )

        # Optional per-category VFI gains and safety margins; if not
        # supplied, :meth:`compute_setpoint_control_signal` falls back to
        # ``self.vfi_gain`` and small defaults.
        if "vfi_gain_planes" in kwargs:
            self.vfi_gain_planes: float = kwargs["vfi_gain_planes"]
        if "vfi_gain_radius" in kwargs:
            self.vfi_gain_radius: float = kwargs["vfi_gain_radius"]
        if "vfi_gain_angles" in kwargs:
            self.vfi_gain_angles: float = kwargs["vfi_gain_angles"]
        if "d_safe_planes" in kwargs:
            self.d_safe_planes: float = kwargs["d_safe_planes"]
        if "d_safe_radius" in kwargs:
            self.d_safe_radius: float = kwargs["d_safe_radius"]
        if "vessel_normals" in kwargs:
            self.vessel_normals: list[DQ] = kwargs["vessel_normals"]
        if "d_safe_angles" in kwargs:
            self.d_safe_angles: float = kwargs["d_safe_angles"]

        self.relative_needle_pose: DQ = relative_needle_pose
        self.vessel_positions: list[DQ] = vessel_positions
        self.needle_radius: float = needle_radius
        self.insertion_constraints: bool = insertion_constraints

    def compute_setpoint_control_signal(self, q: np.ndarray, xd: DQ) -> np.ndarray:
        """Compute the control signal for the next step with vessel constraints.

        Extends the parent's QP with the VFI constraints derived from
        :func:`marinholab.working.needlemanipulation._impl.needle_jacobian`
        and :func:`marinholab.working.needlemanipulation._impl.needle_w`
        and solves the augmented QP.

        Args:
            q: Current joint positions.
            xd: Desired end-effector pose (a unit dual quaternion).

        Returns:
            The joint-velocity vector to be applied for this step.

        Raises:
            Exception: If ``xd`` is not a unit dual quaternion.
        """
        if not is_unit(xd):
            raise Exception("ICRA19TaskSpaceController::compute_setpoint_control_signal::xd should be an unit dual "
                            "quaternion")

        H, f, W, w = self._get_optimization_parameters(q, xd)

        # The relative transformation of the needle is time-constant
        x = self.last_x
        Jx = self.last_Jx
        if x is None or Jx is None:
            raise RuntimeError(
                "Internal error: last_x / last_Jx not set. "
                "Did you call _get_optimization_parameters first?"
            )
        Jx_needle = haminus8(self.relative_needle_pose) @ Jx
        x_needle = x * self.relative_needle_pose

        # VFI-related Jacobian
        W_needle = needle_jacobian(
            Jx_needle,
            x_needle,
            self.vessel_positions,
            self.vessel_normals if hasattr(self, "vessel_normals") else None,
            self.insertion_constraints,
            needle_offset=conj(self.relative_needle_pose),
        )
        # VFI w
        w_needle = needle_w(
            x_needle=x_needle,
            ps_vessel=self.vessel_positions,
            ns_vessel=self.vessel_normals if hasattr(self, "vessel_normals") else None,
            needle_radius=self.needle_radius,
            vfi_gain_planes=self.vfi_gain_planes if hasattr(self, "vfi_gain_planes") else self.vfi_gain,
            vfi_gain_radius=self.vfi_gain_radius if hasattr(self, "vfi_gain_radius") else self.vfi_gain,
            vfi_gain_angles=self.vfi_gain_angles if hasattr(self, "vfi_gain_angles") else self.vfi_gain,
            d_safe_planes=self.d_safe_planes if hasattr(self, "d_safe_planes") else 0.0005,
            d_safe_radius=self.d_safe_radius if hasattr(self, "d_safe_radius") else 0.0005,
            d_safe_angles=self.d_safe_angles if hasattr(self, "d_safe_angles") else math.pi / 4,
            verbose=self.verbose,
            insertion_constraints=self.insertion_constraints,
            needle_offset=conj(self.relative_needle_pose),
        ).reshape((W_needle.shape[0],))

        if W is not None and w is not None:
            W = np.vstack((W, W_needle))
            w = np.hstack((w, w_needle))
        else:
            W = W_needle
            w = w_needle

        # ``_get_optimization_parameters`` always returns a non-None ``W``/``w``
        # (at least the joint-limit rows), but its signature types them as
        # ``Optional``; assert the invariant so the ``.dtype`` checks below
        # narrow correctly.
        assert W is not None and w is not None
        # print(f"H={H} \n\n f={f} \n\n W={W} \n\n w={np.squeeze(w)} \n\n")
        assert H.dtype == np.float64
        assert f.dtype == np.float64
        assert W.dtype == np.float64
        assert np.squeeze(w).dtype == np.float64
        if np.any(w < 0):
            raise RuntimeError(f"ERROR: VFI constraints violated, w={w}")
        u = self.qp_solver.solve_quadratic_program(H, f, W, np.squeeze(w), None, None)

        return u
