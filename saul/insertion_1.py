"""
Saul's pediatric insertion scenario, driven live against a running
PedriatricSimulator over TCP (``127.0.0.1``).

The script:

1. Builds left and right :class:`M3_SerialManipulatorSimulatorFriendly`
   models from the simulator's joint transforms.
2. Configures the joint limits (rad) and RCM constraint spheres for both.
3. Constructs a :class:`NeedleController` for the right robot with the
   insertion constraints enabled, using a vessel point and normal derived
   from the simulator.
4. Runs the closed-loop for a fixed number of outer loops (200) with 10
   inner QP steps each, streaming the needle pose to the simulator.
5. Finally performs a 300-step pure kinematic insertion sweep and
   disconnects.

Requires ``PedriatricSimulator`` on ``PYTHONPATH`` and a running simulator
instance.
"""
import math

from dqrobotics.utils.DQ_Math import deg2rad
import numpy as np
from dqrobotics import (
    conj,
    translation,
    rotation,
    Ad,
    E_,
    i_,
    j_,
    k_
)

import PedriatricSimulator
import time

from marinholab.working.needlemanipulation import NeedleController
from marinholab.working.needlemanipulation import M3_SerialManipulatorSimulatorFriendly

SIMULATOR_FREQUENCY = 60
CONTROLLER_FREQUENCY = 250
CONTROLLER_STEPS = int(CONTROLLER_FREQUENCY / SIMULATOR_FREQUENCY)
CONTROLLER_SAMPLING_TIME = 1.0 / CONTROLLER_FREQUENCY

rcm_joint_index = 6

sim = PedriatricSimulator.PediatricSimulator()
sim.connect("127.0.0.1")
sim.clear_frames()
time.sleep(1)

print("needle radius {}".format(sim.get_needle_radius()))


def make_robot(base_frame, transforms):
    """Build a :class:`M3_SerialManipulatorSimulatorFriendly` model.

    The first joint is anchored to ``base_frame``; all subsequent joints
    are anchored to the identity and all joints are treated as
    :attr:`ActuationType.RX`.

    Args:
        base_frame: Base-frame dual quaternion of the robot.
        transforms: Sequence of per-joint dual-quaternion transforms as
            returned by the simulator (``get_left_robot_model_ith`` /
            ``get_right_robot_model_ith``).

    Returns:
        A :class:`M3_SerialManipulatorSimulatorFriendly` instance with
        ``len(transforms)`` DOF.
    """
    n = len(transforms)
    offsets_before = [base_frame]
    offsets_after = transforms
    actuation_types = []

    for j in range(n):
        if j > 0:
            offsets_before.append(1 + 0 * E_)
        actuation_types.append(M3_SerialManipulatorSimulatorFriendly.ActuationType.RX)

    return M3_SerialManipulatorSimulatorFriendly(
        offsets_before,
        offsets_after,
        actuation_types
    )

dofs = sim.get_robot_dofs()

rrobot = make_robot(sim.get_right_robot_base(), [sim.get_right_robot_model_ith(k) for k in range(dofs)])

lower_q_limit = deg2rad([-85, -85, 5, -265, -85, -355, -170, -30, -30])
upper_q_limit = deg2rad([85, 85, 120, 0, 85, 355, 170, 30, 30])

rrobot.set_lower_q_limit(lower_q_limit)
rrobot.set_upper_q_limit(upper_q_limit)

rrcm = {"position": sim.get_right_trocar_sphere()[0], "radius": sim.get_right_trocar_sphere()[1]}

translate = 1 + 0.5 * E_ * j_ * -0.0005
angle = -math.pi / 2.0
rotate1 = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (i_ * 1.0 + j_ * 0.0 + k_ * 0.0)
angle = math.pi / 2.0
rotate2 = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (i_ * 0.0 + j_ * 0.0 + k_ * 1.0)

p1 = translate * sim.get_right_tube_target_point() * rotate1 * rotate2
sim.set_frame("p1", p1)

needle_pose = sim.get_control_needle_pose()
q = sim.get_right_robot_joints()
ee = rrobot.fkm(q)
relative_needle_pose = conj(ee) * needle_pose
radius = sim.get_needle_radius()

needle_tip_pose = sim.get_needle_frame_at(0.0)
relative_needle_tip_pose = conj(ee) * needle_tip_pose

rrobot.set_effector(relative_needle_tip_pose)

v = j_
n = Ad(rotation(needle_pose), k_)
n1 = Ad(rotation(p1), k_)

needle_positioning_controller = NeedleController(
        kinematics=rrobot,
        gain=100.0,
        damping=np.diag([1,1,1,1,1,1,0,0,0]),
        alpha=1.0,
        rcm_constraints=[
            (rrcm["position"], rrcm["radius"], rcm_joint_index)],
        relative_needle_pose=relative_needle_pose,
        vessel_positions=[translation(p1)],
        vessel_normals=[n1],
        needle_radius=None,
        d_safe_angles=None,
        vfi_gain=1.0,
        verbose=False,
        insertion_constraints=True
    )

q = sim.get_right_robot_joints()
xdc = None

print("Starting needle positioning loop...")
for i in range(1000):
    sim.set_frame("needle", sim.get_control_needle_pose())

    x = rrobot.fkm(q)
    sim.set_frame("x", x)

    # controlled target pose
    xdc = p1
    sim.set_frame("xd", xdc)

    for step in range(CONTROLLER_STEPS):
        # Solve the quadratic program
        u = needle_positioning_controller.compute_setpoint_control_signal(q, xdc)

        # Update the current joint positions
        q = q + u * CONTROLLER_SAMPLING_TIME

    sim.set_right_robot_joints(q)

    time.sleep(1.0 / 60.0)
print("Needle positioning loop finished.")

needle_driving_1_controller = NeedleController(
        kinematics=rrobot,
        gain=2000.0,
        damping=np.diag([1,1,1,1,1,1,0,0,0]),
        alpha=1.0,
        rcm_constraints=[
            (rrcm["position"], rrcm["radius"], rcm_joint_index)],
        relative_needle_pose=relative_needle_pose,
        vessel_positions=[translation(p1)],
        vessel_normals=[v],
        needle_radius=radius,
        d_safe_angles=np.pi / 8.0,
        vfi_gain=1.0,
        verbose=True,
        insertion_constraints=False
    )

print("Starting needle driving loop...")
q = sim.get_right_robot_joints()
needle_center = rrobot.fkm(q) * conj(relative_needle_pose)
for i in range(500):

    angle = 0.001 * i * math.pi / 2.0
    rotate = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (i_ * 0.0 + j_ * 1.0 + k_ * 0.0)

    translate = 1 + 0.5 * E_ * (i_ * 0.0 + j_ * 0.00001 * i + k_ * 0.0)

    if i < 1500:
        xdc = needle_center * rotate * relative_needle_pose

    x = rrobot.fkm(q)
    sim.set_frame("xd_new", xdc)
    sim.set_frame("x", x)
    sim.set_frame("p1", p1)
    sim.set_frame("needle_center", needle_center)
    sim.set_frame("needle_rotate", needle_center * rotate)

    for step in range(CONTROLLER_STEPS):
        # Solve the quadratic program
        u = needle_driving_1_controller.compute_setpoint_control_signal(q, xdc)

        # Update the current joint positions
        q = q + u * CONTROLLER_SAMPLING_TIME
        print(f"Last error norm {np.linalg.norm(needle_driving_1_controller.get_last_error())}")

    sim.set_right_robot_joints(q)

    time.sleep(1/60)

sim.disconnect()
