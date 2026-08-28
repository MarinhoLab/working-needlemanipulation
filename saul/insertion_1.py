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
CONTROLLER_FREQUENCY = 1000
CONTROLLER_STEPS = int(CONTROLLER_FREQUENCY / SIMULATOR_FREQUENCY)
CONTROLLER_SAMPLING_TIME = 1.0 / CONTROLLER_FREQUENCY

rcm_joint_index = 6

sim = PedriatricSimulator.PediatricSimulator()
sim.connect("127.0.0.1")
#sim.clear_frames()
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

p1 = sim.get_right_tube_target_point() * (1 + 0.5 * E_ * j_ * -0.0005)
sim.set_frame("p1", p1)

radius = sim.get_needle_radius()

w_to_needle_center = sim.get_control_needle_pose()
q = sim.get_right_robot_joints()
w_to_robot_ee = rrobot.fkm(q)
w_to_needle_tip = sim.get_needle_frame_at(0.0)

# Tip
needle_center_to_needle_tip = conj(w_to_needle_center) * w_to_needle_tip
needle_tip_to_needle_center = conj(needle_center_to_needle_tip)
robot_ee_to_needle_tip = conj(w_to_robot_ee) * w_to_needle_tip
rrobot.set_effector(robot_ee_to_needle_tip)

# Vessel normals are always j_ in this world
v = j_

safety_set_controller = NeedleController(
        kinematics=rrobot,
        gain=1500.0,
        damping=np.diag([1,1,1,1,1,1,0.000001,0.000001,0.000001]),
        alpha=1.0,
        rcm_constraints=[
            (rrcm["position"], rrcm["radius"], rcm_joint_index)],
        relative_needle_pose=needle_tip_to_needle_center,
        vessel_positions=None,
        vessel_normals=None,
        needle_radius=None,
        d_safe_angles=None,
        vfi_gain=1.0,
        verbose=False
    )

q = sim.get_right_robot_joints()
x = rrobot.fkm(q)
print("Starting safety set controller loop...")
for i in range(200):
    sim.set_frame("needle", sim.get_control_needle_pose())

    x = rrobot.fkm(q)
    sim.set_frame("x", x)

    # controlled target pose
    sim.set_frame("xd", x)

    for step in range(CONTROLLER_STEPS):
        u = safety_set_controller.compute_setpoint_control_signal(q, x)
        q = q + u * CONTROLLER_SAMPLING_TIME

    sim.set_right_robot_joints(q)

    time.sleep(1.0 / 60.0)
print("Safety set controller loop finished.")

needle_positioning_controller = NeedleController(
        kinematics=rrobot,
        gain=1000.0,
        damping=np.diag([1,1,1,1,1,1,0.000001,0.000001,0.000001]),
        alpha=1.0,
        rcm_constraints=[
            (rrcm["position"], rrcm["radius"], rcm_joint_index)],
        relative_needle_pose=needle_tip_to_needle_center,
        vessel_positions=[translation(p1)],
        vessel_normals=[v],
        needle_radius=None,
        d_safe_angles=None,
        vfi_gain=10.0,
        verbose=True,
        insertion_constraints=True
    )

q = sim.get_right_robot_joints()
print("Starting needle positioning loop...")
for i in range(2000):
    xd = p1

    sim.set_frame("needle", sim.get_control_needle_pose())

    x = rrobot.fkm(q)
    sim.set_frame("x", x)
    sim.set_frame("xd", xd)

    for step in range(CONTROLLER_STEPS):
        u = needle_positioning_controller.compute_setpoint_control_signal(q, xd)
        q = q + u * CONTROLLER_SAMPLING_TIME

    sim.set_right_robot_joints(q)

    time.sleep(1.0 / 60.0)
print("Needle positioning loop finished.")

needle_driving_1_controller = NeedleController(
        kinematics=rrobot,
        gain=100.0,
        damping=np.diag([1,1,1,1,1,1,0.000001,0.000001,0.000001]),
        alpha=0.999,
        rcm_constraints=[
            (rrcm["position"], rrcm["radius"], rcm_joint_index)],
        relative_needle_pose=needle_tip_to_needle_center,
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
p1 = sim.get_right_tube_target_point()
sim.set_frame("p1", p1)
needle_center = rrobot.fkm(q) * needle_tip_to_needle_center
sim.clear_frames()
for i in range(50):

    angle = 0.001 * i * math.pi / 2.0
    rotate = math.cos(angle / 2.0) + math.sin(angle / 2.0) * k_

    translate = 1 + 0.5 * E_ * k_ * 0.00001

    if i < 1500:
        xdc = needle_center * rotate * needle_center_to_needle_tip

    x = rrobot.fkm(q)
    needle_center = x * needle_tip_to_needle_center
    sim.set_frame("xd_new", xdc)
    sim.set_frame("x", x)
    sim.set_frame("p1", p1)
    sim.set_frame("needle_center", needle_center)
    sim.set_frame("plane_1", needle_center * (1 + 0.5*E_*0.001*k_))
    sim.set_frame("plane_2", needle_center * (1 + 0.5*E_*-0.001*k_))
    sim.set_frame("needle_rotate", needle_center * rotate)

    for step in range(CONTROLLER_STEPS):
        # Solve the quadratic program
        u = needle_driving_1_controller.compute_setpoint_control_signal(q, xdc)

        # Update the current joint positions
        q = q + u * CONTROLLER_SAMPLING_TIME
        print(f"Last error norm {np.linalg.norm(needle_driving_1_controller.get_last_error())}")

    sim.set_right_robot_joints(q)

    time.sleep(1/60)
print("Needle driving loop finished.")

print("Starting needle driving loop...")
q = sim.get_right_robot_joints()
p1 = sim.get_right_tube_target_point()
p2 = sim.get_left_tube_target_point()
sim.set_frame("p1", p1)
sim.set_frame("p2", p2)
needle_center = rrobot.fkm(q) * needle_tip_to_needle_center
sim.clear_frames()

needle_driving_1_controller = NeedleController(
        kinematics=rrobot,
        gain=100.0,
        damping=np.diag([1,1,1,1,1,1,0.000001,0.000001,0.000001]),
        alpha=0.999,
        rcm_constraints=[
            (rrcm["position"], rrcm["radius"], rcm_joint_index)],
        relative_needle_pose=needle_tip_to_needle_center,
        vessel_positions=[translation(p1), translation(p2)],
        vessel_normals=[v, v],
        needle_radius=radius,
        d_safe_angles=np.pi / 8.0,
        vfi_gain=1.0,
        verbose=True,
        insertion_constraints=False
    )

for i in range(50):

    angle = 0.001 * i * math.pi / 2.0
    rotate = math.cos(angle / 2.0) + math.sin(angle / 2.0) * k_

    translate = 1 + 0.5 * E_ * k_ * 0.00001

    if i < 1500:
        xdc = needle_center * rotate * needle_center_to_needle_tip

    x = rrobot.fkm(q)
    needle_center = x * needle_tip_to_needle_center
    sim.set_frame("xd_new", xdc)
    sim.set_frame("x", x)
    sim.set_frame("p1", p1)
    sim.set_frame("p2", p2)
    sim.set_frame("needle_center", needle_center)
    sim.set_frame("plane_1", needle_center * (1 + 0.5*E_*0.001*k_))
    sim.set_frame("plane_2", needle_center * (1 + 0.5*E_*-0.001*k_))
    sim.set_frame("needle_rotate", needle_center * rotate)

    for step in range(CONTROLLER_STEPS):
        # Solve the quadratic program
        u = needle_driving_1_controller.compute_setpoint_control_signal(q, xdc)

        # Update the current joint positions
        q = q + u * CONTROLLER_SAMPLING_TIME
        print(f"Last error norm {np.linalg.norm(needle_driving_1_controller.get_last_error())}")

    sim.set_right_robot_joints(q)

    time.sleep(1/60)
print("Needle driving loop finished.")

sim.disconnect()
