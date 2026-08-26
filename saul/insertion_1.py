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
import os


from importlib.resources import files

import dqrobotics as dq
from dqrobotics.utils.DQ_Math import deg2rad
import numpy as np
from dqrobotics import conj, E_, translation

import PedriatricSimulator
import time
from marinholab.working.needlemanipulation.example_load_from_file import get_information_from_file
from marinholab.working.needlemanipulation.icra2019_controller import ICRA19TaskSpaceController
from marinholab.working.needlemanipulation import NeedleController

from marinholab.working.needlemanipulation import M3_SerialManipulatorSimulatorFriendly


rcm1_joint_index = 7
rcm2_joint_index = 6

sim = PedriatricSimulator.PediatricSimulator()
sim.connect("127.0.0.1")
#sim.restart()
sim.clear_frames()
time.sleep(1)

#project_path = os.getcwd()
#sim.load_simulation_state(project_path + "/before_insertion.simstate")
#time.sleep(5)


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
            offsets_before.append(1 + 0 * dq.E_)
        actuation_types.append(M3_SerialManipulatorSimulatorFriendly.ActuationType.RX)

    return M3_SerialManipulatorSimulatorFriendly(
        offsets_before,
        offsets_after,
        actuation_types
    )

dofs = sim.get_robot_dofs()

lrobot = make_robot(sim.get_left_robot_base(), [sim.get_left_robot_model_ith(k) for k in range(dofs)])
rrobot = make_robot(sim.get_right_robot_base(), [sim.get_right_robot_model_ith(k) for k in range(dofs)])

lower_q_limit = deg2rad([-85, -85, 5, -265, -85, -355, -170, -30, -30])
upper_q_limit = deg2rad([85, 85, 120, 0, 85, 355, 170, 30, 30])

lrobot.set_lower_q_limit(lower_q_limit)
lrobot.set_upper_q_limit(upper_q_limit)
rrobot.set_lower_q_limit(lower_q_limit)
rrobot.set_upper_q_limit(upper_q_limit)

lrcm1 = {"position": sim.get_left_center_sphere()[0], "radius": sim.get_left_center_sphere()[1]}
lrcm2 = {"position": sim.get_left_trocar_sphere()[0], "radius": sim.get_left_trocar_sphere()[1]}
rrcm1 = {"position": sim.get_right_center_sphere()[0], "radius": sim.get_right_center_sphere()[1]}
rrcm2 = {"position": sim.get_right_trocar_sphere()[0], "radius": sim.get_right_trocar_sphere()[1]}

translate = 1 + 0.5 * dq.E_ * dq.j_ * 0.0005
angle = -math.pi / 2.0
rotate1 = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (dq.i_ * 1.0 + dq.j_ * 0.0 + dq.k_ * 0.0)
angle = math.pi / 2.0
rotate2 = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (dq.i_ * 0.0 + dq.j_ * 0.0 + dq.k_ * 1.0)

p1 = translate * sim.get_right_tube_target_point() * rotate1 * rotate2
sim.set_frame("p1", p1)

needle_pose = sim.get_control_needle_pose()
q = sim.get_right_robot_joints()
ee = rrobot.fkm(q)
relative_needle_pose = dq.conj(ee) * needle_pose
radius = sim.get_needle_radius()


needle_tip_pose = sim.get_needle_frame_at(0.0)
relative_needle_tip_pose = dq.conj(ee) * needle_tip_pose

rrobot.set_effector(relative_needle_tip_pose)

v = dq.j_
n = dq.Ad(dq.rotation(needle_pose), dq.k_)
n1 = dq.Ad(dq.rotation(p1), dq.k_)

needle_controller = NeedleController(
        kinematics=rrobot,
        gain=100.0,
        damping=np.diag([1,1,1,1,1,1,0,0,0]),
        alpha=1.0,
        rcm_constraints=[
            (rrcm1["position"], rrcm1["radius"], rcm1_joint_index),
            (rrcm2["position"], rrcm2["radius"], rcm2_joint_index)],
        relative_needle_pose=relative_needle_pose,
        #vessel_positions=[dq.translation(p1), dq.translation(p2)],
        #vessel_normals=[n1, n2],
        vessel_positions=None,
        vessel_normals=None,
        needle_radius=None,
        d_safe_angles=None,
        vfi_gain=1.0,
        verbose=True,
        insertion_constraints=True
    )

q = sim.get_right_robot_joints()
xdc = None
for i in range(300):
    print(i)
    sim.set_frame("needle", sim.get_control_needle_pose())

    x = rrobot.fkm(q)
    sim.set_frame("x", x)

    # controlled target pose
    xdc = p1
    sim.set_frame("xd", xdc)

    sampling_time = 0.008
    for step in range(10):
        # Solve the quadratic program
        u = needle_controller.compute_setpoint_control_signal(q, xdc)

        # Update the current joint positions
        q = q + u * sampling_time
        print(f"Last error norm {np.linalg.norm(needle_controller.get_last_error())}")

    sim.set_right_robot_joints(q)

    time.sleep(1.0 / 60.0)








sim.clear_frames()

translate = 1 + 0.5 * dq.E_ * dq.j_ * 0.0005
angle = -math.pi / 2.0
rotate1 = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (dq.i_ * 1.0 + dq.j_ * 0.0 + dq.k_ * 0.0)
angle = math.pi / 2.0
rotate2 = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (dq.i_ * 0.0 + dq.j_ * 0.0 + dq.k_ * 1.0)

p1 = translate * sim.get_right_tube_target_point() * rotate1 * rotate2

needle_controller = NeedleController(
        kinematics=rrobot,
        gain=2000.0,
        damping=np.diag([1,1,1,1,1,1,0,0,0]),
        alpha=1.0,
        rcm_constraints=[
            (rrcm1["position"], rrcm1["radius"], rcm1_joint_index),
            (rrcm2["position"], rrcm2["radius"], rcm2_joint_index)],
        relative_needle_pose=relative_needle_pose,
        vessel_positions=[translation(p1)],
        vessel_normals=[v],
        needle_radius=radius,
        d_safe_angles=np.pi / 8.0,
        vfi_gain=1.0,
        verbose=True,
        insertion_constraints=False
    )

q = sim.get_right_robot_joints()
for i in range(100):
    sampling_time = 0.008
    for step in range(10):
        # Solve the quadratic program
        u = needle_controller.compute_setpoint_control_signal(q, xdc)

        # Update the current joint positions
        q = q + u * sampling_time
        print(f"Last error norm {np.linalg.norm(needle_controller.get_last_error())}")

    sim.set_right_robot_joints(q)

    time.sleep(1.0 / 60.0)

# insert needle
q = sim.get_right_robot_joints()
needle_center = rrobot.fkm(q) * conj(relative_needle_pose)
for i in range(500):

    angle = 0.001 * i * math.pi / 2.0
    rotate = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (dq.i_ * 0.0 + dq.j_ * 1.0 + dq.k_ * 0.0)

    translate = 1 + 0.5 * dq.E_ * (dq.i_ * 0.0 + dq.j_ * 0.00001 * i + dq.k_ * 0.0)

    if i < 1500:
        xdc = needle_center * rotate * relative_needle_pose

    sim.set_frame("xd_new", xdc)
    sim.set_frame("x", x)
    sim.set_frame("p1", p1)
    sim.set_frame("needle_center", needle_center)
    sim.set_frame("needle_rotate", needle_center * rotate)

    sampling_time = 0.008
    for step in range(10):
        # Solve the quadratic program
        u = needle_controller.compute_setpoint_control_signal(q, xdc)

        # Update the current joint positions
        q = q + u * sampling_time
        print(f"Last error norm {np.linalg.norm(needle_controller.get_last_error())}")

    sim.set_right_robot_joints(q)

    time.sleep(1/60)



sim.disconnect()
