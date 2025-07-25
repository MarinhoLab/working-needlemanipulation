import math

from importlib.resources import files

import dqrobotics as dq
from dqrobotics.utils.DQ_Math import deg2rad
import numpy as np
from dqrobotics import rotation

import PedriatricSimulator
import time
from marinholab.working.needlemanipulation.example_load_from_file import get_information_from_file
from marinholab.working.needlemanipulation.icra2019_controller import ICRA19TaskSpaceController
from marinholab.working.needlemanipulation import NeedleController

from marinholab.working.needlemanipulation import M3_SerialManipulatorSimulatorFriendly
import yaml

rcm1_joint_index = 7
rcm2_joint_index = 6

sim = PedriatricSimulator.PediatricSimulator()
sim.connect("127.0.0.1")

sim.set_left_robot_target_pose(sim.get_left_robot_effector())
sim.set_right_robot_target_pose(sim.get_right_robot_effector())

sim.restart()
sim.clear_frames()

time.sleep(5)

def make_robot(base_frame, transforms):
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

rrobot = make_robot(sim.get_right_robot_base(), [sim.get_right_robot_model_ith(k) for k in range(dofs)])

lower_q_limit = deg2rad([-85, -85, 5, -265, -85, -355, -170, -30, -30])
upper_q_limit = deg2rad([85, 85, 120, 0, 85, 355, 170, 30, 30])
rrobot.set_lower_q_limit(lower_q_limit)
rrobot.set_upper_q_limit(upper_q_limit)
current_q = sim.get_left_robot_joints()
print(current_q - lower_q_limit)
print(upper_q_limit - current_q)

rrcm1 = {"position": sim.get_left_center_sphere()[0], "radius": sim.get_left_center_sphere()[1]}
rrcm2 = {"position": sim.get_right_trocar_sphere()[0], "radius": sim.get_right_trocar_sphere()[1]}
print(rrcm1)
print(rrcm2)

controller = ICRA19TaskSpaceController(
    kinematics=rrobot,
    gain=100.0,
    damping=np.diag([1,1,1,1,1,1,0,0,0]),
    alpha=0.9999,
    rcm_constraints=[
        (rrcm1["position"], 1.0 * rrcm1["radius"], rcm1_joint_index),
        (rrcm2["position"], 1.0 * rrcm2["radius"], rcm2_joint_index)],
    vfi_gain=2.0
)

sim.set_frame("rcm1", 1 + 0.5 * dq.E_* rrcm1["position"])
sim.set_frame("rcm2", 1 + 0.5 * dq.E_* rrcm2["position"])

#open forceps
sim.set_right_robot_target_grip_angle(15.0)
time.sleep(1.0)

# approach needle
q = sim.get_right_robot_joints()
for i in range(100):
    needle_offset = 0.5 + 0.5 * (dq.i_ - dq.j_ - dq.k_)
    xd = sim.get_needle_frame_at(0.9) * needle_offset
    #sim.set_frame("needle_xd", xd)

    tip_offset = 1 + 0.5 * dq.E_ * (0.0015 * -dq.k_)
    x = sim.get_right_robot_effector() * tip_offset
    #sim.set_frame("tip_x", x)

    ex = dq.translation(xd).vec3() - dq.translation(x).vec3()
    er = dq.conj(dq.rotation(x)) * dq.rotation(xd)

    #elen = np.linalg.norm(ex)
    #if elen < 0.0001 and sim.is_needle_between_right_forceps_tips():
    #    break
    dx = 1.0 * ex
    dr = dq.pow(er, 1.0)

    translate = 1 + 0.5 * dq.E_ * (dq.i_ * dx[0] + dq.j_ * dx[1] + dq.k_ * dx[2])

    xdc = translate * sim.get_right_robot_effector() * dr

    sim.set_frame("xd", xdc)
    sim.set_frame("x", sim.get_right_robot_effector())

    # Loop parameters
    sampling_time = 0.008

    q = sim.get_right_robot_joints()
    sim.set_frame("rcm1_shaft", rrobot.fkm(q, rcm1_joint_index))
    sim.set_frame("rcm2_shaft", rrobot.fkm(q, rcm2_joint_index))
    for step in range(10):
        # Solve the quadratic program
        u = controller.compute_setpoint_control_signal(q, xdc)

        # Update the current joint positions
        q = q + u * sampling_time

    sim.set_right_robot_joints(q)

    time.sleep(1/60)

#close forceps
sim.set_right_robot_target_grip_angle(0.0)
time.sleep(1.0)

# extracting needle ???

needle_pose = sim.get_control_needle_pose()
ee = sim.get_right_robot_effector()
relative_needle_pose = dq.conj(ee) * needle_pose
vessel_point = dq.translation(sim.get_right_tube_current_point())
radius = sim.get_needle_radius()

sim.set_frame("vessel_point", sim.get_right_tube_current_point())

needle_controller = NeedleController(
        kinematics=rrobot,
        gain=100.0,
        damping=np.diag([1,1,1,1,1,1,0,0,0]),
        alpha=0.9999,
        rcm_constraints=[
            (rrcm1["position"], rrcm1["radius"], rcm1_joint_index),
            (rrcm2["position"], rrcm2["radius"], rcm2_joint_index)],
        relative_needle_pose=relative_needle_pose,
        vessel_position=vessel_point,
        needle_radius=radius,
        vfi_gain=2,
        verbose=True
    )


q = sim.get_right_robot_joints()
for i in range(10000):
    sim.set_frame("needle_pose", sim.get_control_needle_pose())

    x = sim.get_right_robot_effector()
    translate = 1 + 0.5 * dq.E_ * (dq.i_ * -0.005 + dq.j_ * 0.01 + dq.k_ * 0.0)
    angle = 0.1 * -math.pi / 2.0
    rotate = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (dq.i_ * 0.0 + dq.j_ * 0.0 + dq.k_ * 1.0)
    xdc = translate * x * rotate

    sim.set_frame("xd", xdc)
    sim.set_frame("x", sim.get_right_robot_effector())

    # Loop parameters
    sampling_time = 0.008

    q = sim.get_right_robot_joints()
    for step in range(10):
        # Solve the quadratic program
        u = needle_controller.compute_setpoint_control_signal(q, xdc)

        # Update the current joint positions
        q = q + u * sampling_time

    sim.set_right_robot_joints(q)

    time.sleep(1/60)

sim.disconnect()
