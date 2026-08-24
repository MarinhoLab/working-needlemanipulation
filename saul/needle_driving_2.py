import math
import os

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
sim.clear_frames()

if __name__ == "__main__":
    sim.restart()
    time.sleep(1)
    project_path = os.getcwd()
    sim.load_simulation_state(project_path + "/before_needle_driving_1.simstate")
    time.sleep(1)

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

sim.set_frame("rcm1", 1+0.5*dq.E_*rrcm1["position"])
sim.set_frame("rcm2", 1+0.5*dq.E_*rrcm2["position"])

p1 = sim.get_right_tube_target_point()
p2 = sim.get_left_tube_target_point()
sim.set_frame("p1", p1)
sim.set_frame("p2", p2)

needle_pose = sim.get_control_needle_pose()
ee = sim.get_right_robot_effector()
relative_needle_pose = dq.conj(ee) * needle_pose
radius = sim.get_needle_radius()

rrobot.set_effector(relative_needle_pose)

v = dq.j_
n = dq.Ad(dq.rotation(needle_pose), dq.k_)
n1 = dq.Ad(dq.rotation(p1), dq.k_)

needle_controller = NeedleController(
        kinematics=rrobot,
        gain=2000.0,
        damping=np.diag([1,1,1,1,1,1,0,0,0]),
        alpha=0.9999,
        rcm_constraints=[
            (rrcm1["position"], rrcm1["radius"], rcm1_joint_index),
            (rrcm2["position"], rrcm2["radius"], rcm2_joint_index)],
        relative_needle_pose=1+dq.E_*0.0,
        vessel_positions=[dq.translation(p1), dq.translation(p2)],
        vessel_normals=[v, v],
        needle_radius=radius,
        d_safe_angles=np.pi / 8.0,
        vfi_gain=1.0,
        verbose=False
    )

q = sim.get_right_robot_joints()
for i in range(1000):
    print(i)
    sim.set_frame("needle", sim.get_control_needle_pose())

    angle = 0.001 * i * math.pi / 2.0
    rotate = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (dq.i_ * 0.0 + dq.j_ * 0.0 + dq.k_ * 1.0)
    tr = 1 + 0.5 * dq.E_ * dq.j_ * 0.001
    x = rrobot.fkm(q)
    xd = tr * needle_pose * rotate

    sim.set_frame("x", x)
    sim.set_frame("xd", xd)

    sampling_time = 0.008

    #q = sim.get_right_robot_joints()

    for step in range(10):
        # Solve the quadratic program
        u = needle_controller.compute_setpoint_control_signal(q, xd)

        # Update the current joint positions
        q = q + u * sampling_time

    sim.set_right_robot_joints(q)

    time.sleep(1.0 / 60.0)



sim.disconnect()
