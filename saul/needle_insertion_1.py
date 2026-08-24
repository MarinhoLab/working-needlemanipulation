import math
import os
from gettext import translation

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
    sim.load_simulation_state(project_path + "/before_needle_insertion_1.simstate")
    time.sleep(1)

print("needle radius {}".format(sim.get_needle_radius()))


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

translate = 1 + 0.5 * dq.E_ * dq.j_ * 0.002
angle = -math.pi / 2.0
rotate1 = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (dq.i_ * 1.0 + dq.j_ * 0.0 + dq.k_ * 0.0)
angle = math.pi / 2.0
rotate2 = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (dq.i_ * 0.0 + dq.j_ * 0.0 + dq.k_ * 1.0)

p1 = translate * sim.get_right_tube_target_point() * rotate1 * rotate2
sim.set_frame("p1", p1)

needle_tip_pose = sim.get_needle_frame_at(0.0)

x = sim.get_right_robot_effector()

x_wrt_npt = dq.conj(needle_tip_pose) * x
xd = p1 * x_wrt_npt

sim.set_frame("xd", xd)

# approach needle to tissue
for i in range(200):
    sim.set_frame("xd", xd)

    needle_tip_pose = sim.get_needle_frame_at(0.0)
    sim.set_frame("needle_tip_pose", needle_tip_pose)

    x = sim.get_right_robot_effector()
    sim.set_frame("x", x)

    ex = dq.translation(xd).vec3() - dq.translation(x).vec3()
    er = dq.conj(dq.rotation(x)) * dq.rotation(xd)
    if er.q[0] < 0.0:
        er = -er

    # elen = np.linalg.norm(ex)
    # if elen < 0.0001 and sim.is_needle_between_right_forceps_tips():
    #    break
    dx = 0.5 * ex
    dr = dq.pow(er, 0.05)

    translate = 1 + 0.5 * dq.E_ * (dq.i_ * dx[0] + dq.j_ * dx[1] + dq.k_ * dx[2])

    xdc = translate * sim.get_right_robot_effector() * dr
    sim.set_frame("xdc", xdc)

    sim.set_right_robot_target_pose(xdc)

    time.sleep(1 / 60)





sim.clear_frames()

needle_frame = sim.get_control_needle_pose()
x = sim.get_right_robot_effector()
x_wrt_needle_frame = dq.conj(needle_frame) * x

# insert needle
for i in range(300):

    angle = 0.001 * i * math.pi / 2.0
    rotate = math.cos(angle / 2.0) + math.sin(angle / 2.0) * (dq.i_ * 0.0 + dq.j_ * 0.0 + dq.k_ * 1.0)

    translate = 1 + 0.5 * dq.E_ * (dq.i_ * 0.0 + dq.j_ * -0.00001 * i + dq.k_ * 0.0)

    xd = translate * needle_frame * rotate * x_wrt_needle_frame

    sim.set_frame("xd", xd)
    sim.set_frame("x", sim.get_right_robot_effector())

    sim.set_right_robot_target_pose(xd)

    time.sleep(1/60)


sim.disconnect()
