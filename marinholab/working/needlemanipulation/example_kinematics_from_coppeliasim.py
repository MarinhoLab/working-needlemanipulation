"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""
import time
import numpy as np
from math import sin, cos

from dqrobotics import *
from dqrobotics.robot_control import DQ_PseudoinverseController, ControlObjective
from dqrobotics.interfaces.coppeliasim import DQ_CoppeliaSimInterfaceZMQ
from marinholab.working.needlemanipulation import M3_SerialManipulatorSimulatorFriendly


def main():

    ci = DQ_CoppeliaSimInterfaceZMQ()
    if not ci.connect():
        raise Exception("Could not connect to CoppeliaSim.")

    ci.start_simulation()

    joint_names = [
        '/UR3/joint',
        '/UR3/joint/link/joint',
        '/UR3/joint/link/joint/link/joint',
        '/UR3/joint/link/joint/link/joint/link/joint',
        '/UR3/joint/link/joint/link/joint/link/joint/link/joint',
        '/UR3/joint/link/joint/link/joint/link/joint/link/joint/link/joint',
    ]

    # Poses of each frame
    x = []
    # Transformation before the actuation
    x_before = []
    # Transformation after the actuation
    x_after = []

    q_init = ci.get_joint_positions(joint_names)
    x_end_effector = ci.get_object_pose("/UR3/connection")

    # We have to consider the transformation between frames but removing the transformation
    # of the joint

    def rot_z(angle):
        return cos(angle/2) + k_*sin(angle/2)

    for joint_name in joint_names:
        x.append(ci.get_object_pose(joint_name))

    x_before.append(x[0] * conj(rot_z(q_init[0])))
    x_after.append(DQ([1]))
    for i in range(1, len(q_init)):
        x_before.append(conj(x[i-1]) * x[i] * conj(rot_z(q_init[i])))
        x_after.append(DQ([1]))

    # Just the last joint has a nontrivial x_after
    x_after[-1] = conj(x[-1]) * x_end_effector

    robot = M3_SerialManipulatorSimulatorFriendly(
        x_before,
        x_after,
        [M3_SerialManipulatorSimulatorFriendly.ActuationType.RZ] * 6
    )

    ci.set_object_pose("x_calculated", robot.fkm(q_init))
    time.sleep(5)

    ci.set_joint_target_positions(joint_names, q_init + 1)
    time.sleep(1)
    x = robot.fkm(q_init + 1)
    ci.set_object_pose("x_calculated", x)
    e = conj(ci.get_object_pose("/UR3/connection"))*x
    print(f"Fkm error {np.linalg.norm(vec8(e))}")
    time.sleep(5)

    x = robot.fkm(q_init - 1)
    ci.set_joint_target_positions(joint_names, q_init - 1)
    time.sleep(1)
    ci.set_object_pose("x_calculated", robot.fkm(q_init - 1))
    e = conj(ci.get_object_pose("/UR3/connection"))*x
    print(f"Fkm error {np.linalg.norm(vec8(e))}")
    time.sleep(5)

    # Control

    controller = DQ_PseudoinverseController(robot)
    controller.set_control_objective(ControlObjective.Pose)
    controller.set_gain(1.0)
    controller.set_damping(0.01)
    #controller = ICRA19TaskSpaceController(
    #    kinematics=robot,
    #    gain=1.0,
    #    damping=0.01,
    #    alpha=0.999,
    #    rcm_constraints=None
    #)

    t_final = 10.0
    t = 0

    # Loop parameters
    sampling_time = 0.008

    q = ci.get_joint_positions(joint_names)
    xd = robot.fkm(q_init)
    ci.set_object_pose("x_d", xd)
    while t < t_final:
        # Solve the quadratic program
        u = controller.compute_setpoint_control_signal(q, vec8(xd))
        print(np.linalg.norm(controller.get_last_error_signal()))

        # Update the current joint positions
        q = q + u * sampling_time
        t = t + sampling_time
        ci.set_joint_target_positions(joint_names, q)
        time.sleep(sampling_time)

    ci.stop_simulation()


if __name__ == '__main__':
    main()