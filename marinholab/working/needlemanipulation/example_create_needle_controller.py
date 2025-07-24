"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""
from importlib.resources import files
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulatorDH

from marinholab.working.needlemanipulation import NeedleController
from marinholab.working.needlemanipulation.example_load_from_file import get_information_from_file

def main():
    lrobot, lrcm1, lrcm2 = get_information_from_file(
        files('marinholab.working.needlemanipulation').joinpath('left_robot.yaml').read_text())

    lower_q_limit = [-85, -85, 5, -265, -85, -355, -170, -30, -30]
    upper_q_limit = [85, 85, 120, 0, 85, 355, 170, 30, 30]
    lrobot.set_lower_q_limit(lower_q_limit)
    lrobot.set_upper_q_limit(upper_q_limit)
    DQ_SerialManipulatorDH.get_lower_q_limit()
    controller = NeedleController(
        kinematics=lrobot,
        gain=10.0,
        damping=0.01,
        alpha=0.999,
        rcm_constraints=[
            (lrcm1["position"], lrcm1["radius"], 6),
            (lrcm2["position"], lrcm2["radius"], 6)],
        relative_needle_pose=DQ([1]),
        vessel_position=DQ([1,2,3]),
        needle_radius=0.003
    )

if __name__ == "__main__":
    main()