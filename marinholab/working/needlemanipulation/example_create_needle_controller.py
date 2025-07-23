"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""
from importlib.resources import files
from dqrobotics import *
from marinholab.working.needlemanipulation import NeedleController
from marinholab.working.needlemanipulation.example_load_from_file import get_information_from_file

def main():
    lrobot, lrcm1, lrcm2 = get_information_from_file(
        files('marinholab.working.needlemanipulation').joinpath('left_robot.yaml').read_text())

    controller = NeedleController(
        kinematics=lrobot,
        gain=10.0,
        damping=0.01,
        alpha=0.999,
        rcm_constraints=[
            (lrcm1["position"], lrcm1["radius"]),
            (lrcm2["position"], lrcm2["radius"])],
        relative_needle_pose=DQ([1]),
        vessel_position=DQ([1,2,3]),
        needle_radius=0.003
    )

if __name__ == "__main__":
    main()