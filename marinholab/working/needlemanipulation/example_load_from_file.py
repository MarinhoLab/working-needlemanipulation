from importlib.resources import files
import yaml
from dqrobotics import *
from marinholab.working.needlemanipulation import M3_SerialManipulatorSimulatorFriendly
from marinholab.working.needlemanipulation.icra2019_controller import ICRA19TaskSpaceController
try:
    from matplotlib import pyplot as plt
    import dqrobotics_extensions.pyplot as dqp
except ImportError:
    dqp = None

def get_information_from_file(file_contents: str) -> (M3_SerialManipulatorSimulatorFriendly, tuple[DQ, float], tuple[DQ, float]):
    """
    The actuation types must be a list of strings. Currently, only 'RX' is accepted.
    The offsets must be a list of DQ objects. They will be normalized.

    :param file_contents: The file after .read() was applied in a suitable format.
    :return: A M3_SerialManipulatorSimulatorFriendly object.
    """
    data_loaded = yaml.safe_load(file_contents)

    # The actuation types are received as strings and should be converted.
    actuation_types = [M3_SerialManipulatorSimulatorFriendly.ActuationType.RX if a == "RX" else None for a in data_loaded["actuation_types"]]
    if None in actuation_types:
        raise RuntimeError("Only RX is accepted in this example.")

    # The dual quaternions are received as lists so must be converted to DQs
    offsets_before = [DQ(x).normalize() for x in data_loaded["offsets_before"]]
    offsets_after = [DQ(x).normalize() for x in data_loaded["offsets_after"]]

    robot = M3_SerialManipulatorSimulatorFriendly(
        offsets_before,
        offsets_after,
        actuation_types
    )

    rcm1 = {
        "position": DQ(data_loaded["rcm1"][0]),
        "radius": data_loaded["rcm1"][1]
    }
    rcm2 = {
        "position": DQ(data_loaded["rcm2"][0]),
        "radius": data_loaded["rcm2"][1]
    }

    return robot, rcm1, rcm2

def example_plot(q, robot, rcm1, rcm2):
    """
    Plots a 3D representation of a robot's configuration along with two red and blue spherical
    regions of constraint.

    Args:
        q: The configuration of the robot as an array or similar structure.
        robot: The robot object whose 3D pose is to be plotted.
        rcm1: A dictionary representing the first region of constraint in 3D space. Must
            include keys "position" for coordinates and "radius" for sphere size.
        rcm2: A dictionary representing the second region of constraint in 3D space. Must
            include keys "position" for coordinates and "radius" for sphere size.
    """
    plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('$x$')
    ax.set_ylabel('$y$')
    ax.set_zlabel('$z$')

    dqp.plot(robot, q=q)
    dqp.plot(rcm1["position"], sphere=True, radius=rcm1["diameter"], color="red", alpha=0.5)
    dqp.plot(rcm2["position"], sphere=True, radius=rcm2["diameter"], color="blue", alpha=0.5)

    plt.show(block=True)

def main():

    try:
        lrobot, lrcm1, lrcm2 = get_information_from_file(files('marinholab.working.needlemanipulation').joinpath('left_robot.yaml').read_text())

        controller = ICRA19TaskSpaceController(
            kinematics=lrobot,
            gain=10.0,
            damping=0.01,
            alpha=0.999,
            rcm_constraints=[
                (lrcm1["position"], lrcm1["radius"]),
                (lrcm2["position"], lrcm2["radius"])]
        )

        q_init = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        x_init = lrobot.fkm(q_init)

        if dqp is not None:
            example_plot(q_init, lrobot, lrcm1, lrcm2)

        # Loop parameters
        sampling_time = 0.008

        q = q_init
        while True:
            xd = x_init # Replace this with your xd calculation

            # Solve the quadratic program
            u = controller.compute_setpoint_control_signal(q, xd)

            # Update the current joint positions
            q = q + u * sampling_time
    except KeyboardInterrupt:
        print("main::KeyboardInterrupt")

if __name__ == '__main__':
    main()