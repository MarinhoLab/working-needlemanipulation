"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""
from importlib.resources import files
import yaml
from dqrobotics import *
from marinholab.working.needlemanipulation import M3_SerialManipulatorSimulatorFriendly
from marinholab.working.needlemanipulation.icra2019_controller import ICRA19TaskSpaceController
try:
    import dqrobotics_extensions.pyplot as dqp
    from matplotlib import pyplot as plt
    import matplotlib.animation as anm  # Matplotlib animation
    from functools import partial  # Need to call functions correctly for matplotlib animations
except ImportError:
    dqp = None

def _set_plot_labels():
    ax = plt.gca()
    ax.set(
        xlabel='x [m]',
        ylabel='y [m]',
        zlabel='z [m]'
    )

def _set_plot_limits(lmin: float = -0.5, lmax: float = 0.5):
    ax = plt.gca()
    ax.set(
        xlim=[lmin, lmax],
        ylim=[lmin, lmax],
        zlim=[lmin, lmax]
    )

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


# Animation function
def animate_robot(n, robot, stored_qs, stored_time):
    """
    Create an animation function compatible with `plt`.
    Adapted from https://marinholab.github.io/OpenExecutableBooksRobotics//lesson-dq8-optimization-based-robot-control.
    :param n: The frame number, necessary for `pyplot`.
    :param robot: The `DQ_SerialManipulator` instance.
    :param stored_qs: The sequence of joint configurations.
    :param stored_time: The sequence of timepoints to plot in the title.
    """
    plt.cla()
    _set_plot_limits(-1.0, 1.0)
    _set_plot_labels()
    plt.title(f'Joint control time={stored_time[n]:.2f} s out of {stored_time[-1]:.2f} s')

    dqp.plot(robot, q=stored_qs[n],
             line_color='b',
             cylinder_color="c",
             cylinder_alpha=0.3)

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

        time_final = 60.0
        time = 0

        if dqp is not None:
            # Store the control signals
            stored_qs = []
            stored_time = []

        # Loop parameters
        sampling_time = 0.008

        q = q_init
        while time < time_final:
            if dqp is not None:
                # Store data for posterior animation
                stored_qs.append(q)
                stored_time.append(time)

            xd = x_init # Replace this with your xd calculation

            # Solve the quadratic program
            u = controller.compute_setpoint_control_signal(q, xd)

            # Update the current joint positions
            q = q + u * sampling_time
            time = time + sampling_time

        if dqp is not None:
            # Set up the plot
            fig = plt.figure(dpi=200, figsize=(12, 10))
            plt.axes(projection='3d')

            print(f"The size of the log is {len(stored_qs)}")
            anim = anm.FuncAnimation(fig,
                                     partial(animate_robot,
                                             robot=lrobot,
                                             stored_qs=stored_qs,
                                             stored_time=stored_time),
                                     interval=1,
                                     frames=len(stored_qs))

            anim.save("output_moving_manipulators.mp4", fps=60)

    except KeyboardInterrupt:
        print("main::KeyboardInterrupt")



if __name__ == '__main__':
    main()