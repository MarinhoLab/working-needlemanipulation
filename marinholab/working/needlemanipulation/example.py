"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
MIT License

Example: build a simple 3-DOF serial manipulator with
:class:`M3_SerialManipulatorSimulatorFriendly` and, when
``dqrobotics_extensions`` and ``matplotlib`` are available, plot it in 3D.
"""
from dqrobotics import *
from marinholab.working.needlemanipulation import M3_SerialManipulatorSimulatorFriendly

try:
    from matplotlib import pyplot as plt
    import dqrobotics_extensions.pyplot as dqp
except ImportError:
    dqp = None


def main() -> None:
    """Build a 3-DOF robot and (optionally) plot it.

    The three joints have offsets that rotate about the X, Y, Z axes with a
    0.5 m translation along each axis before actuation.

    When the plotting backend is importable the robot is drawn at the home
    configuration and the figure blocks until closed.
    """
    offsets_before = [
        1 + 0.5*E_*i_,
        1 + 0.5*E_*j_,
        1 + 0.5*E_*k_,
    ]

    offsets_after = [
        i_,
        j_,
        k_
    ]

    actuation_types = [
        M3_SerialManipulatorSimulatorFriendly.ActuationType.RX,
        M3_SerialManipulatorSimulatorFriendly.ActuationType.RY,
        M3_SerialManipulatorSimulatorFriendly.ActuationType.RZ
    ]

    robot = M3_SerialManipulatorSimulatorFriendly(
        offsets_before,
        offsets_after,
        actuation_types
    )

    if dqp is not None:
        q = [0, 0, 0]
        # Set up the plot
        plt.figure()
        plot_size = 1
        ax = plt.axes(projection='3d')
        ax.set_xlabel('$x$')
        ax.set_xlim((-plot_size, plot_size))
        ax.set_ylabel('$y$')
        ax.set_ylim((-plot_size, plot_size))
        ax.set_zlabel('$z$')
        ax.set_zlim((-plot_size, plot_size))
        dqp.plot(robot, q=q)
        plt.show(block=True)

if __name__ == "__main__":
    main()