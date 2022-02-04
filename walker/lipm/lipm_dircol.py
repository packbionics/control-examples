import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import DirectCollocation, MathematicalProgram, Solve, PiecewisePolynomial
from pydrake.examples.pendulum import PendulumPlant, PendulumState

def pend_walker_dircol():
    """
    Finding limit cycle using Direct Collocation for an actuated linear inverted pendulum.
    Incorporates impact map parameterized by impact angle alpha.

    In summary, the program finds torque inputs that produce a limit cycle for the pendulum,
        if you imagine that a new pendulum stick with angle alpha "catches" 
        the pendulum at the impact point everytime.
    """
    plant = PendulumPlant()
    context = plant.CreateDefaultContext()

    N = 21
    max_dt = 0.5
    max_tf = N * max_dt
    dircol = DirectCollocation(plant, 
                            context, 
                            num_time_samples=N, 
                            minimum_timestep=0.05,
                            maximum_timestep=max_tf)

    prog = dircol.prog()
    dircol.AddEqualTimeIntervalsConstraints()

    # Torque constraints
    torque_limit = 3.0 # N*m
    u = dircol.input()
    dircol.AddConstraintToAllKnotPoints(-torque_limit <= u[0])
    dircol.AddConstraintToAllKnotPoints(u[0] <= torque_limit)

    alpha = 20*np.pi/180

    # State constraints
    dircol.AddConstraintToAllKnotPoints(dircol.state()[0] >= np.pi-alpha)
    dircol.AddConstraintToAllKnotPoints(dircol.state()[0] <= np.pi+alpha)

    # Boundary constraints
    prog.AddConstraint(dircol.initial_state()[0] == np.pi-alpha)
    prog.AddConstraint(dircol.initial_state()[1] == 1.0)
    prog.AddConstraint(dircol.final_state()[0] == np.pi+alpha)

    # Impact map, based on rimless wheel model
    prog.AddConstraint(dircol.initial_state()[1] == dircol.final_state()[1] *
                        np.cos(2 * alpha))

    # Cost on input effort
    R = 10
    dircol.AddRunningCost(R * u[0]**2)

    # Initial guess
    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
        [0., 4.], [[np.pi-alpha, 1.],
                [np.pi+alpha,1.]])
    dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

    # Solve
    result = Solve(prog)
    assert result.is_success()

    # Plot
    x_trajectory = dircol.ReconstructStateTrajectory(result)
    u_trajectory = dircol.ReconstructInputTrajectory(result)

    fig, ax = plt.subplots()

    x_knots = np.hstack([
        x_trajectory.value(t) for t in np.linspace(x_trajectory.start_time(),
                                                x_trajectory.end_time(), 100)
    ])

    ax.set_xlabel('$q$')
    ax.set_ylabel('$\dot{q}$')
    ax.plot(x_knots[0, :], x_knots[1, :])
    ax.set_title('Pendulum Trajectory')

    fig2, ax2 = plt.subplots()

    u_knots = np.hstack([
        u_trajectory.value(t) for t in np.linspace(u_trajectory.start_time(),
                                                u_trajectory.end_time(), 100)
    ])

    ax2.set_xlabel('$t$')
    ax2.set_ylabel('$u$')
    ax2.plot(np.linspace(u_trajectory.start_time(), u_trajectory.end_time(), 100), u_knots[0,:])
    ax2.set_title('Input')
    plt.show()

def main():
    pend_walker_dircol()

if __name__ == "__main__":
    main()