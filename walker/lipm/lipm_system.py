import numpy as np
import time

from pydrake.symbolic import Jacobian, MonomialBasis, Variable, Variables
from pydrake.math import cos, sin, arcsin
from pydrake.all import (MathematicalProgram,
                        SolutionResult,
                        SolverType, 
                        Solve)
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.framework import VectorSystem
from pydrake.trajectories import PiecewisePolynomial

import matplotlib.pyplot as plt


class LIPMSystem(VectorSystem):

    def __init__(self):
        # 1 input (u), 2 outputs (theta, theta_dot)
        super().__init__(1, 2)
        self.num_states = 2
        self.num_inputs = 1
        
        self.g = 9.81
        self.m = 1.0
        self.l = 1.0
        
    def _DoCalcVectorTimeDerivatives(self, context, u, x, x_dot):
        xdot = self.lipm_dynamics(x, u)
        
    def _DoCalcVectorOutput(self, context, u, x, y):
        y[:] = x
        
    def lipm_dynamics(self, x, u):

        xdot = np.zeros_like(x)
            
        xdot[0] = x[1]
        xdot[1] = u / (self.m * self.l**2) - (self.g / self.l) * sin(x[0])
        
        return xdot
    
    def lipm_walk_trajopt(self, x0, xf, N, contact_sequence):
        
        # stopwatch for solver time
        tsolve_pre = time.time()
        
        prog = MathematicalProgram()
        
        # Timesteps
        
        t_f = prog.NewContinuousVariables(1, "t_f")
        dt = t_f[0] / N
        
        # Input and state variables
        u = prog.NewContinuousVariables(1, "u_0")
        input_trajectory = u
        x = prog.NewContinuousVariables(2, "x_0")
        state_trajectory = x
        
        # Instantiate decision variables for each knot point
        for k in range(1, N):
            u = prog.NewContinuousVariables(1, "u_%d" % k)
            x = prog.NewContinuousVariables(2, "x_%d" % k)
            input_trajectory = np.vstack((input_trajectory, u))
            state_trajectory = np.vstack((state_trajectory, x))
            
        # Final state
        x = prog.NewContinuousVariables(2, "x_{}".format(N))
        state_trajectory = np.vstack((state_trajectory, x))
        
        # Final input for dircol ?? first order hold
        # For now use Direct Transcription
        #u = prog.NewContinuousVariables(1, "u_{}".format(N))
        #input_trajectory = np.vstack((input_trajectory, u))
        
        print("Number of decision variables: {}".format(prog.num_vars()))
        
        # Cost function
        R = 10
        def cost(input_trajectory):
            return R * np.square(input_trajectory).sum()
        prog.AddCost(cost, input_trajectory[:,0])
        
        # Initial state constraint
        for i in range(2):
            prog.AddBoundingBoxConstraint(x0[i], x0[i], state_trajectory[0, i])
            
        # Final state constraint
        for i in range(2):
            prog.AddBoundingBoxConstraint(xf[i], xf[i], state_trajectory[-1, i])
            
        # Input constraints
        torque_limit = 3.0
        for i in range(len(input_trajectory[:, 0])):
            prog.AddBoundingBoxConstraint(-torque_limit, torque_limit, input_trajectory[i,0])
            
        # State constraints
        #max_stride_angle = 40.0 * np.pi / 180.0
        #for i in range(len(state_trajectory[:, 0])):
        #    prog.AddBoundingBoxConstraint(np.pi-max_stride_angle, np.pi+max_stride_angle, state_trajectory[i,0])
            
        # Dynamic constraints
        for j in range(1, N+1):
            if j not in contact_sequence:
                dx = dt * self.lipm_dynamics(state_trajectory[j-1,:], input_trajectory[j-1,0])
                for k in range(len(x0)):
                    prog.AddConstraint(state_trajectory[j-1,k] + dx[k] == state_trajectory[j,k])
            else: # impact map
                stride_length = contact_sequence[j]
                impact_angle = arcsin(stride_length / (2*self.l))
                # new pendulum rod impacts the ground, replacing old one
                prog.AddConstraint(-state_trajectory[j-1,0] == state_trajectory[j,0])
                prog.AddConstraint(state_trajectory[j-1,1]*cos(2*impact_angle) == state_trajectory[j,1])
                
        # Initial guess
        # TODO: Warmstart solver
        prog.SetInitialGuess(t_f[0], 20.0)
        prog.AddBoundingBoxConstraint(2.0, 50.0, t_f[0])

        for i in range(len(state_trajectory[:,0])):
            state_guess = ((N - i) / N) * x0 + (i / N) * xf
            prog.SetInitialGuess(state_trajectory[i,0], state_guess[0])
            prog.SetInitialGuess(state_trajectory[i,1], state_guess[1])
        for i in range(N):
            prog.SetInitialGuess(input_trajectory[i,0], 0.1)
        
        # Iteration limit    
        it_limit = int(max(20000, 40*prog.num_vars()))
        prog.SetSolverOption(SolverType.kSnopt, 'Iterations limit', it_limit)
        
        # Get results
        result = Solve(prog)
        t_f = result.GetSolution(t_f)
        input_trajectory = result.GetSolution(input_trajectory)
        state_trajectory_approx = result.GetSolution(state_trajectory)
        time_array = t_f * np.linspace(0.0, 1.0, N+1)
        
        tsolve_post = time.time()
        tsolve = tsolve_post - tsolve_pre

        if not result.is_success():
            infeasible = result.GetInfeasibleConstraints(prog)
            print("Infeasible constraints:")
            for i in range(len(infeasible)):
                print(infeasible[i])
        
        else:
            print("Solution found!")
            print("Solver finished in %.1f seconds" % tsolve)
            print ("t_f computed: %.3f seconds" % t_f[0])
            print ("Cost: %.3f" % cost(input_trajectory))
            
        plt.figure()
        plt.plot(time_array, state_trajectory_approx[:,0], label='$\\theta$')
        plt.plot(time_array, state_trajectory_approx[:,1], label='$\\dot{\\theta}$')
        plt.legend()
        plt.figure()
        plt.plot(time_array[:-1], input_trajectory, label='$u$')
        plt.legend()
        plt.show()
        
        return input_trajectory, state_trajectory_approx, time_array
        
def main():
    system = LIPMSystem()
    x0 = np.array([0, 1.])
    xf = np.array([np.pi, 0.])
    #tf = 5.0
    N = 20
    
    contact_sequence = {}
    system.lipm_walk_trajopt(x0, xf, N, contact_sequence)
    
if __name__ == '__main__':
    main()
    