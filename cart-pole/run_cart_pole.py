from cart_pole_control import *

import gym
import control
import numpy as np

gym.envs.registration.register(
    id='gym_cart_pole:CartPoleSwingUpContinuous-v0',
    entry_point='gym_cart_pole.envs:CartPoleSwingUpEnv',
    max_episode_steps=300,
)

# Constants to define the system
GRAVITY = 9.81
MASS_CART = 1
MASS_POLE = 1
LENGTH_POLE = 0.5


def compute_optimal_gains():
    gravity_per_length = GRAVITY / LENGTH_POLE
    total_mass = MASS_CART + MASS_CART

    x_3 = gravity_per_length / (4/3 - MASS_POLE / total_mass)

    # Plant dynamics
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, x_3, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, x_3, 0.0]
    ])

    # influence of control inputs
    B = np.array([
        [0.0],
        [1 / total_mass],
        [0.0],
        [x_3 / -GRAVITY]
    ])

    # cost of state
    Q = 5 * np.eye(4)
    # cost of inputs
    R = np.eye(1)

    # optimal gains based on given cost matrices Q and R
    k_lqr, _, _ = control.lqr(A, B, Q, R)
    k_lqr = np.array(k_lqr)

    return k_lqr

def run_loop(env, state, controller):
    """
    Simulation loop
    Choose action based on state,
    Completes action
    """

    done = False
    while not done:

        state = controller.state_estimate_callback(state)
        action = controller.get_action()
                
        state, _, done, _ = env.step(action) 
        env.render()

def main():
    env = gym.make('gym_cart_pole:CartPoleSwingUpContinuous-v0')
    state = env.reset()

    swingup_mode = 1
    k_lqr = compute_optimal_gains()

    if (swingup_mode == 0):
        controller = CartPoleEnergyShapingController(gravity=GRAVITY, mass_cart=MASS_CART, mass_pole=MASS_POLE, length_pole=LENGTH_POLE, k_lqr=k_lqr)
    else:
        controller = CartPoleMPCController(gravity=GRAVITY, mass_cart=MASS_CART, mass_pole=MASS_POLE, length_pole=LENGTH_POLE, k_lqr=k_lqr)

    # number of episodes
    for i in range(100):
        try:
            run_loop(env, state=state, controller=controller)
            print('Finishing episode: %d' % (i + 1))
        except RuntimeError as exc:
            print(exc)
        state = env.reset()
    env.close()

if __name__ == '__main__':
    main()