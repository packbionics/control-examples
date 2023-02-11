from cart_pole_control import *

import gym
import control
import numpy as np

gym.envs.registration.register(
    id='gym_cart_pole:CartPoleSwingUpContinuous-v0',
    entry_point='gym_cart_pole.envs:CartPoleSwingUpEnv',
    max_episode_steps=1000,
)

env = gym.make('gym_cart_pole:CartPoleSwingUpContinuous-v0')
env.reset() 
action = None
state = None
t = 0

swingup_mode = 0
swingup_controller = None

gravity = 9.81
mass_cart = 1
mass_pole = 1
length_pole = 0.5

gravity_per_length = gravity / length_pole
total_mass = mass_cart + mass_cart

x_3 = gravity_per_length / (4/3 - mass_pole / total_mass)

A = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, x_3, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, x_3, 0.0]
])

B = np.array([
    [0.0],
    [1 / total_mass],
    [0.0],
    [x_3 / -gravity]
])

Q = 5 * np.eye(4)

R = np.eye(1)

k_lqr, _, _ = control.lqr(A, B, Q, R)
print(k_lqr[0])
k_lqr = np.array(k_lqr)

if (swingup_mode == 0):
    controller = CartPoleEnergyShapingController(gravity=gravity, mass_cart=mass_cart, mass_pole=mass_pole, length_pole=length_pole, k_lqr=k_lqr)
else:
    controller = CartPoleMPCController()

# LQR gains
K = np.array([-3.16,54.04,-4.82,9.74])

"""
Simulation loop
Choose action based on state,
Completes action and returns new state
"""
for _ in range(100000):
    if state is None:
        action = env.action_space.sample()
    else:
        state = controller.state_estimate_callback(state)

        if (abs(controller.theta_distance(state[2],0)) < .7):
            action = controller.upright_lqr()
        else:
            try:
                action = controller.swingup()
            except ValueError:
                break
            
    for _ in range(1):
        env.render()
        state, reward, done, _ = env.step(action) 
        if done:
            env.reset()
            t = 0
            action = np.zeros((1,))
            break
        t += 1
env.close()