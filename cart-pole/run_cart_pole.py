from cart_pole_control import *

import gym
import numpy as np
import matplotlib.pyplot as plt

env = gym.make('gym_cart_pole:CartPoleSwingUpContinuous-v0')
env.reset() 
action = None
state = None
t = 0

swingup_mode = 1
swingup_controller = None

if (swingup_mode == 0):
    controller = CartPoleEnergyShapingController()
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

        if (abs(controller.theta_distance(state[2],math.pi)) < .6):
            action = controller.upright_lqr()
        else:
            action = controller.swingup()

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