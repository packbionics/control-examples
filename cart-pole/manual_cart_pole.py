from cart_pole_control import *

import gym
import numpy as np
import pygame
import sys

env = gym.make('gym_cart_pole:CartPoleSwingUpContinuous-v0')
env.reset() 
action = None
state = None
current_force = 0
pygame.init()
screen = pygame.display.set_mode((640, 480), 0, 32)
pygame.event.set_grab(True)

# LQR gains
K = np.array([-4.47,70,-6,10.45])

def get_key_press():
    pygame.event.pump()
    force_out = 0.
    key_input = pygame.key.get_pressed()

    if key_input[pygame.K_LEFT]:
        force_out = -15
    if key_input[pygame.K_RIGHT]:
        force_out = 15
    if key_input[pygame.K_SPACE]:
        force_out = .999
    if key_input[pygame.K_RETURN]:
        sys.exit()

    return np.array([force_out])

"""
Simulation loop
Choose action based on state,
Completes action and returns new state
"""
controller = EnergyShapingController()

for _ in range(1000000):
    if state is None:
        action = env.action_space.sample()
    else:
        state = controller.state_estimate_callback(state)

        if (abs(controller.theta_distance(state[2],np.pi)) < 0.4):
            if get_key_press() != .999:
                action = controller.upright_lqr()
            else:
                env.reset()
                action = np.zeros((1,))
                continue
        else:
            action = get_key_press()
    for _ in range(1):
        env.render()
        state, reward, done, _ = env.step(action) 
        if done:
            env.reset()
            action = np.zeros((1,))
            break
env.close()