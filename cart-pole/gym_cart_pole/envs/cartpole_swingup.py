import gym
import math
import numpy as np
from gym import spaces, logger
from gym.envs.classic_control.cartpole import CartPoleEnv

class CartPoleSwingUpEnv(CartPoleEnv):
    """
    Description:
        A pole is attached by an un-actuated joint to a cart, which moves along
        a frictionless track. The pendulum starts on the bottom, and the goal is to
        prevent it from falling over by increasing and reducing the cart's
        velocity.
    Source:
        This environment corresponds to the version of the cart-pole problem
        described by Barto, Sutton, and Anderson
    Observation:
        Type: Box(4)
        Num     Observation               Min                     Max
        0       Cart Position             -4.8                    4.8
        1       Cart Velocity             -Inf                    Inf
        2       Pole Angle                -Inf                       Inf
        3       Pole Angular Velocity     -Inf                    Inf
    Actions:
        Type: Box(1)
        Num   Action                      Min                     Max
        0     Move cart. Neg. is left     -force_mag              force_mag
    Reward:
        Reward is -1 until pole is vertical
    Starting State:
        All observations are assigned a uniform random value
    Episode Termination:
        Cart Position is more than 2.4 (center of the cart reaches the edge of
        the display).
        Episode length is greater than 200.
    """
    def __init__(self):
        super().__init__()
        self.theta_threshold_radians = np.finfo(np.float32).max
        high = np.array([self.x_threshold * 2,
                 np.finfo(np.float32).max,
                 self.theta_threshold_radians,
                 np.finfo(np.float32).max],
                dtype=np.float32)
        self.action_space = spaces.Box(-np.finfo(np.float32).max, np.finfo(np.float32).max, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

    def get_force(self, action):
        return min(action, self.force_mag) if action > 0.0 else max(action, -self.force_mag)

    def step(self, action):
        err_msg = "%r (%s) invalid" % (action, type(action))
        assert self.action_space.contains(action), err_msg

        x, x_dot, theta, theta_dot = self.state
        force = self.get_force(action)
        costheta = math.cos(theta)
        sintheta = math.sin(theta)

        # For the interested reader:
        # https://coneural.org/florian/papers/05_cart_pole.pdf
        temp = (force + self.polemass_length * theta_dot ** 2 * sintheta) / self.total_mass
        thetaacc = (self.gravity * sintheta - costheta * temp) / (self.length * (4.0 / 3.0 - self.masspole * costheta ** 2 / self.total_mass))
        xacc = temp - self.polemass_length * thetaacc * costheta / self.total_mass

        if self.kinematics_integrator == 'euler':
            x = x + self.tau * x_dot
            x_dot = x_dot + self.tau * xacc
            theta = theta + self.tau * theta_dot
            theta_dot = theta_dot + self.tau * thetaacc
        else:  # semi-implicit euler
            x_dot = x_dot + self.tau * xacc
            x = x + self.tau * x_dot
            theta_dot = theta_dot + self.tau * thetaacc
            theta = theta + self.tau * theta_dot

        self.state = (x, x_dot, theta, theta_dot)

        done = bool(
            x < -self.x_threshold
            or x > self.x_threshold
        )

        if not done:
            if abs(theta - 0.0) < 0.01:
                reward = 0.0
            else:
                reward = -1.0
        elif self.steps_beyond_done is None:
            # Cart just went out of bounds
            self.steps_beyond_done = 0
            reward = -1.0
        else:
            if self.steps_beyond_done == 0:
                logger.warn(
                    "You are calling 'step()' even though this "
                    "environment has already returned done = True. You "
                    "should always call 'reset()' once you receive 'done = "
                    "True' -- any further steps are undefined behavior."
                )
            self.steps_beyond_done += 1
            reward = 0.0

        return np.array(self.state), reward, done, {}

class CartPoleSwingUpDiscreteEnv(CartPoleSwingUpEnv):
    def __init__(self):
        super().__init__()
        self.action_space = spaces.Discrete(2)

    def get_force(self, action):
        return self.force_mag if action == 1 else -self.force_mag