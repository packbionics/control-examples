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
        swing it to the top
    Source:
        This environment corresponds to the cart-pole swingup control problem
    Observation:
        Type: Box(4)
        Num     Observation               Min                     Max
        0       Cart Position             -4.8                    4.8
        1       Cart Velocity             -Inf                    Inf
        2       Pole Angle                -Inf                    Inf
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
    def __init__(self, gravity=9.81, masscart=1.0, 
                    masspole=1.0, length=0.5, 
                    force_mag=10.0, tau=0.02, 
                    kinematics_integrator='euler'):
        super().__init__()

        assert masscart > 0.
        assert masspole > 0.
        assert length > 0.
        assert force_mag > 0.
        assert tau > 0.
        assert kinematics_integrator in ['euler', 'semi-implicit-euler']

        self.gravity = gravity
        self.masscart = masscart
        self.masspole = masspole
        self.total_mass = (self.masspole + self.masscart)
        self.length = length # actually half the pole's length
        self.polemass_length = (self.masspole * self.length)
        self.force_mag = force_mag
        self.tau = tau  # seconds between state updates
        self.kinematics_integrator = kinematics_integrator

        self.theta_threshold_radians = np.finfo(np.float32).max
        high = np.array([self.x_threshold * 2,
                 np.finfo(np.float32).max,
                 self.theta_threshold_radians,
                 np.finfo(np.float32).max],
                dtype=np.float32)
        self.action_space = spaces.Box(-np.finfo(np.float32).max, np.finfo(np.float32).max, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

    def reset(self):
        super().reset()
        possible_init_angles = [self.np_random.uniform(low=np.pi-0.05, high=np.pi, size=(1,)),
                                self.np_random.uniform(low=-np.pi, high=-np.pi+0.05, size=(1,))]
        self.state[2] = possible_init_angles[np.random.choice(2)]
        return np.array(self.state)

    # get_force maps action to forces constrained by the force limit 
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

        done = False

        if not done:
            if abs(theta - 0.0) < 0.01 and abs(theta_dot - 0.0) < 0.01:
                reward = 0.0
            elif bool(x < -self.x_threshold or x > self.x_threshold):
                reward = -10.
                done = True
                if self.steps_beyond_done is None:
                    self.steps_beyond_done = 0
                elif self.steps_beyond_done == 0:
                    logger.warn(
                        "You are calling 'step()' even though this "
                        "environment has already returned done = True. You "
                        "should always call 'reset()' once you receive 'done = "
                        "True' -- any further steps are undefined behavior."
                    )
                    self.steps_beyond_done += 1
                    reward = 0.0
            else:
                reward = -1.0
        elif self.steps_beyond_done is None:
            # Cart just went out of bounds
            self.steps_beyond_done = 0
            reward = -1.0

        return self.state, reward, done, {}

class CartPoleSwingUpDiscreteEnv(CartPoleSwingUpEnv):
    def __init__(self):
        super().__init__()
        self.action_space = spaces.Discrete(2)

    def get_force(self, action):
        return self.force_mag if action == 1 else -self.force_mag