# control-examples
Examples of control relevant to PackBionics projects

## [Cart-Pole](https://github.ncsu.edu/jcxie/control-examples/tree/main/cart-pole)

### Dependencies
[OpenAI gym](https://gym.openai.com/)
```
$ pip install gym
```
[Pygame](https://www.pygame.org/)
```
$ python3 -m pip install -U pygame --user
```
### Description
An unactuated pendulum is attached to a cart sliding on a rail, swing the pendulum to the top by applying forces to the cart

Three ways to achieve the task is available:
1. Manual control: You move the cart to swing the pole up
1. Energy shaping: Use an "energy function" to guarantee convergence to fixed point
1. Trajectory Optimization: Run optimization to find best trajectory wrt cost

## ODrive
### Dependencies
[ODriveTool](https://docs.odriverobotics.com/v/latest/getting-started.html#downloading-and-installing-odriveool)
A script for ODrive motor calibration, plus position/velocity/torque control

## Walker
### Dependencies
[Drake](https://drake.mit.edu/installation.html)

### Description
Explore trajectory optimization for generating limit cycles in reduced-order models, with applications towards gait generation in prosthetic legs.

### LIPM
Walking reduced to a linear inverted pendulum. Direct collocation with hybrid dynamics.
