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

## [ODrive](https://github.com/packbionics/control-examples/tree/main/odrive)
### Dependencies
[ODriveTool](https://docs.odriverobotics.com/v/latest/getting-started.html#downloading-and-installing-odriveool)
### Description
Scripts for ODrive motor calibration, plus position/velocity/torque control

## Walker
### Dependencies
[Drake](https://drake.mit.edu/installation.html)

### Description
Explore trajectory optimization for generating limit cycles in reduced-order models, with applications towards gait generation in prosthetic legs.

### [LIPM](https://github.com/packbionics/control-examples/tree/main/walker/lipm)
Walking reduced to a linear inverted pendulum. Direct transciption with hybrid dynamics.
