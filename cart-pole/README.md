# cart-pole

Exploration of controlling cart-pole system. Uses "energy shaping" swingup controller, 
LQR for balancing.

Contains modified cart-pole gym environment for testing controller in simulation.
Arduino code used to control real hardware.

## Installation
```
$ pip install -e .
```
## Run
```run_cart_pole.py``` starts up a cart-pole swingup simulation with energy shaping controller + LQR

```manual_cart_pole.py``` utilizes a brain-powered controller
