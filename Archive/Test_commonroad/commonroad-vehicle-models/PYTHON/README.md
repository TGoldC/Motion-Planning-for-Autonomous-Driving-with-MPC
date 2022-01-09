# Python Vehicle Models of CommonRoad

This package contains all vehicle models of the [CommonRoad benchmarks](https://commonroad.in.tum.de/).

We provide implementations of the vehicle dynamics, routines to convert initial states, and vehicle parameters.

## Documentation

For a detailed explanation of the vehicle models, please have a look at the [documentation](https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf).

## Installation

To use vehicle models and parameters, run
```
pip install commonroad-vehicle-models
```

## Code examples

For an extended simulation example demonstrating the advantages of more complicated models, we refer to our [gitlab repository](https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/tree/master/PYTHON/scripts). A simple simulation example for using the single-track model in combination with an odeint solver would be

```python3
from scipy.integrate import odeint
import numpy

from vehiclemodels.init_ks import init_ks
from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks

def func_KS(x, t, u, p):
    f = vehicle_dynamics_ks(x, u, p)
    return f

tStart = 0  # start time
tFinal = 1  # start time

# load vehicle parameters
p = parameters_vehicle1()

# initial state for simulation
delta0 = 0
vel0 = 15
Psi0 = 0
sy0 = 0
initialState = [0, sy0, delta0, vel0, Psi0]
x0_KS = init_ks(initialState)

t = numpy.arange(0, tFinal, 0.01)
u = [0, 5]
x = odeint(func_KS, x0_KS, t, args=(u, p))

```



## Contribute

If you want to contribute new vehicle models, you can create a merge request in our [repository](https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/), or contact via our [forum](https://commonroad.in.tum.de/forum/).


## Changelog
Compared to version 2019b the following features were added:
* kinematic single-track model with on-axle trailer
* vehicle parameter set for a semi-trailer truck (vehicle ID: 4)
* single-track drift model: nonlinear single-track model with Pacejka tire forces


## Referencing

If you use CommonRoad, please cite

*[M. Althoff, M. Koschi, and S. Manzinger, ''CommonRoad: Composable Benchmarks for Motion Planning on Roads,'' in Proc. of the IEEE Intelligent Vehicles Symposium, 2017, pp. 719-726](http://mediatum.ub.tum.de/doc/1379638/776321.pdf)*.
