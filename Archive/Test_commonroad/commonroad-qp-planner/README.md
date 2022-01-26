## CommonRoad Quadratic Planning-Planner

### System Requirements
The software is written in Python 3.7 and has been tested on Ubuntu 20.04. The code is adapted from [CommonRoad Reachability](https://gitlab.lrz.de/cps/commonroad-reachability) by removing reachability analysis dependancies. Now the QP planner is capable with reachable set and other convex constraints.


### The required Python dependencies
* matplotlib>=3.3.4
* numpy>=1.19.5
* cvxpy>=1.1.15
* ecos>=2.0.7
* commonroad-io>=2021.3
* commonroad-vehicle-models>=2.0.0

Furthermore, the [CommonRoad Drivability Checker>=2021.1](https://commonroad.in.tum.de/drivability-checker) and [CommonRoad Route Planner>=1.0.0](https://gitlab.lrz.de/tum-cps/commonroad-route-planner) libraries are required.


## Minimal Example
An example script can be found under the `test/` folder.
