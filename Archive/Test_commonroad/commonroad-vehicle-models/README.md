# Vehicle Models of CommonRoad

This repository contains all vehicle models of the [CommonRoad benchmarks](https://commonroad.in.tum.de/).

We provide implementation examples in MATLAB and Python, routines to convert initial states and parameters, simulation examples to demonstrate the advantages of more complicated models, and a detailed documentation.

## Installation (Python)

To use vehicle models and parameters, add the folder `Python` to your PYTHONPATH. If you are using PyCharm, this can be set through File->Settings->Project Interpreter->Show all->Edit


## Contribute

If you want to contribute new vehicle models, you can create a merge request or contact us via our [forum](https://commonroad.in.tum.de/forum/).


## Changelog
Compared to version 2019b the following features were added:
* kinematic single-track model with on-axle trailer
* vehicle parameter set for a semi-trailer truck (vehicle ID: 4)
* single-track drift model: nonlinear single-track model with Pacejka tire forces
* refactoring and restructuring of MATLAB and PYTHON packages

## References

If you use CommonRoad, please cite *[M. Althoff, M. Koschi, and S. Manzinger, ''CommonRoad: Composable Benchmarks for Motion Planning on Roads,'' in Proc. of the IEEE Intelligent Vehicles Symposium, 2017, pp. 719-726](http://mediatum.ub.tum.de/doc/1379638/776321.pdf)*.
