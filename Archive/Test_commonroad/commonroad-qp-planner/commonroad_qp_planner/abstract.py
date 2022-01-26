from abc import ABC, abstractmethod
import numpy as np

"""
Class for trajectory planner.
"""


def is_valid_duration(duration: float, N=None, dT=None):
    """
    Checks if a specified duration is greater than zero and if a given horizon N is applicable to the duration
    :param duration: The duration
    :param N: The length of the optimization horizon
    :return: True if the specified duration is valid and also if the optimization horizon is valid
    """
    if N is None:
        return duration > 0.0
    else:
        factor = duration / N
        if dT is None:
            return duration > 0.0 and isinstance(N, int) and N > 0
        else:
            return duration > 0.0 and isinstance(N, int) and N > 0 and np.isclose(factor, dT)


class TrajectoryPlanner(ABC):
    """
        Abstract base class for a trajectory planner. Contains basic methods and properties every planner has to offer,
        e.g., time step and horizon
    """

    def __init__(self, horizon: float, N: int, dT: float):
        assert is_valid_duration(horizon,
                                 N,
                                 dT), "Duration or time horizon not valid, e.g. not greater than zero!"
        self._horizon = horizon
        self._N = N
        self._dT = dT

    @property
    def horizon(self) -> float:
        return self._horizon

    @horizon.setter
    def horizon(self, horizon: float):
        raise Exception("You are not allowed to change the horizon of the planner!")

    @property
    def N(self) -> int:
        return self._N

    @N.setter
    def N(self, N: int):
        raise Exception("You are not allowed to change the horizon of the planner!")

    @property
    def dT(self) -> float:
        return self._dT

    @dT.setter
    def dT(self, dT: float):
        raise Exception("You are not allowed to change the time step of the planner!")

    @abstractmethod
    def plan(self, *args, **kwargs):
        """
        Plans the trajectory
        :return: The trajectory of the maneuver with respect to the initial state and environment map
        """
        pass

    @abstractmethod
    def tv_constraints(self, *args, **kwargs):
        """
        Add time-variant constraints
        """
        pass

    @abstractmethod
    def ti_constraints(self, *args, **kwargs):
        """
        Add time-invariant constraints
        """
        pass

    @abstractmethod
    def cost_function(self, *args, **kwargs):
        """
        Define cost functions
        """
        pass