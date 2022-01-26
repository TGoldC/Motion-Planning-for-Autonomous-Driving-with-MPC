import numpy as np

import commonroad.common.validity as val


class TIConstraints:
    a_x_min = -6.0
    a_x_max = 6.0
    a_y_min = -8.0
    a_y_max = 8.0
    v_min = 0.0
    v_max = 48.0
    j_x_min = -15.0
    j_x_max = 15.0
    j_y_min = -10.0
    j_y_max = 10.0
    k_max = 0.2
    a_max = 10.5
    kappa_dot_min = -0.2  # minimum steering rate
    kappa_dot_max = 0.2  # maximum steering rate
    kappa_dot_dot_min = -0.2  # minimum steering rate rate
    kappa_dot_dot_max = 0.2  # maximum steering rate rate
    kappa_max = 0.20  # maximum curvature


class LonConstraints:
    def __init__(self, N: int):
        assert N > 0
        self.s_soft_min = np.repeat(-np.inf, N)
        self.s_soft_max = np.repeat(np.inf, N)
        self.s_hard_min = np.repeat(-np.inf, N)
        self.s_hard_max = np.repeat(np.inf, N)

    @classmethod
    def construct_constraints(cls,
                              s_soft_min: np.ndarray,
                              s_soft_max: np.ndarray,
                              s_hard_min: np.ndarray,
                              s_hard_max: np.ndarray,
                              v_max=None,
                              v_min=None):
        # check if constraints are valid vectors
        assert val.is_real_number_vector(s_soft_min) and val.is_real_number_vector(
            s_soft_max) and val.is_real_number_vector(
            s_hard_min) and val.is_real_number_vector(s_hard_max)
        # check if all constraint has the same length
        assert len(s_soft_min) == len(s_soft_max) and len(s_soft_max) == len(s_hard_min) and len(s_hard_min) == len(
            s_hard_max)

        constraints = LonConstraints(len(s_hard_max))
        constraints.s_soft_min = s_soft_min
        constraints.s_soft_max = s_soft_max
        constraints.s_hard_min = s_hard_min
        constraints.s_hard_max = s_hard_max

        # TODO: fix velocity constraints
        # v_max, v_min: [As, Av, b]: As * s + Av * v <= b
        no_arg = False
        if v_max is None or v_min is None:
            v_max = np.repeat(np.inf, len(s_soft_min))
            v_min = np.repeat(-np.inf, len(s_soft_min))
            no_arg = True
        assert len(v_max) == len(v_min) and len(v_max) == len(s_soft_min)

        constraints.v_max = v_max
        constraints.v_min = v_min
        constraints._valid_velocity = not no_arg

        return constraints

    @property
    def valid_velocity(self) -> bool:
        return self._valid_velocity

    @property
    def v_max(self) -> list:
        return self._v_max

    @v_max.setter
    def v_max(self, v_max: list):
        assert val.is_valid_velocity(v_max)
        self._v_max = v_max

    @property
    def v_min(self) -> list:
        return self._v_min

    @v_min.setter
    def v_min(self, v_min: list):
        assert val.is_valid_velocity(v_min)
        self._v_min = v_min

    @property
    def s_soft_min(self) -> np.ndarray:
        return self._s_soft_min

    @s_soft_min.setter
    def s_soft_min(self, s_soft_min):
        assert val.is_real_number_vector(s_soft_min)
        self._s_soft_min = s_soft_min

    @property
    def s_soft_max(self) -> np.ndarray:
        return self._s_soft_max

    @s_soft_max.setter
    def s_soft_max(self, s_soft_max):
        assert val.is_real_number_vector(s_soft_max)
        self._s_soft_max = s_soft_max

    @property
    def s_hard_min(self) -> np.ndarray:
        return self._s_hard_min

    @s_hard_min.setter
    def s_hard_min(self, s_hard_min):
        assert val.is_real_number_vector(s_hard_min)
        self._s_hard_min = s_hard_min

    @property
    def s_hard_max(self) -> np.ndarray:
        return self._s_hard_max

    @s_hard_max.setter
    def s_hard_max(self, s_hard_max):
        assert val.is_real_number_vector(s_hard_max)
        self._s_hard_max = s_hard_max

    @property
    def N(self):
        return len(self.s_soft_min)

    @N.setter
    def N(self, N):
        pass


class LatConstraints:
    def __init__(self, N: int):
        assert N > 0
        self.d_soft_min = np.repeat(-np.inf, N)
        self.d_soft_max = np.repeat(np.inf, N)
        self.d_hard_min = np.repeat(-np.inf, N)
        self.d_hard_max = np.repeat(np.inf, N)

    @classmethod
    def construct_constraints(cls,
                              d_soft_min: np.ndarray,
                              d_soft_max: np.ndarray,
                              d_hard_min: np.ndarray,
                              d_hard_max: np.ndarray):
        # check if all constraint has the same length
        assert len(d_soft_min) == len(d_soft_max) and len(d_soft_max) == len(d_hard_min) and len(d_hard_min) == len(
            d_hard_max)

        lat_constraints = LatConstraints(len(d_soft_max))
        lat_constraints.d_soft_min = d_soft_min
        lat_constraints.d_soft_max = d_soft_max
        lat_constraints.d_hard_min = d_hard_min
        lat_constraints.d_hard_max = d_hard_max

        return lat_constraints

    @property
    def d_soft_min(self) -> list:
        return self._d_soft_min

    @d_soft_min.setter
    def d_soft_min(self, d_soft_min):
        assert (val.is_real_number_vector(d) for d in d_soft_min)
        self._d_soft_min = d_soft_min

    @property
    def d_soft_max(self) -> list:
        return self._d_soft_max

    @d_soft_max.setter
    def d_soft_max(self, d_soft_max):
        assert (val.is_real_number_vector(d) for d in d_soft_max)
        self._d_soft_max = d_soft_max

    @property
    def d_hard_min(self) -> list:
        return self._d_hard_min

    @d_hard_min.setter
    def d_hard_min(self, d_hard_min):
        assert (val.is_real_number_vector(d) for d in d_hard_min)
        self._d_hard_min = d_hard_min

    @property
    def d_hard_max(self):
        return self._d_hard_max

    @d_hard_max.setter
    def d_hard_max(self, d_hard_max):
        assert (val.is_real_number_vector(d) for d in d_hard_max)
        self._d_hard_max = d_hard_max

    @property
    def N(self):
        return len(self.d_hard_min)

    @N.setter
    def N(self, N):
        pass


class TVConstraints:
    """
    Time variant constraints
    """
    @classmethod
    def create_from_params(cls, horizon: float, N: int, dT: float):
        assert N == int(round(horizon/dT))
        lon, lat = cls.set_default_constraints(N)
        return cls(lon, lat)

    @staticmethod
    def set_default_constraints(N):
        lon = LonConstraints(N)
        lat = LatConstraints(N)

        return lon, lat

    def __init__(self,
                 c_long: LonConstraints,
                 c_lat: LatConstraints):
        self._N = c_long.N
        assert c_long.N == c_lat.N
        self._lon = c_long
        self._lat = c_lat

    @property
    def lon(self) -> LonConstraints:
        return self._lon

    @lon.setter
    def lon(self, c: LonConstraints):
        assert isinstance(c, LonConstraints) and c.N == self.N
        self._lon = c

    @property
    def lat(self) -> LatConstraints:
        return self._lat

    @lat.setter
    def lat(self, c: LatConstraints):
        assert isinstance(c, LatConstraints) and c.N == self.N
        self._lat = c

    @property
    def N(self) -> int:
        return self._N

    @N.setter
    def N(self, N: int):
        raise Exception("You are not allowed to change the horizon of the constraints!")


