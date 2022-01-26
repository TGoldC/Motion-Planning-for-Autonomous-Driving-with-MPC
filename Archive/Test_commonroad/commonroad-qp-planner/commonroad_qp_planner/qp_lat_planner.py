import numpy as npy

from cvxpy import *

from commonroad_qp_planner.abstract import TrajectoryPlanner
from commonroad_qp_planner.constraints import TIConstraints, LatConstraints
from commonroad_qp_planner.trajectory import Trajectory, TrajPoint, TrajectoryType

from commonroad_dc.geometry.util import compute_curvature_from_polyline, compute_pathlength_from_polyline, \
    compute_orientation_from_polyline

import commonroad.common.validity as val


# QPLatPlanner Weights
class QPLatPARAMS:
    W_D = 0.0  # weight for d deviation
    W_THETA = 0.1  # weight for theta deviation
    W_KAPPA = 40  # weight for kappa deviation
    W_D_N = 0.15  # weight for d deviation at end state
    W_THETA_N = 0.1  # weight for theta deviation at end state
    W_KAPPA_N = 40  # weight for kappa deviation at end state
    W_KAPPA_DOT = 20  # weight for kappa dot deviation
    W_U = 1  # weight for input deviation
    W_SLACK_L = 5.0  # linear weight for slack variables
    W_SLACK_Q = 2.0  # quadratic weight for slack variables


class QPLatState(object):
    """
    Class representing a state <d,theta,kappa,kappa_dot> within the QPLatPlanner
    """
    def __init__(self, d: float, theta: float, kappa: float, kappa_dot: float,
                 t=0., s=None, v=None, a=None, j=None, u_lon=None):
        self.d = d
        self.theta = theta
        self.kappa = kappa
        self.kappa_dot = kappa_dot
        self.t = t

        self.s = s
        self.v = v
        self.a = a
        self.j = j
        self.u_lon = u_lon

    @property
    def u_lon(self) -> list:
        return self._u_lon

    @u_lon.setter
    def u_lon(self, u_lon: list):
        self._u_lon = u_lon

    @property
    def s(self) -> float:
        return self._s

    @s.setter
    def s(self, s: float):
        self._s = s

    @property
    def v(self) -> float:
        return self._v

    @v.setter
    def v(self, v: float):
        self._v = v

    @property
    def a(self) -> float:
        return self._a

    @a.setter
    def a(self, a: float):
        self._a = a

    @property
    def j(self) -> float:
        return self._j

    @j.setter
    def j(self, j: float):
        self._j = j

    @property
    def d(self) -> float:
        return self._d

    @d.setter
    def d(self, d: float):
        assert d is not None, "<QPLatState> d is not valid! d = {}".format(d)
        self._d = d

    @property
    def theta(self) -> float:
        return self._theta

    @theta.setter
    def theta(self, theta: float):
        assert val.is_valid_orientation(theta), "<QPLatState> theta is not valid! theta = {}".format(theta)
        self._theta = theta

    @property
    def kappa(self) -> float:
        return self._kappa

    @kappa.setter
    def kappa(self, kappa: float):
        assert kappa is not None, "<QPLatState> kappa is not valid! kappa = {}".format(kappa)
        self._kappa = kappa

    @property
    def kappa_dot(self) -> float:
        return self._kappa_dot

    @kappa_dot.setter
    def kappa_dot(self, kappa_dot):
        assert val.is_valid_orientation(kappa_dot), "<QPLatState> kappa_dot is not valid! " \
                                                    "kappa_dot = {}".format(kappa_dot)
        self._kappa_dot = kappa_dot

    @property
    def t(self):
        return self._t

    @t.setter
    def t(self, t: float):
        assert isinstance(t, val.ValidTypes.NUMBERS) and t >= 0., '<QPLatState> provided t is ' \
                                                                  'not valid. t = {}'.format(t)
        self._t = t

    def to_array(self) -> list:
        return npy.array([self.d, self.theta, self.kappa, self.kappa_dot])

    def __str__(self):
        return '<QPLatState> (d={}, theta={}, kappa{}, kappa_dot={}, t={})'.format(self.d, self.theta, self.kappa,
                                                                                   self.kappa_dot, self.t)


class QPLatRefState(object):
    """
    Class representing a state <s,v,theta,kappa> of QPLatPlanner reference (longitudinal profile and
    curvature/orientation of reference)
    """

    def __init__(self, s: float, v: float, a: float, j: float, theta: float, kappa: float):
        self.s = s
        self.v = v
        self.a = a
        self.j = j
        self.theta = theta
        self.kappa = kappa

    @property
    def s(self) -> float:
        return self._s

    @s.setter
    def s(self, s: float):
        assert s is not None, "<QPLatRefState> s is not valid! s = {}".format(s)
        self._s = s

    @property
    def v(self) -> float:
        return self._v

    @v.setter
    def v(self, v: float):
        assert val.is_valid_velocity(v), "<QPLatRefState> v is not valid! v = {}".format(v)
        self._v = v

    @property
    def a(self) -> float:
        return self._a

    @a.setter
    def a(self, a: float):
        assert isinstance(a, val.ValidTypes.NUMBERS), "<QPLatRefState> a is not valid! a = {}".format(a)
        self._a = a

    @property
    def j(self) -> float:
        return self._j

    @j.setter
    def j(self, j: float):
        assert val.is_real_number(j), '<QPLatRefState>: j is not valid! j = {}'.format(j)
        self._j = j

    @property
    def theta(self) -> float:
        return self._theta

    @theta.setter
    def theta(self, theta: float):
        assert val.is_valid_orientation(theta), "<QPLatRefState> theta is not valid! theta = {}".format(theta)
        self._theta = theta

    @property
    def kappa(self) -> float:
        return self._kappa

    @kappa.setter
    def kappa(self, kappa):
        assert kappa is not None and isinstance(kappa, val.ValidTypes.NUMBERS), "<QPLatRefState> kappa is not valid! " \
                                                                                "kappa = {}".format(kappa)
        self._kappa = kappa

    def to_array(self) -> list:
        return npy.array([self.s, self.v, self.a, self.j, self.theta, self.kappa])

    def __str__(self):
        return '<QPLatRefState> (s={}, v={}, a={}, theta={}, kappa={})'.format(self.s, self.v, self.a, self.theta,
                                                                               self.kappa)


class QPLatReference(object):
    """
    Class representing a QPLatReference made up of a list of QPLatRefStates
    """

    def __init__(self, reference: list()):
        self.reference = reference

    @property
    def reference(self):
        return self._reference

    @reference.setter
    def reference(self, reference: list()):
        # check if reference is list of reference states
        assert isinstance(reference, list) and (isinstance(s, QPLatRefState) for s in reference)
        self._reference = reference

    def length(self) -> int:
        if isinstance(self.reference, list):
            return len(self.reference)
        else:
            return 1

    @classmethod
    def construct_from_lon_traj_and_reference(cls,
                                              lon_traj: Trajectory,
                                              reference: npy.ndarray,
                                              ti=None) -> 'QPLatReference':
        assert isinstance(lon_traj, Trajectory) and lon_traj.coord_type is TrajectoryType.CARTESIAN, \
            '<QPLatReference>: Provided longitudinal trajectory is invalid or not in Frenet. traj = {}'.format(lon_traj)
        assert npy.isclose(npy.sum(lon_traj.get_positions()[:, 1]), 0.), \
            '<QPLatReference>: Provided longitudinal trajectory containts lateral information != 0. d = {}'.format(
                lon_traj.get_positions()[:, 1])
        assert isinstance(reference, npy.ndarray) and reference.ndim == 2 and len(reference) > 1 and len(
            reference[0, :]) == 2, '<QPLatReference>: Provided reference is not valid. reference = {}'.format(reference)
        if ti is not None:
            assert isinstance(ti, TIConstraints), '<QPLatReference>: provided ti constraints are not valid!'

        # compute orientation, curvature and pathlength of reference
        ref_orientation = compute_orientation_from_polyline(reference)
        ref_curvature = compute_curvature_from_polyline(reference)
        ref_pathlength = compute_pathlength_from_polyline(reference)

        # get s coordinates of longitudinal motion for interpolation of theta and kappa of reference
        s = lon_traj.get_positions()[:, 0]
        v = lon_traj.get_velocities()
        a = lon_traj.get_accelerations()
        j = lon_traj.get_jerk()
        # check if numerical errors have happened in lon trajectory
        if ti is not None:
            for i in range(0, len(a)):
                if npy.greater(a[i], ti.a_x_max):
                    a[i] = ti.a_x_max
                if npy.greater(ti.a_x_min, a[i]):
                    a[i] = ti.a_x_min

        assert npy.greater_equal(npy.max(ref_pathlength), npy.max(s)), \
            '<QPLatReference>: Provided reference is not long enough for interpolation! ref = {}, traj = {}'.format(
                npy.max(ref_pathlength), npy.max(s))

        # interpolate curvature at s positions of trajectory
        curvature_interpolated = npy.interp(s, ref_pathlength, ref_curvature)

        # interpolate orientation at s positions of trajectory
        orientation_interpolated = npy.interp(s, ref_pathlength, ref_orientation)
        assert len(curvature_interpolated) == len(orientation_interpolated) == len(s), \
            '<QPLatReference>: interpolation failed!'

        # create QPLat reference
        states = list()
        for i in range(1, len(s)):
            states.append(QPLatRefState(s[i], v[i], a[i], j[i], orientation_interpolated[i], curvature_interpolated[i]))

        return QPLatReference(states)


class QPLatPlanner(TrajectoryPlanner):
    def __init__(self,
                 horizon: float,
                 N: int,
                 dT: float,
                 length: float,
                 slack=False,
                 qp_lat_params: QPLatPARAMS = QPLatPARAMS()):
        """
        Constructor of a QPLatPlanner
        :param horizon: The planning horizon in s
        :param N: The number of time steps of the horizon
        :param dT: The time_step of the planner
        :param length: The distance between rear and front circle of vehicle shape
        :param slack: If slack variable approach should be used
        """
        # set super class variables
        super().__init__(horizon, N, dT)

        # turn off verbose mode as default
        self.verbose = False

        # set parameters
        self._n = 4  # state vector length <d, theta, kappa, kappa dot>
        self._m = 1  # input vector length <kappa dot dot>
        # ---------------------------------------- slack variable ----------------------------------------------------#
        self._n_s = 4  # number of slack variables if slack has been set to True <d_l, d_u, kappa_l, kappa_u>
        # ------------------------------------------------------------------------------------------------------------#

        assert val.is_positive(length), "<QPLatPlanner>: provided wheelbase length " \
                                        "is not valid! length={}".format(length)
        self._length = length

        # define variables and matrices
        self._x = Variable(shape=(self._n, self.N + 1))  # d, theta, kappa, kappa dot

        self._slack = slack
        if self.slack:
            self._u = Variable(shape=(self._m, self.N + self._n_s))
        else:
            self._u = Variable(shape=(self._m, self.N))
        # -------------------------------------------------------------------------------------------------------------

        # store parameters
        self._qp_lat_params = qp_lat_params

        # cost function matrices for state and input
        self._Q = npy.array(
            [[self._qp_lat_params.W_D, 0, 0, 0], [0, self._qp_lat_params.W_THETA, 0, 0],
             [0, 0, self._qp_lat_params.W_KAPPA, 0],
             [0, 0, 0, self._qp_lat_params.W_KAPPA_DOT]])  # for deviation
        self._P = npy.array(
            [[self._qp_lat_params.W_D_N, 0, 0, 0], [0, self._qp_lat_params.W_THETA_N, 0, 0],
             [0, 0, self._qp_lat_params.W_KAPPA_N, 0],
             [0, 0, 0, self._qp_lat_params.W_KAPPA_DOT]])  # for final state deviation
        self._W = npy.array(
            [[1, 0, 0],
             [0, 1, 0],
             [0, 0, 1]])
        self._R = self._qp_lat_params.W_U  # weight for input
        # cost function matrices for slack variables
        self._H_Q = npy.identity(self._n_s) * self._qp_lat_params.W_SLACK_Q
        self._H_L = npy.repeat(1, self._n_s) * self._qp_lat_params.W_SLACK_L


    @property
    def slack(self):
        return self._slack

    @property
    def verbose(self):
        return self._verbose

    @verbose.setter
    def verbose(self, verbose):
        assert isinstance(verbose, bool), "<QPLatPlanner>: verbose flag must be of type bool!"
        self._verbose = verbose

    def plan(self,
             x_initial: QPLatState,
             x_ref: QPLatReference,
             c_ti: TIConstraints,
             c_lat: LatConstraints,
             d_reference=None) -> Trajectory:
        # check if reference has the same size as optimization horizon
        assert x_ref.length() == self.N, "<QPLatPlanner>: reference must have the same length as " \
                                         "optimization horizon. length={}".format(x_ref.length())
        # check length of time-variant constraints
        assert c_lat.N == self.N, "<QPLatPlanner>: lateral time-variant constraints must have the same length as " \
                                  "optimization horizon. length={}".format(c_lat.N)

        if d_reference is not None:
            assert val.is_real_number_vector(d_reference, length=c_lat.N), '<QPLatPlanner/plan>: provided d reference ' \
                                                                        'is not valid! ref = {}'.format(d_reference)

        traj, status, cost = self._cvxpy_plan(x_initial, x_ref, c_ti, c_lat, d_reference)

        if not 'optimal' == status:
            print('\t\t\t Status lateral trajectory planner: {}'.format(status))

        return traj, status

    def _cvxpy_plan(self,
                    x_initial: QPLatState,
                    x_ref: QPLatReference,
                    c_ti: TIConstraints,
                    c_lat: LatConstraints,
                    d_reference=None,) -> Trajectory:
        ############################
        # Define optimization problem
        ############################
        # Calculate cost function
        cost = self.cost_function(x_ref, d_reference)
        # initialize constraints
        constr = []
        # Specify time-variant constraints
        constr += self.tv_constraints(c_lat, c_ti, x_ref)
        # Set up time-invariant constraints
        constr += self.ti_constraints(c_ti, x_initial)

        ############################
        # Solve optimization problem
        ############################
        prob = Problem(Minimize(cost), constr)
        print("Problem is convex:", prob.is_dcp())
        prob.solve(verbose=self.verbose)

        # check result
        if not 'optimal' == prob.status:
            print('Solution not optimal')

        ##########################
        # Create output trajectory
        ##########################
        if not prob.status == 'infeasible':
            trajectory = self.create_output_trajectory(x_ref, x_initial)
        else:
            trajectory = None
        return trajectory, prob.status, prob.value

    def tv_constraints(self, c_lat: LatConstraints, c_ti: TIConstraints, x_ref):
        """
        Specify time-variant transition matrices
        """
        constr = []

        # create all states of the problem along the horizon N
        for k in range(self.N):
            v = x_ref.reference[k].v
            a = x_ref.reference[k].a
            theta = x_ref.reference[k].theta

            # x = Ax+Bu+Dz
            A = npy.array(
                [[1, self.dT * v, (self.dT ** 2) * 0.5 * (v ** 2), (self.dT ** 3) / 6 * (v ** 2)],
                 [0, 1, self.dT * v, (self.dT ** 2) * 0.5 * v],
                 [0, 0, 1, self.dT],
                 [0, 0, 0, 1]])
            B = npy.array([[(self.dT ** 4) / 24 * (v ** 2)],
                           [(self.dT ** 3) / 6 * v],
                           [(self.dT ** 2) * 0.5],
                           [self.dT]])
            # disturbances on input
            D = npy.array([-self.dT * v, 0, 0, 0])
            # selection matrix for output
            S = npy.array([[1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0],
                           [0, 0, 1, 0, 0]])
            C = npy.array([[1, 0, 0, 0],
                           [1, 0.5 * self._length, 0, 0],
                           [1, self._length, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
            # disturbances on output
            E = npy.transpose(npy.array([0, -0.5 * self._length, -self._length, 0, 0]))

            constr += [self._x[:, k + 1] == A @ self._x[:, k] + B @ self._u[:, k] + theta * D]  # state transition

            constr += [S @ (C @ self._x[:, k + 1] + E * theta) <= (c_lat.d_hard_max[k])]  # upper lateral bound
            constr += [S @ (C @ self._x[:, k + 1] + E * theta) >= (c_lat.d_hard_min[k])]  # lower lateral bound

            kappa_lim = npy.min([npy.sqrt(c_ti.a_max ** 2 - a ** 2) / (npy.max([v, 0.5]) ** 2),
                                 c_ti.kappa_max])

            constr += [self._x[2, k + 1] <= kappa_lim]  # curvature constraint Kamm's circle
            constr += [self._x[2, k + 1] >= -kappa_lim]  # curvature constraint Kamm's circle
            # slack constraints
            if self.slack:
                # upper lateral bound
                constr += [S * (C * self._x[:, k + 1] + E * theta) - self._u[self.N] <= (c_tv.d_soft_max[k])]
                # lower lateral bound
                constr += [S * (C * self._x[:, k + 1] + E * theta) + self._u[self.N + 1] >= (c_tv.d_soft_min[k])]

        return constr

    def ti_constraints(self, c_ti: TIConstraints, x_initial):
        """
        Set up time-invariant constraints.
        """
        constr = []
        constr += [self._x[3, :] <= c_ti.kappa_dot_max]
        constr += [self._x[3, :] >= c_ti.kappa_dot_min]
        constr += [self._u[:, :] <= c_ti.kappa_dot_dot_max]  # input constraint
        constr += [self._u[:, :] >= c_ti.kappa_dot_dot_min]  # input constraint
        constr += [self._x[:2, 0] == x_initial.to_array()[:2]]
        if self.slack:
            constr += [self._u[:, self.N:].T >= npy.repeat(0, self._n_s)]
        return constr

    def cost_function(self, x_ref, d_reference):
        """
        Define cost function including reference.
        """
        cost = 0
        # create all states of the problem along the horizon N
        for k in range(self.N):
            #########################################
            # Define cost function including reference
            #########################################

            theta = x_ref.reference[k].theta
            kappa = x_ref.reference[k].kappa

            x_desired = [0 if d_reference is None else d_reference[k], theta, kappa, 0]
            cost += quad_form(self._x[:, k + 1] - x_desired, self._Q) + square(self._u[:, k]) * self._R
        return cost

    def create_output_trajectory(self, x_ref, x_initial):
        """
        Generates output trajectory.
        """
        traj = []
        ref = x_ref.reference
        # add initial state
        assert x_initial.s is not None and x_initial.v is not None and x_initial.a is not None, \
            '<QPLateralPlanner>: initial long state information missing!'
        traj.append(
            TrajPoint(x_initial.t, x_initial.s, x_initial.d, x_initial.theta,
                      x_initial.v, x_initial.a, j=x_initial.j, kappa=x_initial.kappa, kappa_dot=x_initial.kappa_dot,
                      lane=-1))

        for k in range(self.N):
            traj.append(
                TrajPoint(t=x_initial.t + (k + 1) * self.dT,
                          x=ref[k].s,
                          y=self._x[0, k + 1].value,
                          theta=self._x[1, k + 1].value,
                          v=ref[k].v,
                          a=ref[k].a,
                          kappa=self._x[2, k + 1].value,
                          j=ref[k].j,
                          kappa_dot=self._x[3, k + 1].value,
                          lane=-1))

        traj = Trajectory(traj, TrajectoryType.CARTESIAN)
        traj._u_lon = x_initial.u_lon
        traj._u_lat = npy.transpose(self._u.value.flatten())[:self.N]
        return traj