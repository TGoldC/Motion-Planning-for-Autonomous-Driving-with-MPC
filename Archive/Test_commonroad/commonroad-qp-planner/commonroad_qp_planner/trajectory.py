import math

import numpy as np
import enum
import copy

from typing import List

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.prediction.prediction import TrajectoryPrediction
import commonroad.scenario.trajectory as cr_traj
from commonroad.geometry.shape import Rectangle
import commonroad.common.validity as val

import warnings


class TrajectoryType(enum.Enum):
    CARTESIAN = 0
    FRENET = 1


class TrajPoint(object):
    def __init__(self,
                 t: float,
                 x: float,
                 y: float,
                 theta: float,
                 v: float,
                 a: float,
                 kappa=0.,
                 j=0,
                 kappa_dot=0,
                 lane=-1):
        assert isinstance(t, val.ValidTypes.NUMBERS)
        assert isinstance(v, val.ValidTypes.NUMBERS) and v >= -1., \
            '<TrajPoint>: Provided velocity not valid. v= {}'.format(a)
        assert isinstance(a, val.ValidTypes.NUMBERS), '<TrajPoint>: Provided acceleration not valid. a= {}'.format(a)
        assert isinstance(x, val.ValidTypes.NUMBERS), '<TrajPoint>: Provided x coordinate not valid. x= {}'.format(x)
        assert isinstance(y, val.ValidTypes.NUMBERS), '<TrajPoint>: Provided y coordinate not valid. y= {}'.format(y)
        assert isinstance(theta, val.ValidTypes.NUMBERS), \
            '<TrajPoint>: Provided orientation not valid. theta = {}'.format(theta)
        assert isinstance(lane, int) and lane >= -1

        self._t = t
        self._position = np.array([x, y])
        self._lane = lane
        self._orientation = theta
        self._v = v
        self._a = a
        self._kappa = kappa
        self._kappa_dot = kappa_dot
        self._j = j

    @property
    def t(self) -> float:
        return self._t

    @t.setter
    def t(self, t):
        pass

    @property
    def position(self) -> np.ndarray:
        return self._position

    @position.setter
    def position(self, position):
        pass

    @property
    def orientation(self) -> float:
        return self._orientation

    @orientation.setter
    def orientation(self, orientation):
        pass

    @property
    def v(self) -> float:
        return self._v

    @v.setter
    def v(self, v):
        pass

    @property
    def a(self) -> float:
        return self._a

    @a.setter
    def a(self, a):
        pass

    @property
    def kappa(self) -> float:
        return self._kappa

    @kappa.setter
    def kappa(self, kappa):
        pass

    @property
    def kappa_dot(self) -> float:
        return self._kappa_dot

    @kappa_dot.setter
    def kappa_dot(self, kappa_dot):
        pass

    @property
    def j(self) -> float:
        return self._j

    @j.setter
    def j(self, j: float):
        pass

    @property
    def lane(self) -> int:
        return self._lane

    @lane.setter
    def lane(self, lane):
        pass

    def to_list(self) -> list:
        return [self.t, self.position[0], self.position[1], self.orientation, self.v, self.a, self.kappa, self.lane]

    def __str__(self):
        return '<TrajPoint>:  (t={}, position={}, orientation={}, v={}, a={}, j ={}, ' \
               'kappa = {}, kappa_dot = {}, lane={})'.format(
            self.t, self.position, self.orientation, self.v, self.a, self.j, self.kappa, self.kappa_dot, self.lane)

    def __eq__(self, other):
        if isinstance(other, TrajPoint):
            return all(np.equal(self.to_list(), other.to_list()))


class Trajectory(object):
    def __init__(self, traj_points: list, coord_type: TrajectoryType) -> object:
        assert isinstance(coord_type, TrajectoryType), \
            '<Trajectory>: Provided trajectory type is not valid. type = {}'.format(coord_type)
        assert isinstance(traj_points, list) and (isinstance(t, TrajPoint) for t in traj_points), \
            '<Trajectory>: Provided trajectory points invalid. points = {}'.format(traj_points)
        assert self.__valid_trajectory(traj_points), \
            '<Trajectory>: Provided trajectory has invalid time information. points = {}'.format(traj_points)

        self._traj_points = traj_points
        self._t_0 = traj_points[0].t
        self._t_h = traj_points[-1].t
        self._dT = traj_points[1].t - traj_points[0].t

        self._N = len(traj_points)
        self._coord_type = coord_type
        self._u_lon = None
        self._u_lat = None

    @property
    def u_lon(self) -> list:
        return self._u_lon

    @property
    def u_lat(self) -> list:
        return self._u_lat

    @property
    def t_0(self) -> float:
        return self._t_0

    @t_0.setter
    def t_0(self, t_0):
        pass

    @property
    def t_h(self) -> float:
        return self._t_h

    @t_h.setter
    def t_h(self, t_h):
        pass

    @property
    def N(self) -> int:
        return self._N

    @N.setter
    def N(self, N):
        pass

    @property
    def dT(self):
        return self._dT

    @dT.setter
    def dT(self, dT):
        pass

    @property
    def states(self) -> List[TrajPoint]:
        return self._traj_points

    @states.setter
    def states(self, states):
        pass

    @property
    def coord_type(self) -> TrajectoryType:
        return self._coord_type

    @coord_type.setter
    def coord_type(self, coord_type):
        pass

    def reset_time(self, t_0: val.ValidTypes.NUMBERS):
        """
        Resets the initial time of trajectory
        :param t_0: The new initial time of the trajectory
        """
        assert val.is_real_number(t_0), '<Trajectory/reset_time>: provided t_0 is not valid! t_0 new = {} t_0 ' \
                                        'old = {}'.format(t_0, self.t_0)

        self._t_0 = t_0
        self._t_h = t_0 + len(self.states) * self.dT

        for i in range(len(self.states)):
            s = self._traj_points[i]
            s._t = t_0 + i * self.dT
            self._traj_points[i] = s

    def __valid_trajectory(self, traj_points: list):

        # check inputs
        assert len(traj_points) > 1

        # result of validation
        valid = True

        # step size
        dT = traj_points[1].t - traj_points[0].t

        # check if dT is positive
        valid = valid and dT > 0

        # check trajectory times
        for i in range(1, len(traj_points)):
            # dT must be positive and every step size must be equal
            if not np.isclose(dT, traj_points[i].t - traj_points[i - 1].t):
                warnings.warn(
                    'Expected dt = {} but received {} at i={}'.format(dT, traj_points[i].t - traj_points[i - 1].t, i))
                valid = False
                i = len(traj_points)
                continue

        return valid

    def get_positions(self) -> np.ndarray:
        """
        Returns a numpy array with all positions inside the trajectory
        :return: The numpy array of positions
        """
        pos = list()
        for s in self.states:
            pos.append(s.position)

        return np.array(pos)

    def get_orientations(self) -> np.ndarray:
        """
        Returns a numpy array with all orientations inside the trajectory
        :return: The numpy array of orientations
        """
        theta = list()
        for s in self.states:
            theta.append(s.orientation)

        return np.array(theta)

    def get_velocities(self) -> np.ndarray:
        """
        Returns a numpy array with all velocities inside the trajectory
        :return: The numpy array of velocities
        """
        vs = list()
        for s in self.states:
            vs.append(s.v)

        return np.array(vs)

    def get_time(self) -> np.ndarray:
        """
        Returns a numpy array with all times inside the trajectory
        :return: The numpy array of times
        """
        ts = list()
        for s in self.states:
            ts.append(s.t)

        return np.array(ts)

    def get_accelerations(self) -> np.ndarray:
        """
        Returns a numpy array with all accelerations inside the trajectory
        :return: The numpy of accelerations
        """
        a = list()
        for s in self.states:
            a.append(s.a)

        return np.array(a)

    def get_jerk(self) -> np.ndarray:
        """
        Returns a numpy array with all jerks inside the trajectory
        :return: The numpy of jerks
        """
        j = list()
        for s in self.states:
            j.append(s.j)

        return np.array(j)

    def get_kappa(self) -> np.ndarray:
        """
        Returns a numpy array with all kappas inside the trajectory
        :return: The numpy of jerks
        """
        k = list()
        for s in self.states:
            k.append(s.kappa)

        return np.array(k)

    def get_kappa_dot(self) -> np.ndarray:
        """
        Returns a numpy array with all kappa dots inside the trajectory
        :return: The numpy of jerks
        """
        k = list()
        for s in self.states:
            k.append(s.kappa_dot)

        return np.array(k)

    def cartesian_ptsX(self) -> List[float]:
        x = []
        for p in self.states:
            x.append(p.position[0])
        return x

    def cartesian_ptsY(self) -> List[float]:
        y = []
        for p in self.states:
            y.append(p.position[1])
        return y

    def convert_to_cr_trajectory(self, wheelbase: float):
        """
        Converts the current trajectory to a CommonRoad trajectory and StateTupleFactory
        :return: CommonRoad trajectory object, StateTupleFactory
        """

        def convert_curvature_to_angle(kappa: float, wheelbase: float = 2.471) -> float:
            # tan(steering angle) = wheelbase/radius
            if kappa > 0 or kappa < 0:
                steering_angle = np.arctan(wheelbase / (1 / kappa))
            else:
                steering_angle = 0.0
            return steering_angle

        traj_cr = list()

        # transform every trajectory point
        for p in self.states:
            state_values = dict()

            # add position
            state_values['position'] = p.position + wheelbase / 2 * np.array([np.cos(p.orientation),
                                                                              np.sin(p.orientation)])
            # add orientation
            state_values['orientation'] = float(p.orientation)
            # add velocity
            state_values['velocity'] = float(p.v)
            state_values['velocity_y'] = float(p.v*math.sin(p.orientation))
            # # add acceleration
            state_values['acceleration'] = float(p.a)
            state_values['yaw_rate'] = 0.0
            state_values['steering_angle'] = convert_curvature_to_angle(p.kappa, wheelbase)
            state_values['slip_angle'] = 0.0
            # add time_idx
            state_values['time_step'] = int(round(p.t / self.dT))
            # add steering angle

            state = cr_traj.State(**state_values)
            traj_cr.append(state)

        return cr_traj.Trajectory(0, traj_cr)

    def convert_to_cr_ego_vehicle(self,
                                  width: float,
                                  length: float,
                                  wheelbase: float,
                                  vehicle_id: int = 0) -> DynamicObstacle:
        """
        Converts trajectory object to CommonRoad obstacle with specified width and length
        :param width: The width of the ego vehicle
        :param length: The length of the ego vehicle
        :param vehicle_id: ID of ego vehicle
        :return: The CommonRoad DynamicObstacle object containing the current trajectory
        """
        assert isinstance(width, (
            int, float)) and width > 0, '<Trajectory>: Provided width of vehicle invalid! width = {}'.format(width)
        assert isinstance(length, (int, float)) and length > 0, '<Trajectory>: Provided width of vehicle ' \
                                                                'invalid! width = {}'.format(length)

        # get trajectory
        traj = self.convert_to_cr_trajectory(wheelbase)
        shape = Rectangle(length, width)
        pred = TrajectoryPrediction(traj, shape)

        # create new object
        ego = DynamicObstacle(obstacle_id=vehicle_id,
                              obstacle_type=ObstacleType.CAR,
                              prediction=pred,
                              obstacle_shape=shape,
                              initial_state=traj.state_list[0])
        return ego

    @classmethod
    def concatenate_trajectories(cls, pre: 'Trajectory', post: 'Trajectory') -> 'Trajectory':
        """
        Concatenates two subsequent trajectories pre and post to a new trajectory. Both trajectories are connected
        if the end and start state of pre and post are connected in epsilon region.
        :param pre: The pre trajectory
        :param post: The post trajectory
        :return: The concatenated trajectory pre||post
        """

        assert isinstance(pre, Trajectory), '<Trajectory/concatenate>: pre trajectory is not valid! pre = {}'.format(
            pre)
        assert isinstance(post, Trajectory), '<Trajectory/concatenate>: post trajectory is not valid! post = {}'.format(
            post)
        assert pre.states[-1] == post.states[
            0], '<Trajectory/concatenate>: pre and post are not connected! pre = {} and post = {}'.format(
            pre.states[-1], post.states[0])
        assert pre.coord_type == post.coord_type, '<Trajectory/concatenate>: pre and post are in different ' \
                                                  'coordinate systems! pre = {} and ' \
                                                  'post = {}'.format(pre.coord_type, post.coord_type)

        state_list = pre.states + post.states[1:]
        traj = Trajectory(state_list, pre.coord_type)
        traj._u_lon = np.concatenate((pre.u_lon, post.u_lon))
        traj._u_lat = np.concatenate((pre.u_lat, post.u_lat))

        return traj

    def split_at_state(self, index: int) -> ('Trajectory', 'Trajectory'):
        """
        Splits a trajectory at a given state
        :param index: The index of the state for the splitting
        :return: Tuple (pre,pos) where pre is the part of the trajectory including split state and post is trajectory
        starting after split state
        """

        assert val.is_natural_number(
            index), '<Trajectory/split_at_state>: Provided index is not valid! index = {}'.format(index)
        assert val.is_in_interval(index, 0, self.N - 1), '<Trajectory/split_at_state>: Provided index ' \
                                                         'is out of range! index = {}, N = {}'.format(index, self.N)

        pre = Trajectory(self.states[:(index + 1)], self.coord_type)
        pre._u_lon = self._u_lon[:index]
        pre._u_lat = self._u_lat[:index]

        if index < (self.N - 1):
            post = Trajectory(self.states[(index + 1):], self.coord_type)
            post._u_lon = self._u_lon[(index + 1):]
            post._u_lat = self._u_lat[(index + 1):]
        else:
            post = []
            warnings.warn('Empty trajectory since requested index is last state!')

        return pre, post

    def split_at_time(self, t: float) -> ('Trajectory', 'Trajectory'):
        """
        Splits a trajectory at a given time t
        :param t: The time at which the trajectory should be split
        :return: Tuple (pre,post) where pre is the part of the trajectory before/possibly including the provided
        time and post after the time
        """

        assert val.is_real_number(t) and np.greater_equal(t, 0.), '<Trajectory/split_at_time>: Provided time ' \
                                                                  'is not valid! time = {}'.format(t)
        assert val.is_in_interval(t, self.t_0, self.t_h), '<Trajectory/split_at_time>: Provided time is outside ' \
                                                          'of trajectory! t = {}'.format(t)

        dist = np.abs(self.get_time() - t)
        index = np.argmin(dist)

        # check if t is precise enough
        if not np.equal(dist[index], 0.):
            warnings.warn('<Trajectory/split_at_time>: provided time is between two states!')

        return self.split_at_state(index)

    def split_at_position(self, pos: list) -> ('Trajectory', 'Trajectory'):
        """
        Splits a trajectory at a given position
        :param pos: The position at which the trajectory should be split
        :return: Tuple (pre,post) where pre is the part of the trajectory before/including the provided
        position and post may contain the position
        """

        assert val.is_real_number_vector(pos, length=2), '<Trajectory/split_at_position>: Provided position ' \
                                                         'is not valid! pos = {}'.format(pos)

        # compute Euclidean distance to each state of trajectory
        dist = (self.get_positions() - pos) ** 2
        dist = np.sqrt(dist[:, 0 + dist[:, 1]])

        # find minimum distance
        index = np.argmin(dist)

        # check if distance is greater than time step at 130km/h (in order to detect false positions)
        threshhold = 130 / 3.6 / (1. / self.dT)
        if np.greater_equal(dist[index], threshhold):
            warnings.warn(
                '<Trajectory/split_at_position>: provided position seems to be too far away! pos = {}'.format(pos))

        return self.split_at_state(index)

    def shorten(self, horizon: float) -> 'Trajectory':
        """
        Shortens a trajectory according to the specified shorter horizon
        :param horizon: The new horizon which is shorter than the current one
        :return: The shortened trajectory with the new horizon
        """

        assert val.is_positive(horizon), '<Trajectory/shorten>: Provided horizon is not valid! horizon = {}'.format(
            horizon)
        assert np.greater_equal(self.t_h - self.t_0, horizon), '<Trajectory/shorten>: Provided horizon is larger ' \
                                                               'than current one! horizon {} <= {}'.format(
            horizon, self.t_h - self.t_0)

        index = np.floor(horizon / self.dT)

        # check if trajectory is actually shortened
        if index == self.N - 1:
            warnings.warn('<Trajectory/shorten>: Trajectory is shortened at last state; horizon remains the same!')

        return self.split_at_state(index)[0]

    def resample(self, output_sampling_time, subsampling_factor=1) -> (list, list, list, list):
        planning_steps = len(self.u_lon)
        print('DT IS = {}'.format(self.dT))
        # planning time has length planning_steps+1, i.e. for the last entry in planning_times no u is given
        planning_time = np.cumsum([self.t_0] + [self.dT] * planning_steps)
        input_time = planning_time[:-1]

        prediction_sampling_time = output_sampling_time / subsampling_factor
        dt = prediction_sampling_time
        prediction_time = np.arange(planning_time[0], planning_time[-1], prediction_sampling_time)
        # find indexes of last input before prediction_time (assume sample and hold for inputs)
        input_idxs = np.searchsorted(input_time, prediction_time, 'right') - 1

        A = np.array([[1, dt, dt ** 2 / 2., dt ** 3 / 6.], [0, 1, dt, dt ** 2 / 2], [0, 0, 1, dt], [0, 0, 0, 1]])
        B = np.array([dt ** 4 / 24., dt ** 3 / 6., dt ** 2 / 2., dt])

        x_0 = np.array([self.states[0].position[0], self.states[0].v, self.states[0].a, self.states[0].j]).transpose()

        r_s = list()
        r_v = list()
        r_a = list()
        r_j = list()

        r_s.append(x_0[0])
        r_v.append(x_0[1])
        r_a.append(x_0[2])
        r_j.append(x_0[3])

        for i in range(len(input_idxs)):
            x_0 = A.dot(x_0) + B * self.u_lon[input_idxs[i]]
            r_s.append(x_0[0])
            r_v.append(x_0[1])
            r_a.append(x_0[2])
            r_j.append(x_0[3])

        x_0 = np.array([self.states[0].position[1], self.states[0].orientation, self.states[0].kappa,
                        self.states[0].kappa_dot]).transpose()

        r_d = list()
        r_theta = list()
        r_kappa = list()
        r_kappa_dot = list()

        r_d.append(x_0[0])
        r_theta.append(x_0[1])
        r_kappa.append(x_0[2])
        r_kappa_dot.append(x_0[3])

        for i in range(len(input_idxs)):
            v = r_v[i + 1]
            A = np.array(
                [[1, dt * v, dt ** 2 * 0.5 * (v ** 2), (dt ** 3 / 6.) * (v ** 2)], [0, 1, dt * v, (dt ** 2 / 2) * v],
                 [0, 0, 1, dt], [0, 0, 0, 1]])
            B = np.array([(dt ** 4 / 24.) * (v ** 2), (dt ** 3 / 6.) * v, (dt ** 2 / 2.), dt])
            x_0 = A.dot(x_0) + B * self.u_lat[input_idxs[i]]
            r_d.append(x_0[0])
            r_theta.append(x_0[1])
            r_kappa.append(x_0[2])
            r_kappa_dot.append(x_0[3])

        # use sampling factor

        r_s = r_s[::subsampling_factor]
        r_v = r_v[::subsampling_factor]
        r_a = r_a[::subsampling_factor]
        r_j = r_j[::subsampling_factor]

        r_d = r_d[::subsampling_factor]
        r_theta = r_theta[::subsampling_factor]
        r_kappa = r_kappa[::subsampling_factor]
        r_kappa_dot = r_kappa_dot[::subsampling_factor]
        prediction_time = np.concatenate((prediction_time, [self.t_h]))
        prediction_time = prediction_time[::subsampling_factor]

        # new states
        states = list()
        for i in range(len(r_s)):
            states.append(
                TrajPoint(prediction_time[i], r_s[i], r_d[i], r_theta[i], r_v[i] if r_v[i] >= 0. else 0., r_a[i],
                          r_kappa[i], r_j[i], r_kappa_dot[i], -1))
        traj = Trajectory(states, self.coord_type)
        traj._u_lon = self._u_lon
        traj._u_lat = self._u_lat
        return traj

    def compute_history_for_initial_state(self, horizon: float):
        N = horizon / self.dT
        ind = np.flipud(np.arange(0, N + 1))

        states = list()
        for i in ind:
            x_0 = copy.deepcopy(self.states[0])
            x_0._t = self.t_0 - self.dT * i
            # print(x_0)
            states.append(x_0)
        traj = Trajectory(states, self.coord_type)
        traj._u_lon = np.zeros(len(ind) - 1)
        traj._u_lat = np.zeros(len(ind) - 1)
        return traj

    def sub_trajectory(self, t_0, N):

        t = self.get_time()
        dist = np.abs(t - t_0)
        index_low = np.argmin(dist)
        if index_low > 0 and np.greater(t[index_low], t_0):
            index_low -= 1

        index_up = np.minimum(index_low + N, self.N - 1)

        return Trajectory(self.states[index_low:(index_up + 1)], self.coord_type) if index_low < index_up - 1 else []
