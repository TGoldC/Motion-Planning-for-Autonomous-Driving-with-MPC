import numpy as np 
import casadi as ca
from typing import Dict, Union, List, Tuple, Any
import enum

from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from vehiclemodels.utils.vehicle_dynamics_ks_cog import vehicle_dynamics_ks_cog
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from vehiclemodels.vehicle_dynamics_std import vehicle_dynamics_std
from commonroad_route_planner.route_planner import RoutePlanner

# commonroad-io
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.common.validity import is_real_number
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet

# commonroad-curvilinear-coordinate-system
import commonroad_dc.pycrccosy as pycrccosy
from commonroad_dc.geometry.util import (chaikins_corner_cutting, compute_curvature_from_polyline, resample_polyline)


VEHICLE_ID = int
TIME_IDX = int


class ReferencePoint(enum.Enum):
    """
    The reference point for the ego vehicle to describe the kinematics of the ego vehicle.
    If we choose the rear axis, the slip angle can be assumed to be zero at low speeds and generally
    neglected at high speeds.
    """
    CENTER = "center"
    REAR = "rear"


class PlanningConfigurationVehicle:
    """ Class which holds all necessary vehicle-specific parameters for the
    trajectory planning."""

    def __init__(self):
        """
        Default settings.
        """
        self.vehicle_id = -1
        self.min_speed_x = -np.inf
        self.max_speed_x = +np.inf
        self.min_speed_y = -np.inf
        self.max_speed_y = +np.inf
        self.a_max_x = 12.0
        self.a_min_x = -10.0
        self.a_max_y = 6.0
        self.a_min_y = -6.0
        self.a_max = 15.0
        self.j_min_x = -15.0
        self.j_max_x = 15.0
        self.j_min_y = -10.0
        self.j_max_y = 10.0
        self.desired_speed = 0.0
        self.wheelbase = 3.0
        self.length = 4.5
        self.width = 2.0
        self._curvilinear_coordinate_system = None
        self._reference_path = None
        self._reference_point = ReferencePoint.REAR
        self._lanelet_network = None

    @property
    def vehicle_id(self) -> VEHICLE_ID:
        """ Unique ID of the vehicle, e.g, the planning problem ID from CommonRoad."""
        return self._vehicle_id

    @vehicle_id.setter
    def vehicle_id(self, vehicle_id: VEHICLE_ID):
        assert (type(vehicle_id) is VEHICLE_ID), '<PlanConfiguration/vehicle_id> Expected type int; ' \
                                                 'Got type %s instead.' % (type(vehicle_id))
        self._vehicle_id = vehicle_id

    @property
    def min_speed_x(self) -> float:
        """ Minimum speed of the vehicle in longitudinal direction."""
        return self._min_speed_x

    @min_speed_x.setter
    def min_speed_x(self, min_speed_x: float):
        assert (type(min_speed_x) is float), '<PlanConfiguration/min_speed_x> Expected type float; ' \
                                             'Got type %s instead.' % (type(min_speed_x))
        self._min_speed_x = min_speed_x

    @property
    def max_speed_x(self) -> float:
        """ Maximum speed of the vehicle in longitudinal direction."""
        return self._max_speed_x

    @max_speed_x.setter
    def max_speed_x(self, max_speed_x: float):
        assert (type(max_speed_x) is float), '<PlanConfiguration/max_speed_x> Expected type float; ' \
                                             'Got type %s instead.' % (type(max_speed_x))
        self._max_speed_x = max_speed_x

    @property
    def min_speed_y(self) -> float:
        """ Minimum speed of the vehicle in lateral direction."""
        return self._min_speed_y

    @min_speed_y.setter
    def min_speed_y(self, min_speed_y: float):
        assert (type(min_speed_y) is float), '<PlanConfiguration/min_speed_y> Expected type float; ' \
                                             'Got type %s instead.' % (type(min_speed_y))
        self._min_speed_y = min_speed_y

    @property
    def max_speed_y(self) -> float:
        """ Maximum speed of the vehicle in lateral direction."""
        return self._max_speed_y

    @max_speed_y.setter
    def max_speed_y(self, max_speed_y: float):
        assert (type(max_speed_y) is float), '<PlanConfiguration/max_speed_y> Expected type float; ' \
                                             'Got type %s instead.' % (type(max_speed_y))
        self._max_speed_y = max_speed_y

    @property
    def a_max_x(self) -> float:
        """ Maximum acceleration of the vehicle in longitudinal direction."""
        return self._a_max_x

    @a_max_x.setter
    def a_max_x(self, a_max_x: float):
        assert (type(a_max_x) is float), '<PlanConfiguration/a_max_x> Expected type float; ' \
                                         'Got type %s instead.' % (type(a_max_x))
        self._a_max_x = a_max_x

    @property
    def a_min_x(self) -> float:
        """ Minimum acceleration of the vehicle in longitudinal direction."""
        return self._a_min_x

    @a_min_x.setter
    def a_min_x(self, a_min_x: float):
        assert (type(a_min_x) is float), '<PlanConfiguration/a_min_x> Expected type float; ' \
                                         'Got type %s instead.' % (type(a_min_x))
        self._a_min_x = a_min_x

    @property
    def a_max_y(self) -> float:
        """ Maximum acceleration of the vehicle in lateral direction."""
        return self._a_max_y

    @a_max_y.setter
    def a_max_y(self, a_max_y: float):
        assert (type(a_max_y) is float), '<PlanConfiguration/a_max_y> Expected type float; ' \
                                         'Got type %s instead.' % (type(a_max_y))
        self._a_max_y = a_max_y

    @property
    def a_min_y(self) -> float:
        """ Minimum acceleration of the vehicle in lateral direction."""
        return self._a_min_y

    @a_min_y.setter
    def a_min_y(self, a_min_y: float):
        assert (type(a_min_y) is float), '<PlanConfiguration/a_min_y> Expected type float; ' \
                                         'Got type %s instead.' % (type(a_min_y))
        self._a_min_y = a_min_y

    @property
    def a_max(self) -> float:
        """ Maximum overall acceleration of the vehicle."""
        return self._a_max

    @a_max.setter
    def a_max(self, a_max: float):
        assert (type(a_max) is float), '<PlanConfiguration/a_max> Expected type float; ' \
                                       'Got type %s instead.' % (type(a_max))
        self._a_max = a_max

    @property
    def j_min_x(self) -> float:
        """ Minimum jerk in longitudinal direction."""
        return self._j_min_x

    @j_min_x.setter
    def j_min_x(self, j_min_x: float):
        assert isinstance(j_min_x, float), '<PlanConfiguration/j_min_x> Expected type float; ' \
                                           'Got type %s instead.' % (type(j_min_x))
        self._j_min_x = j_min_x

    @property
    def j_max_x(self) -> float:
        """ Maximum jerk of the vehicle in longitudinal direction."""
        return self._j_max_x

    @j_max_x.setter
    def j_max_x(self, j_max_x: float):
        assert isinstance(j_max_x, float), '<PlanConfiguration/j_max_x> Expected type float; ' \
                                           'Got type %s instead.' % (type(j_max_x))
        self._j_max_x = j_max_x

    @property
    def j_min_y(self) -> float:
        """ Minimum jerk in lateral direction."""
        return self._j_min_y

    @j_min_y.setter
    def j_min_y(self, j_min_y: float):
        assert isinstance(j_min_y, float), '<PlanConfiguration/j_min_y> Expected type float; ' \
                                           'Got type %s instead.' % (type(j_min_y))
        self._j_min_y = j_min_y

    @property
    def j_max_y(self) -> float:
        """ Maximum jerk in lateral direction."""
        return self._j_max_y

    @j_max_y.setter
    def j_max_y(self, j_max_y: float):
        assert isinstance(j_max_y, float), '<PlanConfiguration/j_max_y> Expected type float; ' \
                                           'Got type %s instead.' % (type(j_max_y))
        self._j_max_y = j_max_y

    @property
    def desired_speed(self) -> float:
        """ Desired speed of the vehicle for trajectory planning."""
        return self._desired_speed

    @desired_speed.setter
    def desired_speed(self, desired_speed: float):
        assert isinstance(desired_speed, float), '<PlanConfiguration/desired_speed> Expected type float; ' \
                                                 'Got type %s instead.' % (type(desired_speed))
        self._desired_speed = desired_speed

    @property
    def wheelbase(self) -> float:
        """
        The vehicle’s shape is approximated with three circles with equal radius. The centers of the first and third
        circle coincides with the rear and front axle, respectively. The distance between the first and third center
        is the wheelbase."""
        return self._wheelbase

    @wheelbase.setter
    def wheelbase(self, wheelbase: float):
        assert (type(wheelbase) is float), '<PlanConfiguration/wheelbase> ' \
                                           'Expected type float; Got type %s instead.' % (type(wheelbase))
        self._wheelbase = wheelbase

    @property
    def length(self) -> float:
        """ Length of the vehicle."""
        return self._length

    @length.setter
    def length(self, length: float):
        assert is_real_number(length), '<PlanConfiguration/length>: argument "length" is not a real number. ' \
                                       'length = {}'.format(length)
        self._length = length

    @property
    def width(self) -> float:
        """ Width of the vehicle."""
        return self._width

    @width.setter
    def width(self, width: float):
        assert is_real_number(width), '<PlanConfiguration/width>: argument "width" is not a real number. ' \
                                      'width = {}'.format(width)
        self._width = width

    @property
    def curvilinear_coordinate_system(self) -> pycrccosy.CurvilinearCoordinateSystem:
        """ Curvilinear coordinate system for the reachable set computation in curvilinear coordinates."""
        if self._curvilinear_coordinate_system is None:
            self._curvilinear_coordinate_system = pycrccosy.CurvilinearCoordinateSystem(self.reference_path)

        return self._curvilinear_coordinate_system

    @curvilinear_coordinate_system.setter
    def curvilinear_coordinate_system(
            self, curvilinear_coordinate_system: pycrccosy.CurvilinearCoordinateSystem):
        assert (isinstance(curvilinear_coordinate_system, pycrccosy.CurvilinearCoordinateSystem)), \
            '<PlanConfiguration/curvilinear_coordinate_system> Expected type ' \
            'pycrccosy.PolylineCoordinateSystem; Got type %s instead.' % (type(curvilinear_coordinate_system))
        self._curvilinear_coordinate_system = curvilinear_coordinate_system

    @property
    def reference_path(self):
        """ Reference path of the vehicle for the generation of a curvilinear coordinate system or trajectory
        planning. The reference path must be given as polyline."""
        return self._reference_path

    @reference_path.setter
    def reference_path(self, reference_path: np.ndarray):
        assert isinstance(reference_path, np.ndarray) and reference_path.ndim == 2 \
               and len(reference_path) > 1 and len(reference_path[0, :]) == 2, \
               '<PlanConfiguration/reference_path>: Provided reference is not valid. reference = {}'. \
               format(reference_path)
        self._reference_path = reference_path

    @property
    def reference_point(self) -> ReferencePoint:
        return self._reference_point

    @reference_point.setter
    def reference_point(self, reference_point: ReferencePoint):
        assert isinstance(reference_point, ReferencePoint), \
            '<ReachSetConfiguration/reference_point>: argument reference_point of wrong type. Expected type: %s. ' \
            'Got type: %s' % (ReferencePoint, type(reference_point))
        self._reference_point = reference_point

    @property
    def lanelet_network(self) -> Union[None, LaneletNetwork]:
        """ The part of the lanelet network of the scenario, the vehicle is allowed or should drive on."""
        return self._lanelet_network

    @lanelet_network.setter
    def lanelet_network(self, lanelet_network: LaneletNetwork):
        assert isinstance(lanelet_network, LaneletNetwork), '<PlanConfiguration/lanelet_network>: argument ' \
                                                            'lanelet_network of wrong type. Expected type: %s. ' \
                                                            'Got type: %s.' % (LaneletNetwork, type(lanelet_network))
        self._lanelet_network = lanelet_network


class VehicleDynamics(object):
    def __init__(self, p=parameters_vehicle2()):
        self.l = p.a + p.b
        self.g = 9.81
        self.mu = p.tire.p_dy1
        self.C_Sf = -p.tire.p_ky1/p.tire.p_dy1
        self.C_Sr = -p.tire.p_ky1/p.tire.p_dy1
        self.lf = p.a
        self.lr = p.b
        self.h = p.h_s
        self.m = p.m
        self.I = p.I_z
        self.p = p

    @classmethod
    def KS_casadi(self, x, u):
        """Defines dynamics of the car, i.e. equality constraints.
        parameters:
        state x = [xPos,yPos,delta,v,psi]
        input u = [deltaDot,aLong]
        """
        p = parameters_vehicle2()
        l = p.a + p.b
        return ca.vertcat(x[3] * ca.cos(x[4]),
                              x[3] * ca.sin(x[4]),
                              u[0],
                              u[1],
                              x[3] / l * ca.tan(x[2]))

    def KS(self, states, controls, type ='casadi'):
        if type == 'casadi':
            f = ca.vertcat(states[3]*ca.cos(states[4]))
            f = ca.vertcat(f, states[3]*ca.sin(states[4]))
            f = ca.vertcat(f, controls[0])
            f = ca.vertcat(f, controls[1])
            f = ca.vertcat(f, (ca.tan(states[2])*states[3])/self.l)
        else:
            f = vehicle_dynamics_ks(states, controls, self.p)
        return f

    def ST(self, x, u, type ='casadi'):
        if type == 'casadi':
            if abs(x[3]) < 0.1:
                x_ks = [x[0],  x[1],  x[2],  x[3],  x[4]]
                f_ks = vehicle_dynamics_ks_cog(x_ks, u, self.p)
                f = [f_ks[0],  f_ks[1],  f_ks[2],  f_ks[3],  f_ks[4]]
                d_beta = (self.lr * u[0]) / (self.l*ca.cos(x[2])**2 * (1 + (ca.tan(x[2])**2 * self.lr/self.l)**2))
                dd_psi = 1/self.l * (u[1]*ca.cos(x[6])*ca.tan(x[2]) - x[3]*ca.sin(x[6])*d_beta*ca.tan(x[2]) + x[3]*ca.cos(x[6])*u[0]/ca.cos(x[2])**2)
                f.append(dd_psi)
                f.append(d_beta)
            else:
                f = [x[3]*ca.cos(x[6] + x[4]),
                    x[3]*ca.sin(x[6] + x[4]),
                    u[0],
                    u[1],
                    x[5],
                    -self.mu*self.m/(x[3]*self.I*(self.lr+self.lf))*(self.lf**2*self.C_Sf*(self.g*self.lr-u[1]*self.h) + self.lr**2*self.C_Sr*(self.g*self.lf + u[1]*self.h))*x[5] \
                        +self.mu*self.m/(self.I*(self.lr+self.lf))*(self.lr*self.C_Sr*(self.g*self.lf + u[1]*self.h) - self.lf*self.C_Sf*(self.g*self.lr - u[1]*self.h))*x[6] \
                        +self.mu*self.m/(self.I*(self.lr+self.lf))*self.lf*self.C_Sf*(self.g*self.lr - u[1]*self.h)*x[2],
                    (self.mu/(x[3]**2*(self.lr+self.lf))*(self.C_Sr*(self.g*self.lf + u[1]*self.h)*self.lr - self.C_Sf*(self.g*self.lr - u[1]*self.h)*self.lf)-1)*x[5] \
                        -self.mu/(x[3]*(self.lr+self.lf))*(self.C_Sr*(self.g*self.lf + u[1]*self.h) + self.C_Sf*(self.g*self.lr-u[1]*self.h))*x[6] \
                        +self.mu/(x[3]*(self.lr+self.lf))*(self.C_Sf*(self.g*self.lr-u[1]*self.h))*x[2]]
        else:
            f = vehicle_dynamics_st(x, u, self.p)
        return f

    def func_STD(x, u, p):
        f = vehicle_dynamics_std(x, u, p)
        return f


def create_optimization_configuration_vehicle(
        scenario: Scenario,
        route_planner: RoutePlanner,
        planning_problem: PlanningProblem,
        settings: Dict, ):
    assert (planning_problem.planning_problem_id in settings), \
        'Cannot find settings for planning problem {}'.format(planning_problem.planning_problem_id)

    vehicle_settings = settings[planning_problem.planning_problem_id]
    configuration = PlanningConfigurationVehicle()

    reference_path, lanelets_leading_to_goal = find_reference_path_and_lanelets_leading_to_goal(
        route_planner, planning_problem, settings)

    configuration.lanelet_network = create_lanelet_network(scenario.lanelet_network, lanelets_leading_to_goal)
    configuration.reference_path = np.array(reference_path)

    if 'reference_point' in vehicle_settings:
        configuration.reference_point = set_reference_point(vehicle_settings['reference_point'])

    configuration.vehicle_id = planning_problem.planning_problem_id
    configuration.min_speed_x = vehicle_settings['min_speed_x']
    configuration.max_speed_x = vehicle_settings['max_speed_x']
    configuration.min_speed_y = vehicle_settings['min_speed_y']
    configuration.max_speed_y = vehicle_settings['max_speed_y']

    configuration.a_max_x = vehicle_settings['a_max_x']
    configuration.a_min_x = vehicle_settings['a_min_x']
    configuration.a_max_y = vehicle_settings['a_max_y']
    configuration.a_min_y = vehicle_settings['a_min_y']
    configuration.a_max = vehicle_settings['a_max']

    configuration.j_min_x = vehicle_settings['j_min_x']
    configuration.j_max_x = vehicle_settings['j_max_x']
    configuration.j_min_y = vehicle_settings['j_min_y']
    configuration.j_max_y = vehicle_settings['j_max_y']

    configuration.length = vehicle_settings['length']
    configuration.width = vehicle_settings['width']
    configuration.wheelbase = vehicle_settings['wheelbase']

    configuration.curvilinear_coordinate_system = create_curvilinear_coordinate_system(
        configuration.reference_path)

    return configuration


def set_reference_point(reference_point: str) -> ReferencePoint:
    if reference_point == 'rear':
        return ReferencePoint.REAR
    elif reference_point == 'center':
        return ReferencePoint.CENTER
    else:
        raise ValueError("<set_reference_point>: reference point of the ego vehicle is unknown: {}".format(
            reference_point))


def find_reference_path_and_lanelets_leading_to_goal(route_planner: RoutePlanner,
                                                     planning_problem: PlanningProblem,
                                                     settings: Dict):
    """
    Find a reference path (and the corresponding lanelets) to the given goal region. The obtained reference path will be
    resampled if needed.
    """
    def interval_extract(list_ids):
        list_ids = sorted(set(list_ids))
        range_start = previous_number = list_ids[0]
        merged_intervals = list()
        for number in list_ids[1:]:
            if number == previous_number + 1:
                previous_number = number
            else:
                merged_intervals.append([range_start, previous_number])
                range_start = previous_number = number
        merged_intervals.append([range_start, previous_number])
        return merged_intervals

    assert (planning_problem.planning_problem_id in settings), \
        'Cannot find settings for planning problem {}'.format(planning_problem.planning_problem_id)

    vehicle_settings = settings[planning_problem.planning_problem_id]

    candidate_holder = route_planner.plan_routes()
    route = candidate_holder.retrieve_first_route()
    reference_path = route.reference_path
    lanelets_leading_to_goal = route.list_ids_lanelets

    # visualize the route
    # visualize_route(route, draw_route_lanelets=True, draw_reference_path=True, size_x=6)

    # extend the reference path:
    first_lanelet = route_planner.lanelet_network.find_lanelet_by_id(lanelets_leading_to_goal[0])
    while first_lanelet.predecessor:
        first_lanelet = route_planner.lanelet_network.find_lanelet_by_id(first_lanelet.predecessor[0])
        reference_path = np.concatenate((first_lanelet.center_vertices, reference_path))
    last_lanelet = route_planner.lanelet_network.find_lanelet_by_id(lanelets_leading_to_goal[-1])
    while last_lanelet.successor:
        last_lanelet = route_planner.lanelet_network.find_lanelet_by_id(last_lanelet.successor[0])
        reference_path = np.concatenate((reference_path, last_lanelet.center_vertices))

    max_curvature = vehicle_settings['max_curvature_reference_path'] + 0.2
    # resampling the reference path
    if vehicle_settings['resampling_reference_path']:
        while max_curvature > vehicle_settings['max_curvature_reference_path']:
            reference_path = np.array(chaikins_corner_cutting(reference_path))
            reference_path = resample_polyline(reference_path, vehicle_settings['resampling_reference_path'])
            abs_curvature = abs(compute_curvature_from_polyline(reference_path))
            max_curvature = max(abs_curvature)
        if 'resampling_reference_path_depending_on_curvature' in vehicle_settings \
                and vehicle_settings['resampling_reference_path_depending_on_curvature']:
            # resample path with higher value where curvature is small
            resampled_path = list()
            intervals = list()
            abs_curvature[0:5] = 0.2
            merged_intervals_ids = interval_extract([i for i, v in enumerate(abs_curvature) if v < 0.01])
            for i in range(0, len(merged_intervals_ids) - 1):
                if i == 0 and merged_intervals_ids[i][0] != 0:
                    intervals.append([0, merged_intervals_ids[i][0]])
                if merged_intervals_ids[i][0] != merged_intervals_ids[i][1]:
                    intervals.append(merged_intervals_ids[i])
                intervals.append([merged_intervals_ids[i][1], merged_intervals_ids[i + 1][0]])

            if len(merged_intervals_ids) == 1:
                if merged_intervals_ids[0][0] != 0:
                    intervals.append([0, merged_intervals_ids[0][0]])
                if merged_intervals_ids[0][0] != merged_intervals_ids[0][1]:
                    intervals.append(merged_intervals_ids[0])

            if intervals and intervals[-1][1] != len(reference_path):
                intervals.append([intervals[-1][1], len(reference_path)])

            resampled_path = None
            for i in intervals:
                if i in merged_intervals_ids:
                    step = 3.
                else:
                    step = vehicle_settings['resampling_reference_path']
                if resampled_path is None:
                    resampled_path = resample_polyline(reference_path[i[0]:i[1]], step)
                else:
                    resampled_path = np.concatenate(
                        (resampled_path, resample_polyline(reference_path[i[0]:i[1]], step)))
        else:
            resampled_path = reference_path

    else:
        resampled_path = reference_path
    return resampled_path, lanelets_leading_to_goal


def create_lanelet_network(lanelet_network: LaneletNetwork, lanelets_leading_to_goal: List[int]) -> LaneletNetwork:
    """
    Create a new lanelet network based on the current structure and given reference lanelets.
    """
    new_lanelet_network = LaneletNetwork()

    for lanelet_id in lanelets_leading_to_goal:
        lanelet_orig = lanelet_network.find_lanelet_by_id(lanelet_id)

        predecessor = list(set(lanelet_orig.predecessor).intersection(lanelets_leading_to_goal))
        successor = list(set(lanelet_orig.successor).intersection(lanelets_leading_to_goal))

        lanelet = Lanelet(lanelet_orig.left_vertices, lanelet_orig.center_vertices, lanelet_orig.right_vertices,
                          lanelet_orig.lanelet_id, predecessor, successor)

        if {lanelet_orig.adj_left}.intersection(lanelets_leading_to_goal):
            lanelet.adj_left = lanelet_orig.adj_left
            lanelet.adj_left_same_direction = lanelet_orig.adj_left_same_direction
        if {lanelet_orig.adj_right}.intersection(lanelets_leading_to_goal):
            lanelet.adj_right = lanelet_orig.adj_right
            lanelet.adj_right_same_direction = lanelet_orig.adj_right_same_direction
        new_lanelet_network.add_lanelet(lanelet)
    return new_lanelet_network


def compute_approximating_circle_radius(ego_length, ego_width) -> Tuple[Union[float, Any], Any]:
    """
    From Julia Kabalar
    Computes parameters of the circle approximation of the ego_vehicle

    :param ego_length: Length of ego vehicle
    :param ego_width: Width of ego vehicle
    :return: radius of circle approximation, circle center point distance
    """
    assert ego_length >= 0 and ego_width >= 0, 'Invalid vehicle dimensions = {}'.format([ego_length, ego_width])

    if np.isclose(ego_length, 0.0) and np.isclose(ego_width, 0.0):
        return 0.0, 0.0

    # Divide rectangle into 3 smaller rectangles
    square_length = ego_length / 3

    # Calculate minimum radius
    diagonal_square = np.sqrt((square_length / 2) ** 2 + (ego_width / 2) ** 2)

    # Round up value
    if diagonal_square > round(diagonal_square, 1):
        approx_radius = round(diagonal_square, 1) + 0.1
    else:
        approx_radius = round(diagonal_square, 1)

    return approx_radius, round(square_length * 2, 1)


def create_curvilinear_coordinate_system(
        reference_path: np.ndarray) -> pycrccosy.CurvilinearCoordinateSystem:
    cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path)
    return cosy
