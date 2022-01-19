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
from commonroad_dc.geometry.util import (chaikins_corner_cutting, compute_curvature_from_polyline, resample_polyline, compute_orientation_from_polyline)


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
        self.vehicle_model = "parameters_vehicle2"
        self.desired_speed = 0.0
        self.wheelbase = 3.0
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
    def vehicle_model(self) -> str:
        """Vehicle model, e.g. parameters_vehicle2"""
        return self._vehicle_model

    @vehicle_model.setter
    def vehicle_model(self, vehicle_model: str):
        assert (type(vehicle_model) is str), '<PlanConfiguration/vehicle_id> Expected type int; ' \
                                                 'Got type %s instead.' % (type(vehicle_model))
        self._vehicle_model = vehicle_model

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
        self.p = p
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

    @classmethod
    def KS_casadi(cls, x, u):
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


class Configuration(object):
    def __init__(self,
                 settings: Dict,
                 scenario: Scenario,
                 planning_problem: PlanningProblem):
        self.settings = settings
        self.scenario = scenario
        self.planning_problem = planning_problem
        # instantiate a route planner with the scenario and the planning problem
        self.route_planner = RoutePlanner(scenario,
                                          planning_problem,
                                          backend=RoutePlanner.Backend.NETWORKX_REVERSED)
        self.configuration = self.create_optimization_configuration_vehicle()
        # contain configuration of vehicle and problem_configuration(delta_t, resampled_reference_path)

    def create_optimization_configuration_vehicle(self):
        assert (self.planning_problem.planning_problem_id in self.settings["vehicle_settings"]), \
            'Cannot find settings for planning problem {}'.format(self.planning_problem.planning_problem_id)

        vehicle_settings = self.settings["vehicle_settings"][self.planning_problem.planning_problem_id]

        # add some attributes to configuration
        configuration = PlanningConfigurationVehicle()

        desired_velocity, delta_t = self.get_desired_velocity_and_delta_t()
        configuration.desired_velocity = desired_velocity
        configuration.delta_t = delta_t
        # add reference path
        reference_path, lanelets_leading_to_goal = self.find_reference_path_and_lanelets_leading_to_goal(desired_velocity, delta_t)
        configuration.lanelet_network = self.create_lanelet_network(self.scenario.lanelet_network, lanelets_leading_to_goal)
        configuration.reference_path = np.array(reference_path)

        configuration.iter_length = reference_path.shape[0]

        # compute orientation from resampled reference path
        orientation = compute_orientation_from_polyline(reference_path)
        configuration.orientation = orientation

        # add predict horizon
        configuration.predict_horizon = self.settings["general_planning_settings"]["predict_horizon"]

        if 'reference_point' in vehicle_settings:
            configuration.reference_point = self.set_reference_point(vehicle_settings['reference_point'])

        configuration.vehicle_id = self.planning_problem.planning_problem_id
        configuration.p = eval(vehicle_settings["vehicle_model"])()
        configuration.wheelbase = vehicle_settings['wheelbase']

        configuration.curvilinear_coordinate_system = self.create_curvilinear_coordinate_system(
            configuration.reference_path)

        # configuration.vehicle_dynamics = VehicleDynamics(configuration.p)

        return configuration

    @staticmethod
    def set_reference_point(reference_point: str) -> ReferencePoint:
        if reference_point == 'rear':
            return ReferencePoint.REAR
        elif reference_point == 'center':
            return ReferencePoint.CENTER
        else:
            raise ValueError("<set_reference_point>: reference point of the ego vehicle is unknown: {}".format(
                reference_point))

    def find_reference_path_and_lanelets_leading_to_goal(self, desired_velocity, delta_t):
        """
        Find a reference path (and the corresponding lanelets) to the given goal region. The obtained reference path will be
        resampled if needed.
        """
        assert (self.planning_problem.planning_problem_id in self.settings["vehicle_settings"]), \
            'Cannot find settings for planning problem {}'.format(self.planning_problem.planning_problem_id)

        vehicle_settings = self.settings["vehicle_settings"][self.planning_problem.planning_problem_id]

        candidate_holder = self.route_planner.plan_routes()
        route = candidate_holder.retrieve_best_route_by_orientation()
        origin_reference_path = route.reference_path  # origin reference path from route planner
        lanelets_leading_to_goal = route.list_ids_lanelets

        # resampling the reference path
        if vehicle_settings['resampling_reference_path']:
            reference_path = np.array(chaikins_corner_cutting(origin_reference_path))
            resampled_path = resample_polyline(reference_path, step=desired_velocity * delta_t)
        else:
            resampled_path = origin_reference_path
        return resampled_path, lanelets_leading_to_goal

    def get_desired_velocity_and_delta_t(self):
        # goal state configuration
        if hasattr(self.planning_problem.goal.state_list[0], 'velocity'):
            if self.planning_problem.goal.state_list[0].velocity.start != 0:
                desired_velocity = (self.planning_problem.goal.state_list[0].velocity.start
                                    + self.planning_problem.goal.state_list[0].velocity.end) / 2
            else:
                desired_velocity = (self.planning_problem.goal.state_list[0].velocity.start
                                    + self.planning_problem.goal.state_list[0].velocity.end) / 2
        else:
            desired_velocity = self.planning_problem.initial_state.velocity

        if not hasattr(self.scenario, 'dt'):
            delta_t = 0.1  # default time step
        else:
            delta_t = self.scenario.dt

        return desired_velocity, delta_t

    @staticmethod
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

    @staticmethod
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

    @staticmethod
    def create_curvilinear_coordinate_system(reference_path: np.ndarray) -> pycrccosy.CurvilinearCoordinateSystem:
        cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path)
        return cosy
