import math
from typing import Dict, Union
import matplotlib.pyplot as plt
import numpy as np
from decimal import Decimal

# commonroad-io
from commonroad.scenario.trajectory import State, Trajectory as CRTrajectory
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.common.util import make_valid_orientation

# trajectory planning tools
from commonroad_qp_planner.constraints import LonConstraints, LatConstraints, TIConstraints, TVConstraints
from commonroad_qp_planner.configuration import PlanningConfigurationVehicle, ReferencePoint
from commonroad_qp_planner.initialization import compute_initial_state
from commonroad_qp_planner.trajectory import Trajectory, TrajPoint, TrajectoryType
from commonroad_qp_planner.qp_lat_planner import QPLatPlanner, QPLatReference, QPLatState, QPLatPARAMS
from commonroad_qp_planner.qp_long_planner import QPLongPlanner, QPLongReference, QPLongState, QPLongPARAMS

# commonroad-dc
from commonroad_dc.geometry.util import (compute_pathlength_from_polyline, compute_orientation_from_polyline)


class QPPlanner:
    def __init__(self,
                 scenario: Scenario,
                 planning_problem: PlanningProblem,
                 time_horizon: float,
                 vehicle_configuration: PlanningConfigurationVehicle,
                 qp_long_parameters: Union[Dict, None] = None,
                 qp_lat_parameters: Union[Dict, None] = None,
                 slack_usage: bool = False,
                 verbose: bool = True):
        if not hasattr(scenario, 'dt'):
            self.dt = 0.1  # default time step
        else:
            if Decimal(str(time_horizon)) % Decimal(str(scenario.dt)) != Decimal('0.0'):
                raise ValueError('<QPPlanner>: the given time step {} is inapproparite,'
                                 'since time horizon is {}.'.format(scenario.dt, time_horizon))
            self.dt = scenario.dt
        self.t_h = time_horizon

        self.N = round(time_horizon/self.dt)
        if isinstance(planning_problem.initial_state, State):
            self.initial_state = compute_initial_state(planning_problem, vehicle_configuration)
        elif not isinstance(planning_problem.initial_state, TrajPoint):
            raise ValueError('<QPPlanner/__init__>: Initial state must be of type {} or '
                             'of type {}. Got type {}.'.format(type(State),
                                                               type(TrajPoint),
                                                               type(planning_problem.initial_state)))
        self.vehicle_configuration = vehicle_configuration
        self.planning_problem = planning_problem

        if vehicle_configuration.reference_point != ReferencePoint.REAR:
            raise ValueError('<QPPlanner>: Reference point must be rear axis!')

        if planning_problem.goal.state_list:
            if self.initial_state.v > planning_problem.goal.state_list[0].velocity.end:
                self.vehicle_configuration.desired_speed = planning_problem.goal.state_list[0].velocity.end
            else:
                self.vehicle_configuration.desired_speed = self.initial_state.v
        else:
            self.vehicle_configuration.desired_speed = self.initial_state.v

        self.slack_usage = slack_usage
        self.verbose = verbose

        self.time_invariant_constraints = self._set_time_invariant_constraints(vehicle_configuration)

        if qp_long_parameters is not None:
            self.qp_long_params = self._set_qp_longitudinal_parameters(qp_long_parameters)
        else:
            self.qp_long_params = QPLongPARAMS()

        if qp_lat_parameters is not None:
            self.qp_lat_params = self._set_qp_lateral_parameters(qp_lat_parameters)
        else:
            self.qp_lat_params = QPLatPARAMS()

    def plan_trajectories(self,
                          c_tv: TVConstraints,
                          reference: QPLongReference):
        assert c_tv is not None, "Given constraints is not defined"
        print('\t\t Longitudinal optimization')
        traj_lon, status = self.longitudinal_trajectory_planning(c_tv.lon, reference)
        if status is not 'optimal':
            raise ValueError('<QPPlanner/_longitudinal_trajectory_planning>: failed')
        print('\t\t Lateral optimization')
        trajectory, status = self.lateral_trajectory_planning(traj_lon, c_tv.lat)
        # convert trajectory to cartesian space
        if status is not 'optimal':
            raise ValueError('<QPPlanner/_lateral_trajectory_planning>: failed')
        return trajectory

    @classmethod
    def _set_time_invariant_constraints(cls, configuration) -> TIConstraints:
        time_invariant_constraints = TIConstraints()
        time_invariant_constraints.j_x_min = configuration.j_min_x
        time_invariant_constraints.j_x_max = configuration.j_max_x
        time_invariant_constraints.j_y_min = configuration.j_min_y
        time_invariant_constraints.j_y_max = configuration.j_max_y
        time_invariant_constraints.a_x_min = configuration.a_min_x
        time_invariant_constraints.a_x_max = configuration.a_max_x
        time_invariant_constraints.a_y_min = configuration.a_min_y
        time_invariant_constraints.a_y_max = configuration.a_max_y
        time_invariant_constraints.a_max = configuration.a_max
        time_invariant_constraints.v_min = configuration.min_speed_x
        time_invariant_constraints.v_max = configuration.max_speed_x
        return time_invariant_constraints

    @classmethod
    def _set_qp_longitudinal_parameters(cls, qp_long_parameters: Dict) -> QPLongPARAMS:
        qp_long_params = QPLongPARAMS()
        qp_long_params.W_S = qp_long_parameters['W_S']
        qp_long_params.W_V = qp_long_parameters['W_V']
        qp_long_params.W_A = qp_long_parameters['W_A']
        qp_long_params.W_J = qp_long_parameters['W_J']
        qp_long_params.W_U = qp_long_parameters['W_U']
        qp_long_params.W_S_Q = qp_long_parameters['W_S_Q']
        qp_long_params.W_S_L = qp_long_parameters['W_S_L']
        qp_long_params.W_S_C_Q = qp_long_parameters['W_C_Q']
        qp_long_params.W_S_C_L = qp_long_parameters['W_C_L']
        qp_long_params.W_S_A_L = qp_long_parameters['W_S_A_L']
        qp_long_params.W_S_A_U = qp_long_parameters['W_S_A_U']
        qp_long_params.L_ENLARGE = qp_long_parameters['L_ENLARGE']
        return qp_long_params

    @classmethod
    def _set_qp_lateral_parameters(cls, qp_lat_parameters: Dict) -> QPLatPARAMS:
        qp_lat_params = QPLatPARAMS()
        qp_lat_params.W_D = qp_lat_parameters['W_D']
        qp_lat_params.W_THETA = qp_lat_parameters['W_THETA']
        qp_lat_params.W_KAPPA = qp_lat_parameters['W_KAPPA']
        qp_lat_params.W_D_N = qp_lat_parameters['W_D_N']
        qp_lat_params.W_THETA_N = qp_lat_parameters['W_THETA_N']
        qp_lat_params.W_KAPPA_N = qp_lat_parameters['W_KAPPA_N']
        qp_lat_params.W_KAPPA_DOT = qp_lat_parameters['W_KAPPA_DOT']
        qp_lat_params.W_U = qp_lat_parameters['W_U']
        qp_lat_params.W_SLACK_L = qp_lat_parameters['W_SLACK_L']
        qp_lat_params.W_SLACK_Q = qp_lat_parameters['W_SLACK_Q']
        qp_lat_params.KAPPA_DOT_MIN = qp_lat_parameters['KAPPA_DOT_MIN']
        qp_lat_params.KAPPA_DOT_MAX = qp_lat_parameters['KAPPA_DOT_MAX']
        qp_lat_params.KAPPA_DOT_DOT_MIN = qp_lat_parameters['KAPPA_DOT_DOT_MIN']
        qp_lat_params.KAPPA_DOT_DOT_MAX = qp_lat_parameters['KAPPA_DOT_DOT_MAX']
        qp_lat_params.KAPPA_MAX = qp_lat_parameters['KAPPA_MAX']
        return qp_lat_params

    def transform_trajectory_to_cartesian_coordinates(
            self, trajectory: Trajectory):
        cartesian_traj_points = list()
        for state in trajectory.states:
            cart_pos = self.vehicle_configuration.curvilinear_coordinate_system.convert_to_cartesian_coords(
                state.position[0], state.position[1])
            ref_orientation = compute_orientation_from_polyline(self.vehicle_configuration.reference_path)
            ref_pathlength = compute_pathlength_from_polyline(self.vehicle_configuration.reference_path)
            orientation_interpolated = np.interp(state.position[0], ref_pathlength, ref_orientation)
            cartesian_traj_points.append(TrajPoint(
                t=state.t, x=cart_pos[0], y=cart_pos[1], theta=state.orientation + orientation_interpolated, v=state.v,
                a=state.a, kappa=state.kappa, kappa_dot=state.kappa_dot, j=state.j, lane=state.lane))
        traj = Trajectory(cartesian_traj_points, TrajectoryType.CARTESIAN)

        traj._u_lon = trajectory.u_lon
        traj._u_lat = trajectory.u_lat

        return traj

    def longitudinal_trajectory_planning(self, c_long: LonConstraints, reference: QPLongReference):
        ##############################
        # Plan longitudinal trajectory
        ##############################
        lon_planner = QPLongPlanner(self.t_h, self.N, self.dt, slack=False,
                                    qp_long_params=self.qp_long_params)
        lon_planner.verbose = self.verbose  # turn on diagnose solver output
        # initial state s,v,a,j,t
        if isinstance(self.initial_state, TrajPoint):
            x_init = QPLongState(self.initial_state.position[0],
                                 self.initial_state.v,
                                 self.initial_state.a,
                                 self.initial_state.j,
                                 t=0.)
        else:
            raise ValueError('<QPPlanner/_longitudinal_trajectory_planning>: Initial state must be of type {} or '
                             'of type {}. Got type {}.'.format(type(State),
                                                               type(TrajPoint),
                                                               type(self.initial_state)))

        traj, status = lon_planner.plan(x_init, reference, self.time_invariant_constraints, c_long)

        if status == 'optimal':
            if self.verbose:
                print('x_init: {}\n'.format(x_init))
                for i, s in enumerate(traj.cartesian_ptsX()[1:]):
                    print('\t\t\t Longitudinal constraints: time step= {}, s = {}, s_min = {}, s_max = {} \n'.format(
                        i+1, s, c_long.s_hard_min[i], c_long.s_hard_max[i]))
        else:
            if self.verbose:
                print('x_init: {}\n'.format(x_init))
                for i, s in enumerate(c_long.s_hard_min):
                    print('\t\t\t Longitudinal constraints: time step= {}, s_min = {}, s_max = {} \n'.format(
                            i+1, c_long.s_hard_min[i], c_long.s_hard_max[i]))

        if not status == 'optimal' and self.verbose:
            # plot state variables
            print('\t\t\t Lon state: {} \n'.format(x_init))
            plt.figure(figsize=(5, 3))
            plt.plot(0, x_init.s, '*r', label="propagated positions")
            plt.plot(np.array(range(c_long.N)) + 1,
                     x_init.s + x_init.v * self.dt * (np.array(range(c_long.N)) + 1)
                     + 0.5*x_init.a*((np.array(range(self.N)) + 1)*self.dt)**2
                     + (1/6)*x_init.j*((np.array(range(self.N)) + 1)*self.dt)**3
                     , '*g', linewidth=5)
            plt.plot(np.array(range(c_long.N)) + 1,
                     x_init.s + x_init.v * self.dt * (np.array(range(self.N)) + 1)
                     + 0.5*self.vehicle_configuration.a_min_x*((np.array(range(self.N)) + 1)*self.dt)**2,
                     '*r', linewidth=5)
            plt.plot(np.array(range(c_long.N)) + 1, c_long.s_hard_min, "black", label="constraints")
            plt.plot(np.array(range(c_long.N)) + 1, c_long.s_hard_max, "black")
            plt.xlabel("Time steps")
            plt.ylabel("s position[m]")
            plt.legend()
            plt.autoscale()
            plt.show(block=True)
        return traj, status

    def lateral_trajectory_planning(self,
                                    longitudinal_trajectory: Trajectory,
                                    c_lat: LatConstraints):
        #########################
        # Plan lateral trajectory
        #########################
        # create lateral planner with t_h, N, dt, wheelbase, slack usage, lateral parameters
        lat_planner = QPLatPlanner(self.t_h, c_lat.N, self.dt,
                                   self.vehicle_configuration.wheelbase,
                                   self.slack_usage, self.qp_lat_params)
        lat_planner.verbose = self.verbose

        # TODO: remove if condition
        if isinstance(self.initial_state, TrajPoint):
            x_init = QPLatState(d=self.initial_state.position[1],
                                theta=self.initial_state.orientation,
                                kappa=self.initial_state.kappa,
                                kappa_dot=self.initial_state.kappa_dot,
                                t=0.0,
                                s=longitudinal_trajectory.states[0].position[0],
                                v=longitudinal_trajectory.states[0].v,
                                a=longitudinal_trajectory.states[0].a,
                                j=longitudinal_trajectory.states[0].j,
                                u_lon=longitudinal_trajectory.u_lon)
        else:
            raise ValueError('<QPPlanner/_longitudinal_trajectory_planning>: Initial state must be of type {} or '
                             'of type {}. Got type {}.'.format(type(State), type(TrajPoint),
                                                               type(self.initial_state)))

        # create reference
        x_ref = QPLatReference.construct_from_lon_traj_and_reference(
              longitudinal_trajectory, self.vehicle_configuration.reference_path,
              self.time_invariant_constraints)
        self.x_ref = x_ref

        # plan trajectory
        traj_lat, status = lat_planner.plan(x_init, x_ref, self.time_invariant_constraints, c_lat)

        if not status == 'optimal' and self.verbose:
            for s in longitudinal_trajectory.states:
                print('\t\t\t Lon state: {} \n'.format(s))
            print('Lat state: {} \n'.format(x_init))
            for s in x_ref.reference:
                print('\t\t\t Ref state: {} \n'.format(s))

            plt.figure(figsize=(15, 10))
            plt.plot(0, x_init.d, '*-r', linewidth=5)
            plt.plot(0, x_init.d + self.vehicle_configuration.wheelbase / 2.0 *
                     np.sin(self.initial_state.orientation - x_ref.reference[0].theta), '*g')
            plt.plot(0, x_init.d + self.vehicle_configuration.wheelbase *
                     np.sin(self.initial_state.orientation - x_ref.reference[0].theta), '*b')
            plt.plot(np.array(range(1, self.N + 1)), c_lat.d_soft_min[:, 0], '*-r')
            plt.plot(np.array(range(1, self.N + 1)), c_lat.d_soft_max[:, 0], '*-r')
            plt.plot(np.array(range(1, self.N + 1)), c_lat.d_soft_min[:, 1], '-g')
            plt.plot(np.array(range(1, self.N + 1)), c_lat.d_soft_max[:, 1], '-g')
            plt.plot(np.array(range(1, self.N + 1)), c_lat.d_soft_min[:, 2], '-b')
            plt.plot(np.array(range(1, self.N + 1)), c_lat.d_soft_max[:, 2], '-b')
            plt.show(block=True)

        return traj_lat, status

    def convert_to_cartesian_cr_ks_trajectory(self, trajectory: Trajectory, CLCS):
        """
        Converts the current trajectory to a CommonRoad trajectory
        :return: CommonRoad trajectory object
        """

        def convert_curvature_to_angle(kappa: float, wheelbase: float = 2.471) -> float:
            # TODO: remove after fix vehicle model in qp planner
            # tan(steering angle) = wheelbase/radius
            if kappa > 0 or kappa < 0:
                steering_angle = np.arctan(wheelbase / (1 / kappa))
            else:
                steering_angle = 0.0
            return steering_angle

        wheelbase = self.vehicle_configuration.wheelbase

        state_list = []
        # transform every trajectory point
        for p in trajectory.states:
            # convert rear ref point to center point
            x, y = CLCS.convert_to_cartesian_coords(p.position[0], p.position[1])
            if self.vehicle_configuration.reference_point == ReferencePoint.REAR:
                x, y = [x, y] + wheelbase / 2 * np.array([np.cos(p.orientation), np.sin(p.orientation)])

            # PM model
            # state_values = dict()
            # state_values['position'] = np.array(x, y)
            # state_values['velocity'] = float(p.v * np.cos(p.orientation))
            # state_values['velocity_y'] = float(p.v * np.sin(p.orientation))
            # state_values['acceleration'] = float(p.a * np.cos(p.orientation))
            # state_values['acceleration_y'] = float(p.a * np.sin(p.orientation))
            # state_values['orientation'] = float(p.orientation)
            # state_values['time_step'] = int(round(p.t / dT))

            # KS model
            kwarg = {
                "position": np.array([x, y]),
                "steering_angle": convert_curvature_to_angle(p.kappa, wheelbase=wheelbase),
                "velocity": float(p.v),
                "orientation": make_valid_orientation(float(p.orientation)),
                "acceleration": float(p.a),
                "yaw_rate": float(p.kappa * p.v),
                "time_step": int(round(p.t / self.dt)),
            }
            state_list.append(State(**kwarg))

        return CRTrajectory(state_list[0].time_step, state_list)