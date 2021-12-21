from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.init_ks import init_ks
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
import numpy as np

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route
from commonroad.visualization.mp_renderer import MPRenderer


class ReferencePath:
    def __init__(self, scenario, planning_problem):
        self.scenario = scenario
        self. planning_problem = planning_problem
        self.reference_path = self._generate_reference_path()

        self.desired_velocity, self.delta_t = self.get_desired_velocity_and_delta_t()
        self.accumulated_distance_in_reference_path = self._accumulated_distance_in_reference_path()

    def _generate_reference_path(self):
        """
        position_init: 1D array (x_pos, y_pos); the initial position of the planning problem
        reference_path: the output of route planner, which is considered as reference path
        """
        route_planer = RoutePlanner(self.scenario, self.planning_problem, backend=RoutePlanner.Backend.NETWORKX)
        candidate_route = route_planer.plan_routes()
        reference_path = candidate_route.retrieve_best_route_by_orientation().reference_path  # 800*2
        return reference_path

    def plot(self, plot_scenario_and_planning_problem=True, plot_route=True):
        if plot_scenario_and_planning_problem:
            renderer = MPRenderer(figsize=(12, 12))
            self.scenario.draw(renderer)
            self.planning_problem.draw(renderer)
            renderer.render()
            plt.show()
        if plot_route:
            visualize_route(self.reference_path, draw_route_lanelets=True, draw_reference_path=True, size_x=6)

    def _accumulated_distance_in_reference_path(self):
        return np.cumsum(np.linalg.norm(np.diff(self.reference_path, axis=0), axis=1))

    def find_nearest_point_in_reference_path(self, t_past):  # 其实可以直接resampled the reference path, 之后只用这个resampled的
        """
        :param desired_velocity: velocity in the goal region, which is defined in planning problem
        :param t_past: past time from initial position to current step
        :return: index in reference path. The distance from initial position to this index position is nearest distance
        to desired distance.
        """
        desired_distance = self.desired_velocity * t_past
        index = np.abs(self.accumulated_distance_in_reference_path - desired_distance).argmin()
        return index

    def resample_reference_path(self):
        num_path_points = self.reference_path.shape[0]
        cumsum_distance = self._accumulated_distance_in_reference_path
        sim_time = int(cumsum_distance[-1] / self.desired_velocity)
        iter_length = int(sim_time / self.delta_t)
        interval_lenth = int(num_path_points / iter_length)
        resampled_reference_path = self.reference_path[::interval_lenth, :]
        return resampled_reference_path, iter_length, sim_time

    def get_init_value(self):

        if hasattr(self.planning_problem.initial_state, 'acceleration'):
            init_acceleration = self.planning_problem.initial_state.acceleration
        else:
            init_acceleration = 0.

        if hasattr(self.planning_problem.initial_state, 'orientation'):
            init_orientation = self.planning_problem.initial_state.orientation
        else:
            init_orientation = 0

        if hasattr(self.planning_problem.initial_state, "position"):
            init_position = self.planning_problem.initial_state.position
        else:
            init_position = np.array([0, 0])

        return init_position, init_acceleration, init_orientation

    def get_desired_velocity_and_delta_t(self):
        # goal state configuration    这一部分不应该写在这个class里面。要参考qp planner来写,写在main里面
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

