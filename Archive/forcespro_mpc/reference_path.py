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
        self.planning_problem = planning_problem
        self.reference_path = self._generate_reference_path()
        self.desired_velocity, self.delta_t = self.get_desired_velocity_and_delta_t()

    def _generate_reference_path(self):
        """
        position_init: 1D array (x_pos, y_pos); the initial position of the planning problem
        reference_path: the output of route planner, which is considered as reference path
        """
        route_planer = RoutePlanner(self.scenario, self.planning_problem, backend=RoutePlanner.Backend.NETWORKX)
        candidate_route = route_planer.plan_routes()
        reference_path = candidate_route.retrieve_best_route_by_orientation().reference_path  # 800*2
        return reference_path

    def resample_reference_path(self):
        num_path_points = self.reference_path.shape[0]
        cumsum_distance = self._accumulated_distance_in_reference_path()
        sim_time = int(cumsum_distance[-1] / self.desired_velocity)
        iter_length = int(sim_time / self.delta_t)
        interval_length = int(num_path_points / iter_length)
        resampled_reference_path = self.reference_path[::interval_length, :]  # N*2
        return resampled_reference_path, iter_length, sim_time

    @staticmethod
    def find_closest_point(path_points, current_point):
        """Find the index of the closest point in points from the current car position
        points = array of points on path
        ref_point = current car position
        """
        # num_points = path_points.shape[1]  # 这里的points是 2*N 的
        diff = np.transpose(path_points) - current_point
        diff = np.transpose(diff)
        squared_diff = np.power(diff, 2)
        squared_dist = squared_diff[0, :] + squared_diff[1, :]
        return np.argmin(squared_dist)

    def extract_next_path_points(self, path_points, current_pos, N):  # pos是car当前位置，先找到最近的index，然后返回其之后的N个点
        """Extract the next N points on the path for the next N stages starting from
        the current car position pos
        """
        idx = self.find_closest_point(path_points, current_pos)
        # num_points = path_points.shape[1]  # 有多少个点
        # num_ellipses = np.ceil((idx+N+1)/num_points)  # np.ceil() 向上取整，单一椭圆的话，就是1
        # path_points = np.tile(path_points, (1, int(num_ellipses)))  # np.tile复制 1*int(num_ellipses) 多份，正常是1
        return path_points[:, idx+1:idx+N+1]  # return the next N points 2*N

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

        if hasattr(self.planning_problem, "velocity"):
            init_velocity = self.planning_problem.initial_state.velocity
        else:
            init_velocity = 0

        return init_position, init_velocity, init_acceleration, init_orientation

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

    def find_nearest_point_in_reference_path(self, t_past):
        """
        :param desired_velocity: velocity in the goal region, which is defined in planning problem
        :param t_past: past time from initial position to current step
        :return: index in reference path. The distance from initial position to this index position is nearest distance
        to desired distance.
        """
        desired_distance = self.desired_velocity * t_past
        index = np.abs(self._accumulated_distance_in_reference_path() - desired_distance).argmin()
        return index

    def _accumulated_distance_in_reference_path(self):
        return np.cumsum(np.linalg.norm(np.diff(self.reference_path, axis=0), axis=1))

    def plot(self, plot_scenario_and_planning_problem=True, plot_route=True):
        if plot_scenario_and_planning_problem:
            renderer = MPRenderer(figsize=(12, 12))
            self.scenario.draw(renderer)
            self.planning_problem.draw(renderer)
            renderer.render()
            plt.show()
        if plot_route:
            visualize_route(self.reference_path, draw_route_lanelets=True, draw_reference_path=True, size_x=6)
