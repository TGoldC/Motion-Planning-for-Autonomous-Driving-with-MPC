
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
    def __init__(self, path_scenario, id_scenario):
        self.path_scenario = path_scenario
        self.id_scenario = id_scenario

        self.planning_problem = list(self.planning_problem_set.planning_problem_dict.values())[0]
        self.position_init, self.reference_path = self._generate_reference_path()

        self.accumulated_distance_in_reference_path = self._accumulated_distance_in_reference_path()
        self.init_orientation = self.planning_problem.initial_state.orientation

        # goal state configuration    这一部分不应该写在这个class里面。要参考qp planner来写,写在main里面
        self.goal = self.planning_problem.goal
        if hasattr(self.planning_problem.goal.state_list[0], 'velocity'):
            if self.planning_problem.goal.state_list[0].velocity.start != 0:
                self.desired_velocity = (self.planning_problem.goal.state_list[0].velocity.start
                                         + self.planning_problem.goal.state_list[0].velocity.end) / 2
            else:
                self.desired_velocity = (self.planning_problem.goal.state_list[0].velocity.start
                                         + self.planning_problem.goal.state_list[0].velocity.end) / 2
        else:
            self.desired_velocity = self.planning_problem.initial_state.velocity

    def _generate_reference_path(self):
        """
        position_init: 1D array (x_pos, y_pos); the initial position of the planning problem
        reference_path: the output of route planner, which is considered as reference path
        """
        position_init = self.planning_problem.initial_state.position
        #### self.planning_problem.initial_state.orientation   initial heading angle
        route_planer = RoutePlanner(self.scenario, self.planning_problem, backend=RoutePlanner.Backend.NETWORKX)
        candidate_route = route_planer.plan_routes()
        reference_path = candidate_route.retrieve_best_route_by_orientation().reference_path  # 800*2
        return position_init, reference_path

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









