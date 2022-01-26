import matplotlib.pyplot as plt
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner

class ReferencePath:
    def __init__(self, path_scenario, id_scenario):
        self.path_scenario = path_scenario
        self.id_scenario = id_scenario
        self.scenario, self.planning_problem_set = CommonRoadFileReader(self.path_scenario + self.id_scenario).open()
        self.planning_problem = list(self.planning_problem_set.planning_problem_dict.values())[0]
        self.position_init, self.reference_path = self._generate_reference_path()
        # self.desired_velocity = self.planning_problem.goal.state_list[0].velocity.end
        self.accumulated_distance_in_reference_path = self._accumulated_distance_in_reference_path()

    def _generate_reference_path(self):
        """
        position_init: 1D array (x_pos, y_pos); the initial position of the planning problem
        reference_path: the output of route planner, which is considered as reference path
        """
        position_init = self.planning_problem.initial_state.position
        route_planer = RoutePlanner(self.scenario, self.planning_problem, backend=RoutePlanner.Backend.NETWORKX)
        candidate_route = route_planer.plan_routes()
        reference_path = candidate_route.retrieve_best_route_by_orientation().reference_path  # 800*2
        return position_init, reference_path

    def _accumulated_distance_in_reference_path(self):
        return np.cumsum(np.linalg.norm(np.diff(self.reference_path, axis=0), axis=1))

