
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.init_ks import init_ks
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
import numpy as np

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route
from commonroad.visualization.mp_renderer import MPRenderer


# define vehicle model
def ks_dynamics(x, u, p):
    p = parameters_vehicle2()
    f = vehicle_dynamics_ks(x, u, p)
    return f


class ReferencePath:
    def __init__(self, path_scenario, id_scenario):
        self.path_scenario = path_scenario
        self.id_scenario = id_scenario
        self.scenario, self.planning_problem_set = CommonRoadFileReader(self.path_scenario + self.id_scenario).open()
        self.planning_problem = list(self.planning_problem_set.planning_problem_dict.values())[0]

    def reference_path(self):
        """
        position_init: 1D array (x_pos, y_pos); the initial position of the planning problem
        reference_path: the output of route planner, which is considered as reference path
        """
        position_init = self.planning_problem.initial_state.position
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
            _, reference_path = self.reference_path()
            visualize_route(reference_path, draw_route_lanelets=True, draw_reference_path=True, size_x=6)


























"""
# initialization
delta0 = 0
vel0 = 15
Psi0 = 0
dotPsi0 = 0
beta0 = 0
sy0 = 0
initialState = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]  # initial state for simulation
x0_KS = init_ks(initialState)  # initial state for kinematic single-track model
"""








