# This ist test file for route planning

import os
import matplotlib.pyplot as plt
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route
from commonroad.visualization.mp_renderer import MPRenderer

path_scenario = os.path.join(os.path.dirname(os.path.realpath(__file__)), "commonroad-qp-planner/scenarios/")
id_scenario = 'USA_Peach-2_1_T-1.xml'
scenario, planning_problem_set = CommonRoadFileReader(path_scenario+id_scenario).open()
length = len(list(planning_problem_set.planning_problem_dict.values()))
# print(length)  # 1
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
# print(planning_problem.initial_state.position)
# print("goal", planning_problem.goal.state_list)
desired_velocity = planning_problem.goal.state_list[0].velocity.end
# print("desired_velocity", desired_velocity)

"""
renderer = MPRenderer(figsize=(12, 12))
scenario.draw(renderer)
planning_problem.draw(renderer)

renderer.render()
plt.show()
"""

route_planer = RoutePlanner(scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX)
candidate_route = route_planer.plan_routes()

route = candidate_route.retrieve_best_route_by_orientation()
reference_path = route.reference_path
# print(reference_path)
# visualize_route(route, draw_route_lanelets=True, draw_reference_path=True, size_x=6)

A = np.arange(2 , 14).reshape((2, 6))
print('A:' , A[:, 0].shape)
# B = np.diff(A, axis=0)
# C = np.linalg.norm(B, axis=1)
# D = np.cumsum(C)
# print(D)
# desired_distance = 5
# print(np.abs(D - desired_distance).argmin())
B = np.squeeze(A.reshape(1, -1))
print("B", B)
print(B.reshape((2, 6)))

mylist = [1, 2, 3]
print(mylist*2)