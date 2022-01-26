from typing import Dict, Tuple, List, Union, Any
import numpy as np

from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route

# commonroad-io
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario, State
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet

# commonroad-curvilinear-coordinatesystem
import commonroad_dc.pycrccosy as pycrccosy
from commonroad_dc.geometry.util import (chaikins_corner_cutting, compute_curvature_from_polyline, resample_polyline,
                                         compute_pathlength_from_polyline, compute_orientation_from_polyline)

from commonroad_qp_planner.trajectory import TrajPoint
from commonroad_qp_planner.configuration import PlanningConfigurationVehicle, ReferencePoint
from commonroad_qp_planner.qp_lat_planner import QPLatState
from commonroad_qp_planner.qp_long_planner import QPLongState


def set_up(settings: Dict,
           scenario: Scenario,
           planning_problem: PlanningProblem):
    # instantiate a route planner with the scenario and the planning problem
    route_planner = RoutePlanner(scenario,
                                 planning_problem,
                                 backend=RoutePlanner.Backend.NETWORKX_REVERSED)
    vehicle_configuration = create_optimization_configuration_vehicle(
        scenario,
        route_planner,
        planning_problem,
        settings['vehicle_settings'])
    return vehicle_configuration


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


def convert_pos_curvilinear(
        state: State,
        configuration: PlanningConfigurationVehicle) -> [float, float]:
    """
    Converts the position of the state to the CLCS.
    """
    if configuration.reference_point == ReferencePoint.REAR:
        pos = configuration.curvilinear_coordinate_system.convert_to_curvilinear_coords(
            state.position[0] - configuration.wheelbase / 2 * np.cos(state.orientation),
            state.position[1] - configuration.wheelbase / 2 * np.sin(state.orientation))
    elif configuration.reference_point == ReferencePoint.CENTER:
        pos = configuration.curvilinear_coordinate_system.convert_to_curvilinear_coords(
            state.position[0], state.position[1])
    else:
        raise ValueError("<compute_initial_state>: unknown reference point: {}".format(
            configuration.reference_point))
    return pos


def compute_initial_state(
        planning_problem: PlanningProblem,
        configuration: PlanningConfigurationVehicle) -> TrajPoint:
    """
    This function computes the initial state of the ego vehicle for the qp-planner given a
    planning problem according to CommonRoad. It is assumed that d*kappa_ref << 1 holds, where d is the distance of
    the ego vehicle to the reference path and kappa_ref is the curvature of the reference path,
    for the transformation of the ego vehicle's velocity to the curvilinear coordinate system.

    :param planning_problem: CommonRoad planning problem
    :param configuration: parameters of the ego vehicle
    :return: initial state of the ego vehicle in curvilinear coordinates (TrajPoint)
    """
    pos = convert_pos_curvilinear(planning_problem.initial_state, configuration)
    reference_path = configuration.reference_path
    ref_orientation = compute_orientation_from_polyline(reference_path)
    ref_pathlength = compute_pathlength_from_polyline(reference_path)
    ref_curvature = compute_curvature_from_polyline(reference_path)
    ref_curvature_dot = np.gradient(ref_curvature, ref_pathlength)
    orientation_interpolated = np.interp(pos[0], ref_pathlength, ref_orientation)

    v_x = planning_problem.initial_state.velocity * np.cos(
        planning_problem.initial_state.orientation - orientation_interpolated)

    if hasattr(planning_problem.initial_state, 'acceleration'):
        a = planning_problem.initial_state.acceleration
    else:
        a = 0.

    # compute orientation in curvilinear coordinate frame
    theta_cl = planning_problem.initial_state.orientation - orientation_interpolated
    # compute curvature
    kr = np.interp(pos[0], ref_pathlength, ref_curvature)
    kr_d = np.interp(pos[0], ref_pathlength, ref_curvature_dot)

    return TrajPoint(t=0., x=pos[0], v=v_x, a=a, j=0., y=pos[1], theta=theta_cl, kappa=kr, kappa_dot=kr_d)


def create_curvilinear_coordinate_system(
        reference_path: np.ndarray) -> pycrccosy.CurvilinearCoordinateSystem:
    cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path)
    return cosy
