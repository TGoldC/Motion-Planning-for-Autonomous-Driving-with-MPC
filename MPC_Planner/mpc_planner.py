from commonroad_route_planner.route_planner import RoutePlanner
from optimizer import *
import sys
sys.path.append("..")


class MPC_Planner(object):
    def __init__(self, scenario, planning_problem):
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.origin_reference_path = self._generate_reference_path()
        self.desired_velocity, self.delta_t = self.get_desired_velocity_and_delta_t()

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

    def get_init_values(self):

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

    def _generate_reference_path(self):
        """
        position_init: 1D array (x_pos, y_pos); the initial position of the planning problem
        reference_path: the output of route planner, which is considered as reference path
        """
        route_planer = RoutePlanner(self.scenario, self.planning_problem, backend=RoutePlanner.Backend.NETWORKX)
        candidate_route = route_planer.plan_routes()
        origin_reference_path = candidate_route.retrieve_best_route_by_orientation().reference_path  # 800*2
        return origin_reference_path

    def resample_reference_path(self):
        cumsum_distance = compute_polyline_length(self.origin_reference_path)
        reference_path = np.array(chaikins_corner_cutting(self.origin_reference_path))
        resampled_reference_path = resample_polyline(reference_path, step=self.desired_velocity * self.delta_t)
        iter_length = resampled_reference_path.shape[0]
        return resampled_reference_path, iter_length

    def plan(self, name_solver):
        assert name_solver == "casadi" or "forcespro" or "Casadi" or "Forcespro", 'Cannot find settings for planning problem {}'.format(name_solver)
        # resample the original reference path
        resampled_path_points, iter_length = self.resample_reference_path()

        # get init values and desired velocity
        init_values = MPC_Planner_instance.get_init_values()  # init_position, init_velocity, init_acceleration, init_orientation
        desired_velocity, delta_t = MPC_Planner_instance.get_desired_velocity_and_delta_t()

        # compute orientation from resampled reference path
        orientation = compute_orientation_from_polyline(resampled_path_points)

        if name_solver == "forcespro" or "Forcespro":
            optimizer = ForcesproOptimizer(p=parameters_vehicle2(),
                                           predict_horizon=10,
                                           resampled_path_points=resampled_path_points,
                                           iter_length=iter_length,
                                           init_values=init_values,
                                           delta_t=delta_t,
                                           desired_velocity=desired_velocity,
                                           orientation=orientation)
        else:
            optimizer = CasadiOptimizer(p=parameters_vehicle2(),
                                        predict_horizon=10,
                                        resampled_path_points=resampled_path_points,
                                        iter_length=iter_length,
                                        init_values=init_values,
                                        delta_t=delta_t,
                                        desired_velocity=desired_velocity,
                                        orientation=orientation)
        optimizer.optimize()


if __name__ == '__main__':
    # define the scenario and planning problem
    path_scenario = "/home/xin/PycharmProjects/forcespro_mpc/scenarios/"
    id_scenario = "USA_Lanker-2_18_T-1.xml"  # "ZAM_Tutorial_Urban-3_2.xml"
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario + id_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
    MPC_Planner_instance = MPC_Planner(scenario, planning_problem)
    MPC_Planner_instance.plan("forcespro")
    # MPC_Planner_instance.plan("casadi")



