from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.visualization.mp_renderer import MPRenderer
import commonroad_dc.feasibility.feasibility_checker as feasibility_checker
from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics, VehicleType
from optimizer import *
import imageio
import sys
sys.path.append("..")


class MPCPlanner(object):
    def __init__(self, scenario, planning_problem):
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.origin_reference_path = self._generate_reference_path()
        self.desired_velocity, self.delta_t = self.get_desired_velocity_and_delta_t()
        self.init_values = self.get_init_values()

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

    def plot_and_create_gif(self, iter_length, x):
        dynamic_obstacle_initial_state = State(position=np.array([self.init_values[0][0], self.init_values[0][1]]),
                                               velocity=self.init_values[2],
                                               orientation=self.init_values[3],
                                               time_step=0)
        # generate the states for the obstacle for time steps 1 to 40 by assuming constant velocity
        state_list = []

        for i in range(1, iter_length):
            # compute new position
            new_position = np.array([x[0, i], x[1, i]])
            # create new state
            new_state = State(position=new_position, velocity=x[3, i], orientation=x[4, i], time_step=i)
            # add new state to state_list
            state_list.append(new_state)

        # create the trajectory of the obstacle, starting at time step 1
        dynamic_obstacle_trajectory = Trajectory(1, state_list)

        # create the prediction using the trajectory and the shape of the obstacle
        dynamic_obstacle_shape = Rectangle(width=1.8, length=4.3)
        dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, dynamic_obstacle_shape)

        # generate the dynamic obstacle according to the specification
        dynamic_obstacle_id = self.scenario.generate_object_id()
        dynamic_obstacle_type = ObstacleType.CAR
        dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id,
                                           dynamic_obstacle_type,
                                           dynamic_obstacle_shape,
                                           dynamic_obstacle_initial_state,
                                           dynamic_obstacle_prediction)

        # add dynamic obstacle to the scenario
        self.scenario.add_objects(dynamic_obstacle)

        # plot the scenario for each time step
        for i in range(0, iter_length):
            plt.figure(figsize=(25, 10))
            rnd = MPRenderer()
            self.scenario.draw(rnd, draw_params={'time_begin': i})
            self.planning_problem.draw(rnd)
            rnd.render()
            plt.savefig("../figures/temp{}.png".format(i))
            # plt.show()
            plt.clf()

        figures_list = []
        for i in range(0, iter_length):
            figures_list.append("../figures/temp{}.png".format(i))
        with imageio.get_writer('../mygif.gif', mode='I') as writer:
            for filename in figures_list:
                image = imageio.imread(filename)
                writer.append_data(image)

    def check_feasibility(self, ego_vehicle_trajectory):
        # set time step as scenario time step
        dt = self.scenario.dt

        # choose vehicle model (here kinematic single-track model)
        vehicle_dynamics = VehicleDynamics.KS(VehicleType.BMW_320i)

        # check feasibility of planned trajectory for the given vehicle model
        feasible, reconstructed_inputs = feasibility_checker.trajectory_feasibility(ego_vehicle_trajectory, vehicle_dynamics, dt)
        print('The planned trajectory is feasible: %s' % feasible)

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
        final_states = optimizer.optimize()
        self.plot_and_create_gif(iter_length, final_states)


if __name__ == '__main__':
    # define the scenario and planning problem
    path_scenario = "../scenarios/"  # relative dir path, scenarios and mpc_planner.py are in the same directory
    id_scenario = "USA_Lanker-2_18_T-1.xml"  # "ZAM_Tutorial_Urban-3_2.xml"
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario + id_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
    MPC_Planner_instance = MPCPlanner(scenario, planning_problem)
    MPC_Planner_instance.plan("forcespro")
    # MPC_Planner_instance.plan("casadi")
    # TODO introduce Collision Avoidance feature




