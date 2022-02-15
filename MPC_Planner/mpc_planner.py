import numpy as np
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle, StaticObstacle
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.param_server import ParamServer
from MPC_Planner.optimizer import *
from MPC_Planner.configuration import *
import imageio
import matplotlib.pyplot as plt
import os
import sys
sys.path.append("..")


def find_closest_point(path_points, current_point):
    """Find the index of the closest point in points from the current car position
    path_points: (n, 2)
    current_points: (2,)
    """
    # num_points = path_points.shape[1]  # 这里的points是 2*N 的
    diff = np.transpose(path_points) - current_point.reshape(2, 1)
    diff = np.transpose(diff)
    squared_diff = np.power(diff, 2)
    squared_dist = squared_diff[:, 0] + squared_diff[:, 1]
    return np.argmin(squared_dist)


class MPCPlanner(object):
    def __init__(self, scenario, planning_problem, configuration, predict_horizon):
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.configuration = configuration
        self.init_values = self.get_init_values()
<<<<<<< HEAD
        self.resampled_reference_path, self.iter_length = self.resample_reference_path()

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
=======
        # self.static_obstacles = self.static_obstacle()  # : StaticObstacle
        self.predict_horizon = predict_horizon
>>>>>>> develop

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

        if hasattr(self.planning_problem.initial_state, "velocity"):
            init_velocity = self.planning_problem.initial_state.velocity
        else:
            init_velocity = 0

        return init_position, init_velocity, init_acceleration, init_orientation

<<<<<<< HEAD
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

    def plot(self, x):
        ego_vehicle_initial_state = State(position=np.array([self.init_values[0][0], self.init_values[0][1]]),
                                               velocity=self.init_values[2],
                                               orientation=self.init_values[3],
                                               time_step=0)
        # generate the states for the obstacle for time steps 1 to 40 by assuming constant velocity
        state_list = []

        for i in range(1, self.iter_length):
=======
    def static_obstacle(self,):
        static_obstacle_id = self.scenario.generate_object_id()
        static_obstacle_type = ObstacleType.PARKED_VEHICLE
        static_obstacle_shape = Rectangle(width=self.configuration.static_obstacle["width"], length=self.configuration.static_obstacle["length"])
        static_obstacle_initial_state = State(position=np.array([self.configuration.static_obstacle["position_x"], self.configuration.static_obstacle["position_y"]]),
                                              orientation=self.configuration.static_obstacle["orientation"], time_step=0)
        
        # feed in the required components to construct a static obstacle
        static_obstacles = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape, static_obstacle_initial_state)
        
        return static_obstacles

    def plot_and_create_gif(self, x, u, solve_time):
        ego_vehicle_initial_state = State(position=np.array([self.init_values[0][0], self.init_values[0][1]]),
                                          velocity=self.init_values[2],
                                          orientation=self.init_values[3],
                                          time_step=0)
        # generate the states for the obstacle for time steps 1 to 40 by assuming constant velocity
        state_list = []

        for i in range(1, self.configuration.iter_length):
>>>>>>> develop
            # compute new position
            new_position = np.array([x[i, 0], x[i, 1]])
            # create new state
            new_state = State(position=new_position, velocity=x[i, 3], orientation=x[i, 4], time_step=i)
            # add new state to state_list
            state_list.append(new_state)

        # create the trajectory of the obstacle, starting at time step 1
        ego_vehicle_trajectory = Trajectory(1, state_list)

        # create the prediction using the trajectory and the shape of the obstacle
        ego_vehicle_shape = Rectangle(width=1.8, length=4.3)
        ego_vehicle_prediction = TrajectoryPrediction(ego_vehicle_trajectory, ego_vehicle_shape)

        # generate the dynamic obstacle according to the specification
        ego_vehicle_id = self.scenario.generate_object_id()
        ego_vehicle_type = ObstacleType.CAR
        ego_vehicle = DynamicObstacle(ego_vehicle_id,
<<<<<<< HEAD
                                           ego_vehicle_type,
                                           ego_vehicle_shape,
                                           ego_vehicle_initial_state,
                                           ego_vehicle_prediction)

        # plot the scenario for each time step
        for i in range(0, 19):
=======
                                      ego_vehicle_type,
                                      ego_vehicle_shape,
                                      ego_vehicle_initial_state,
                                      ego_vehicle_prediction)
        # add static obstacle
        # self.scenario.add_objects(self.static_obstacle())

        # create a folder to save images at each time step
        path_figures = "../test/figures_{}_{}_{}/".format(self.configuration.framework_name, str(self.scenario.scenario_id), self.configuration.use_case)
        folder_exist = os.path.exists(path_figures)
        if not folder_exist:
            os.makedirs(path_figures)

        # plot the scenario for each time step
        for i in range(0, self.configuration.iter_length):
>>>>>>> develop
            plt.figure(figsize=(25, 10))
            if "ZAM_Over" in str(self.scenario.scenario_id):
                rnd = MPRenderer(plot_limits=[25, 125, -10, 20])
            elif "USA_Lanker" in str(self.scenario.scenario_id):
                rnd = MPRenderer(plot_limits=[-20, 60, -25, 100])
            else:
                raise ValueError("Only ZAM_Over-1_1 and USA_Lanker-2_18_T-1 can be processed!")

            self.scenario.draw(rnd, draw_params={'time_begin': i})
<<<<<<< HEAD
            ego_vehicle.draw(rnd, draw_params={'time_begin': i, 'dynamic_obstacle': {
                        'vehicle_shape': {'occupancy': {'shape': {'rectangle': {
                            'facecolor': 'r'}}}}}})
            self.planning_problem.draw(rnd)
            rnd.render()
            rnd.ax.plot(self.resampled_reference_path[i, 0], self.resampled_reference_path[i, 1], color='r', marker='.', markersize=1, zorder=19, linewidth=0.8,
                        label='reference path')
            plt.savefig("figures/test_temp{}.png".format(i))
            plt.clf()

        figures_list = []
        for i in range(0, 19):
            figures_list.append("figures/test_temp{}.png".format(i))
        with imageio.get_writer('test_mygif000.gif', mode='I') as writer:
=======
            ego_vehicle.draw(rnd,
                             draw_params=ParamServer(
                                 {"time_begin": i, "time_end": i+self.predict_horizon, "trajectory": {
                                     "draw_trajectory": True},
                                  "occupancy": {
                                      "draw_occupancies": 1,
                                      "shape": {"rectangle": {
                                          "facecolor": 'r',
                                          "edgecolor": 'r',
                                          "zorder": 500}
                                      }},
                                  "dynamic_obstacle":
                                      {"vehicle_shape": {
                                          "occupancy": {
                                              "shape": {"rectangle": {
                                                  "facecolor": 'r',
                                                  "edgecolor": 'r',
                                                  "zorder": 500}
                                              }}}}}))

            self.planning_problem.draw(rnd, draw_params={"initial_state": {"state": {"draw_arrow": False}}})
            rnd.render()
            # add reference path to animation
            rnd.ax.plot(self.configuration.reference_path[:, 0], self.configuration.reference_path[:, 1], color='r', marker='.', markersize=1, zorder=19, linewidth=1.0,
                        label='reference path')
            plt.savefig(path_figures + "temp{}.png".format(i))

            # plt.show()
            plt.clf()

        figures_list = []
        for i in range(0, self.configuration.iter_length):
            figures_list.append(path_figures + "temp{}.png".format(i))
        with imageio.get_writer("../test/gif_{}_{}_{}.gif".format(self.configuration.framework_name, str(self.scenario.scenario_id), self.configuration.use_case), mode='I') as writer:
>>>>>>> develop
            for filename in figures_list:
                image = imageio.imread(filename)
                writer.append_data(image)

<<<<<<< HEAD

    def plan(self, name_solver):
        assert name_solver == "casadi" or "forcespro" or "Casadi" or "Forcespro", 'Cannot find settings for planning problem {}'.format(name_solver)
        # resample the original reference path
        resampled_path_points, iter_length = self.resample_reference_path()

        # get init values and desired velocity
        init_values = MPC_Planner_instance.get_init_values()  # init_position, init_velocity, init_acceleration, init_orientation
        desired_velocity, delta_t = MPC_Planner_instance.get_desired_velocity_and_delta_t()

        # compute orientation from resampled reference path
        orientation = compute_orientation_from_polyline(resampled_path_points)

        if name_solver == "casadi" or "Casadi":
            optimizer = CasadiOptimizer(p=parameters_vehicle2(),
                                        predict_horizon=10,
                                        resampled_path_points=resampled_path_points,
                                        iter_length=iter_length,
                                        init_values=init_values,
                                        delta_t=delta_t,
                                        desired_velocity=desired_velocity,
                                        orientation=orientation)
        else:
            optimizer = ForcesproOptimizer(p=parameters_vehicle2(),
                                           predict_horizon=10,
                                           resampled_path_points=resampled_path_points,
                                           iter_length=iter_length,
                                           init_values=init_values,
                                           delta_t=delta_t,
                                           desired_velocity=desired_velocity,
                                           orientation=orientation)
        final_states = optimizer.optimize()
        return final_states
        # self.plot(iter_length, final_states)


if __name__ == '__main__':
    # define the scenario and planning problem
    path_scenario = "scenarios/"  # relative dir path, scenarios and mpc_planner.py are in the same directory
    id_scenario = "USA_Lanker-2_18_T-1.xml"  # "ZAM_Tutorial_Urban-3_2.xml"
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario + id_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
    MPC_Planner_instance = MPC_Planner(scenario, planning_problem)
    #MPC_Planner_instance.plan("forcespro")
    #MPC_Planner_instance.plan("casadi")
    states = MPC_Planner_instance.plan("casadi")
    MPC_Planner_instance.plot(states)
=======
        # plot 2D figures for analyse
        path_2D_figures = "../test/2D_plots_{}_{}_{}/".format(self.configuration.framework_name, str(self.scenario.scenario_id), self.configuration.use_case)
        folder_exist = os.path.exists(path_2D_figures)
        if not folder_exist:
            os.makedirs(path_2D_figures)

        self.plot_deviation_euclidean_dis(x, path_2D_figures)
        self.plot_control_inputs(u, path_2D_figures)
        self.plot_solve_time(solve_time, path_2D_figures)
        self.plot_path(x, u, path_2D_figures)
        return ego_vehicle_trajectory, ego_vehicle

    def plot_deviation_euclidean_dis(self, x, save_path):
        plt.figure()
        # deviation = np.hstack((self.configuration.reference_path[:, 0], self.configuration.reference_path[-1, 0])) - x[:, 0]
        nearest_points = np.zeros((x.shape[0], 2))
        for i in range(x.shape[0]):
            nearest_points[i] = self.configuration.origin_reference_path[find_closest_point(self.configuration.origin_reference_path, x[i, 0:2])]

        deviation_x = nearest_points[:, 0] - x[:, 0]
        deviation_y = nearest_points[:, 1] - x[:, 1]
        deviation = np.sqrt(deviation_x ** 2 + deviation_y ** 2)

        plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, deviation)
        plt.title('deviation with reference path ')
        plt.xlabel('time [s]')
        plt.ylabel('deviation in euclidean distance [m]')

        plt.savefig(save_path + "2D_plot_{}_{}_{}_deviation".format(self.configuration.framework_name, self.scenario.scenario_id, self.configuration.use_case))
        plt.show()

    def plot_control_inputs(self, u, save_path):
        plt.figure()
        # print("control inputs: ", repr(u))
        # control_inputs_unnoised = np.array()
        plt.subplot(2, 1, 1)
        plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t,  np.rad2deg(u[:, 0]), color="b", label="noised")
        # plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, np.rad2deg(control_inputs_unnoised[:, 0]), color="g", label="unnoised")
        plt.legend(loc='upper right')
        plt.legend(loc='upper right')
        plt.title('steering velocity')
        plt.xlabel('time [s]')
        plt.ylabel('delta_v [deg/s]')
        plt.subplots_adjust(hspace=0.8)

        plt.subplot(2, 1, 2)
        plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, u[:, 1], color="b", label="noised")
        # plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, control_inputs_unnoised[:, 1], color="g", label="unnoised")
        plt.legend()
        plt.title('longitudinal acceleration')
        plt.xlabel('time [s]')
        plt.ylabel('long. acc. [m/s2]')
>>>>>>> develop

        plt.savefig(save_path + "2D_plot_{}_{}_{}_control_inputs".format(self.configuration.framework_name, self.scenario.scenario_id, self.configuration.use_case))
        plt.show()

    def plot_solve_time(self, solve_time, save_path):
        plt.figure()
        # print("solve_time", repr(solve_time))
        plt.plot(np.arange(self.configuration.iter_length), solve_time*1000,  color='b', label='forcespro computation time')
        # plt.plot(np.arange(self.configuration.iter_length), solve_time_casadi * 1000, color='g', label='casadi computation time')
        plt.legend()
        plt.title('Computation time over iteration')
        plt.xlabel('iteration')
        plt.ylabel('Computation time [ms]')

        plt.savefig(save_path + "2D_plot_{}_{}_{}_solve_time".format(self.configuration.framework_name, self.scenario.scenario_id, self.configuration.use_case))
        plt.show()

    def plot_path(self, x, u, save_path):
        # 2D plot
        plt.figure()

        plt.subplot(2, 1, 1)
        plt.title('Performance in x-direction')
        plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, self.configuration.reference_path[:, 0], color='red', linestyle="--", label='reference path')
        plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, x[:, 0], color='green', label='MPC planned path')
        # plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, path_without_noise_x[:-1], color='blue', label='MPC planned path without noise')
        plt.legend()
        plt.xlabel('time [s]')
        plt.ylabel('x-position [m]')
        plt.subplots_adjust(hspace=0.8)

        plt.subplot(2, 1, 2)
        plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, self.configuration.reference_path[:, 1], color='red', linestyle="--", label='reference path')
        plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, x[:, 1], color='green', label='MPC planned path')
        # plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, path_without_noise_y[:-1], color='blue', label='MPC planned path without noise')
        plt.title('Performance in y-direction')
        plt.legend()
        plt.xlabel('time [s]')
        plt.ylabel('y-position [m]')

        plt.subplots_adjust(wspace=0.25, hspace=0.5)
        plt.savefig(save_path + "2D_plot_{}_{}_{}_performance".format(self.configuration.framework_name, self.scenario.scenario_id, self.configuration.use_case))
        plt.show()

    def compute_rmsd(self, x):
        sum_x = 0
        sum_y = 0
        for i in range(self.configuration.iter_length):
            sum_x += (self.configuration.reference_path[i, 0] - x[i, 0]) ** 2
            sum_y += (self.configuration.reference_path[i, 1] - x[i, 1]) ** 2
        rmsd = ca.sqrt(sum_x / (self.configuration.iter_length - 1))
        rmsd_y = ca.sqrt(sum_y / (self.configuration.iter_length - 1))
        print('rmsd of x: %s' % rmsd)
        print('rmsd of y: %s' % rmsd_y)
        return rmsd

    def plan(self):
        if self.configuration.framework_name == "casadi":
            optimizer = CasadiOptimizer(configuration=self.configuration, init_values=self.init_values, predict_horizon=self.predict_horizon)
        elif self.configuration.framework_name == "forcespro":
            optimizer = ForcesproOptimizer(configuration=self.configuration, init_values=self.init_values, predict_horizon=self.predict_horizon)
        else:
            raise ValueError("Only casadi and forcespro are available!")
            
        final_states, final_control_inputs, final_solve_time = optimizer.optimize()
        ego_vehicle_trajectory, ego_vehicle = self.plot_and_create_gif(final_states, final_control_inputs, final_solve_time)
        self.compute_rmsd(final_states)
        return ego_vehicle_trajectory, ego_vehicle
