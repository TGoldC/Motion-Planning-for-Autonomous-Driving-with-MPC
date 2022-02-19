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


def save_data(x, u, solve_time, path_to_save):
    np.save(path_to_save + 'save_data', x, u, solve_time)


class MPCPlanner(object):
    def __init__(self, scenario, planning_problem, configuration, predict_horizon):
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.configuration = configuration
        self.init_values = self.get_init_values()
        # self.static_obstacles = self.static_obstacle()  # : StaticObstacle
        self.predict_horizon = predict_horizon

    def get_init_values(self):
        """"
        Get initial values from planning problem
        :return: a tuple containing initial position -> ndarray(2,)
                                    initial velocity -> float
                                    initial acceleration -> float
                                    initial orientation -> float
        """

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

    def static_obstacle(self,):
        """
        Used to set an additional obstacle in the scenario
        :return: a static obstacle in CommonRoad framework
        """
        static_obstacle_id = self.scenario.generate_object_id()
        static_obstacle_type = ObstacleType.PARKED_VEHICLE
        static_obstacle_shape = Rectangle(width=self.configuration.static_obstacle["width"], length=self.configuration.static_obstacle["length"])
        static_obstacle_initial_state = State(position=np.array([self.configuration.static_obstacle["position_x"], self.configuration.static_obstacle["position_y"]]),
                                              orientation=self.configuration.static_obstacle["orientation"], time_step=0)
        
        # feed in the required components to construct a static obstacle
        static_obstacles = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape, static_obstacle_initial_state)
        
        return static_obstacles

    def plot_and_create_gif(self, x, u, solve_time):
        """
        Transform the planned trajectory into CommonRoad Trajectory. Plot and save animation and 2D figures
        """
        ego_vehicle_initial_state = State(position=np.array([self.init_values[0][0], self.init_values[0][1]]),
                                          velocity=self.init_values[2],
                                          orientation=self.init_values[3],
                                          time_step=0)
        # generate the states for the obstacle for time steps 1 to 40 by assuming constant velocity
        state_list = []

        for i in range(1, self.configuration.iter_length):
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
                                      ego_vehicle_type,
                                      ego_vehicle_shape,
                                      ego_vehicle_initial_state,
                                      ego_vehicle_prediction)

        # create a folder to save images at each time step
        path_figures = "../test/figures_{}_{}_{}/".format(self.configuration.framework_name, str(self.scenario.scenario_id), self.configuration.use_case)
        folder_exist = os.path.exists(path_figures)
        if not folder_exist:
            os.makedirs(path_figures)

        # plot the scenario for each time step
        for i in range(0, self.configuration.iter_length):
            plt.figure(figsize=(25, 10))
            if "ZAM_Over" in str(self.scenario.scenario_id):
                rnd = MPRenderer(plot_limits=[25, 125, -10, 20])
            elif "USA_Lanker" in str(self.scenario.scenario_id):
                rnd = MPRenderer(plot_limits=[-20, 60, -25, 100])
            else:
                raise ValueError("Only ZAM_Over-1_1 and USA_Lanker-2_18_T-1 can be processed!")

            self.scenario.draw(rnd, draw_params={'time_begin': i})
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
            for filename in figures_list:
                image = imageio.imread(filename)
                writer.append_data(image)

        # plot 2D figures for analyse
        path_2D_figures = "../test/2D_plots_{}_{}_{}/".format(self.configuration.framework_name, str(self.scenario.scenario_id), self.configuration.use_case)
        folder_exist = os.path.exists(path_2D_figures)
        if not folder_exist:
            os.makedirs(path_2D_figures)

        self.plot_deviation_euclidean_dis(x, path_2D_figures)
        self.plot_control_inputs(u, path_2D_figures)
        self.plot_solve_time(solve_time, path_2D_figures)
        self.plot_path(x, u, path_2D_figures)

        # Compute Root-mean-square deviation of the x-position and y-position for lane_following use case
        if self.configuration.use_case == "lane_following":
            self.compute_rmsd(x, path_2D_figures)

        return ego_vehicle_trajectory, ego_vehicle

    def plot_deviation_euclidean_dis(self, x, save_path):
        """
        Plot the euclidean distance between planned trajectory and original reference path
        """
        plt.figure()
        # deviation = np.hstack((self.configuration.reference_path[:, 0], self.configuration.reference_path[-1, 0])) - x[:, 0]
        nearest_points = np.zeros((x.shape[0], 2))
        for i in range(x.shape[0]):
            nearest_points[i] = self.configuration.origin_reference_path[find_closest_point(self.configuration.origin_reference_path, x[i, 0:2])]

        deviation_x = nearest_points[:, 0] - x[:, 0]
        deviation_y = nearest_points[:, 1] - x[:, 1]
        deviation = np.sqrt(deviation_x ** 2 + deviation_y ** 2)
        np.savetxt(save_path+'deviation.txt', deviation)

        plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, deviation)
        plt.title('deviation with reference path ')
        plt.xlabel('time [s]')
        plt.ylabel('deviation in euclidean distance [m]')

        plt.savefig(save_path + "2D_plot_{}_{}_{}_deviation".format(self.configuration.framework_name, self.scenario.scenario_id, self.configuration.use_case))
        plt.show()

    def plot_control_inputs(self, u, save_path):
        """
        Plot control inputs over the time
        """
        np.savetxt(save_path+'control inputs.txt', u)
        plt.figure()
        # print("control inputs: ", repr(u))
        # control_inputs_unnoised = np.array()
        plt.subplot(2, 1, 1)
        plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t,  np.rad2deg(u[:, 0]), color="b")
        # plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, np.rad2deg(control_inputs_unnoised[:, 0]), color="g", label="unnoised")
        plt.title('steering velocity')
        plt.xlabel('time [s]')
        plt.ylabel('delta_v [deg/s]')
        plt.subplots_adjust(hspace=0.8)

        plt.subplot(2, 1, 2)
        plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, u[:, 1], color="b")
        # plt.plot(np.arange(self.configuration.iter_length) * self.configuration.delta_t, control_inputs_unnoised[:, 1], color="g", label="unnoised")
        plt.title('longitudinal acceleration')
        plt.xlabel('time [s]')
        plt.ylabel('long. acc. [m/s2]')

        plt.savefig(save_path + "2D_plot_{}_{}_{}_control_inputs".format(self.configuration.framework_name, self.scenario.scenario_id, self.configuration.use_case))
        plt.show()

    def plot_solve_time(self, solve_time, save_path):
        """
        Plot solve time over the time
        """
        np.savetxt(save_path+'solve time.txt', solve_time)
        plt.figure()
        # print("solve_time", repr(solve_time))
        plt.plot(np.arange(self.configuration.iter_length), solve_time*1000,  color='b')
        # plt.plot(np.arange(self.configuration.iter_length), solve_time_casadi * 1000, color='g', label='casadi computation time')
        plt.title('Computation time over iteration')
        plt.xlabel('iteration')
        plt.ylabel('Computation time [ms]')

        plt.savefig(save_path + "2D_plot_{}_{}_{}_solve_time".format(self.configuration.framework_name, self.scenario.scenario_id, self.configuration.use_case))
        plt.show()

    def plot_path(self, x, u, save_path):
        """
        Plot the reference path and planned trajectory in one figure to show the performance, respectively in x and y direction
        """
        np.savetxt(save_path + 'planned states.txt', x)
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
        plt.legend()
        plt.title('Performance in y-direction')
        plt.xlabel('time [s]')
        plt.ylabel('y-position [m]')

        plt.subplots_adjust(wspace=0.25, hspace=0.5)
        plt.savefig(save_path + "2D_plot_{}_{}_{}_performance".format(self.configuration.framework_name, self.scenario.scenario_id, self.configuration.use_case))
        plt.show()

    def compute_rmsd(self, x, path_to_save):
        """
        Compute Root-mean-square deviation of the x-position and y-position for lane_following use case
        """
        sum_x = 0
        sum_y = 0
        for i in range(self.configuration.iter_length):
            sum_x += (self.configuration.reference_path[i, 0] - x[i, 0]) ** 2
            sum_y += (self.configuration.reference_path[i, 1] - x[i, 1]) ** 2
        rmsd_x = ca.sqrt(sum_x / (self.configuration.iter_length - 1))
        rmsd_y = ca.sqrt(sum_y / (self.configuration.iter_length - 1))
        np.savetxt(path_to_save + 'RMSD.txt', ca.vertcat(rmsd_x, rmsd_y))
        print('RMSD of x-position: %s' % rmsd_x)
        print('RMSD of y-position: %s' % rmsd_y)

    def plan(self):
        """
        Instantiate an optimizer according to framework_name to solve MPC problem, plot animation and 2D figures
        :return: ego_vehicle_trajectory -> Trajectory
                 ego_vehicle -> DynamicObstacle
        """
        # Instantiate an optimizer according to framework_name
        if self.configuration.framework_name == "casadi":
            optimizer = CasadiOptimizer(configuration=self.configuration, init_values=self.init_values, predict_horizon=self.predict_horizon)
        elif self.configuration.framework_name == "forcespro":
            optimizer = ForcesproOptimizer(configuration=self.configuration, init_values=self.init_values, predict_horizon=self.predict_horizon)
        else:
            raise ValueError("Only casadi and forcespro are available!")

        # Use optimizer to solve MPC problem
        final_states, final_control_inputs, final_solve_time = optimizer.optimize()

        # plot animation and 2D figures
        ego_vehicle_trajectory, ego_vehicle = self.plot_and_create_gif(final_states, final_control_inputs, final_solve_time)

        return ego_vehicle_trajectory, ego_vehicle
