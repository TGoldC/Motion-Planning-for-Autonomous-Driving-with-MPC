from commonroad.scenario.trajectory import State, Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.visualization.mp_renderer import MPRenderer
import commonroad_dc.feasibility.feasibility_checker as feasibility_checker
from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics, VehicleType
from MPC_Planner.optimizer import *
import imageio
import sys
sys.path.append("..")


class MPCPlanner(object):
    def __init__(self, scenario, planning_problem, configuration, predict_horizon):
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.configuration = configuration
        self.init_values = self.get_init_values()
        self.predict_horizon = predict_horizon

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

    # def _generate_reference_path(self):
    #     """
    #     position_init: 1D array (x_pos, y_pos); the initial position of the planning problem
    #     reference_path: the output of route planner, which is considered as reference path
    #     """
    #     route_planer = RoutePlanner(self.scenario, self.planning_problem, backend=RoutePlanner.Backend.NETWORKX)
    #     candidate_route = route_planer.plan_routes()
    #     origin_reference_path = candidate_route.retrieve_best_route_by_orientation().reference_path  # 800*2
    #     return origin_reference_path
    #
    # def resample_reference_path(self):
    #     cumsum_distance = compute_polyline_length(self.origin_reference_path)
    #     reference_path = np.array(chaikins_corner_cutting(self.origin_reference_path))
    #     resampled_reference_path = resample_polyline(reference_path, step=self.desired_velocity * self.delta_t)
    #     iter_length = resampled_reference_path.shape[0]
    #     return resampled_reference_path, iter_length

    def plot_and_create_gif(self, x):
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

        # plot the scenario for each time step
        for i in range(0, self.configuration.iter_length):
            plt.figure(figsize=(25, 10))
            rnd = MPRenderer()
            self.scenario.draw(rnd, draw_params={'time_begin': i})
            ego_vehicle.draw(rnd, draw_params={'time_begin': i, 'dynamic_obstacle': {
                'vehicle_shape': {'occupancy': {'shape': {'rectangle': {
                    'facecolor': 'r'}}}}}})
            self.planning_problem.draw(rnd)
            rnd.render()
            rnd.ax.plot(self.configuration.reference_path[i, 0], self.configuration.reference_path[i, 1], color='r', marker='.', markersize=1, zorder=19, linewidth=0.8,
                        label='reference path')
            plt.savefig("../test/figures_{}/temp{}.png".format(str(self.scenario.scenario_id), i))
            # plt.show()
            plt.clf()

        figures_list = []
        for i in range(0, self.configuration.iter_length):
            figures_list.append("../test/figures_{}/temp{}.png".format(str(self.scenario.scenario_id), i))
        with imageio.get_writer("../test/gif_{}.gif".format(str(self.scenario.scenario_id)), mode='I') as writer:
            for filename in figures_list:
                image = imageio.imread(filename)
                writer.append_data(image)

    # TODO
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
        init_values = self.get_init_values()

        if name_solver == "casadi" or name_solver == "Casadi":
            optimizer = CasadiOptimizer(configuration=self.configuration, init_values=init_values, predict_horizon=self.predict_horizon)
        else:
            optimizer = ForcesproOptimizer(configuration=self.configuration, init_values=init_values, predict_horizon=self.predict_horizon)
            
        final_states = optimizer.optimize()
        self.plot_and_create_gif(final_states)





