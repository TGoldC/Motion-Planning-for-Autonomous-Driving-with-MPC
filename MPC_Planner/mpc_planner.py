from turtle import distance
import matplotlib.pyplot as plt
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker, create_collision_object
import commonroad_dc.feasibility.feasibility_checker as feasibility_checker
from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics, VehicleType
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from optimizer import *
import imageio
import sys
sys.path.append("..")


class MPC_Planner(object):
    def __init__(self, scenario, planning_problem):
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.origin_reference_path = self._generate_reference_path()
        self.desired_velocity, self.delta_t = self.get_desired_velocity_and_delta_t()
        self.init_values = self.get_init_values()
        self.resampled_reference_path, self.iter_length = self.resample_reference_path()
        self.orientation = compute_orientation_from_polyline(self.resampled_reference_path)
        self.static_obstacles = self.static_obstacle()

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

    def plot_and_create_gif(self, x):
        ego_vehicle_initial_state = State(position=np.array([self.init_values[0][0], self.init_values[0][1]]),
                                               velocity=self.init_values[2],
                                               orientation=self.init_values[3],
                                               time_step=0)
        # generate the states for the obstacle for time steps 1 to 40 by assuming constant velocity
        state_list = []

        for i in range(1, self.iter_length):
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
        self.scenario.add_objects(self.static_obstacles)
        #print(self.resampled_reference_path)

        # plot the scenario for each time step
        #for i in range(0, 30):
        #    plt.figure(figsize=(25, 10))
        #    rnd = MPRenderer()
        #    self.scenario.draw(rnd, draw_params={'time_begin': i})
        #    ego_vehicle.draw(rnd, draw_params={'time_begin': i, 'dynamic_obstacle': {
        #                'vehicle_shape': {'occupancy': {'shape': {'rectangle': {
        #                    'facecolor': 'r'}}}}}})
        #    self.planning_problem.draw(rnd)
        #    rnd.render()
        #    rnd.ax.plot(self.resampled_reference_path[:, 0], self.resampled_reference_path[:, 1], color='r', marker='_', markersize=1, zorder=19, linewidth=0.8,
        #                label='reference path')
        #    plt.savefig("../figures/test_temp{}.png".format(i))
        #    plt.clf()
#
        #figures_list = []
        #for i in range(0, 25):
        #    figures_list.append("../figures/test_temp{}.png".format(i))
        #with imageio.get_writer('test.gif', mode='I') as writer:
        #    for filename in figures_list:
        #        image = imageio.imread(filename)
        #        writer.append_data(image)

        # create collision checker from scenario
        cc = create_collision_checker(self.scenario)
        
        # create ego vehicle collision object
        ego_vehicle_co = create_collision_object(ego_vehicle)
        # create the road boundary
        _, road_boundary = create_road_boundary_obstacle(self.scenario)
        
        # add road boundary to collision checker
        cc.add_collision_object(road_boundary)
        
        # Again: check if ego vehicle collides
        res = cc.collide(ego_vehicle_co)
        print('Collision between the ego vehicle and the road boundary: %s' % res)

        ## set time step as scenario time step
        #dt = self.scenario.dt
        #
        ## choose vehicle model (here kinematic single-track model)
        #vehicle_dynamics = VehicleDynamics.KS(VehicleType.BMW_320i)
        #
        ## check feasibility of planned trajectory for the given vehicle model
        #feasible, reconstructed_inputs = feasibility_checker.trajectory_feasibility(ego_vehicle_trajectory, vehicle_dynamics, dt)
        #print('The planned trajectory is feasible: %s' % feasible)
        
    
    def two_dim_plot(self, x):
        #2D plot 
        # Todo: subplots
        plt.figure('Draw')
        deviation = self.resampled_reference_path[:, 0]- x[:,0]
        #print(self.resampled_reference_path)
        plt.plot(np.arange(self.iter_length), deviation)
        plt.title('deviation of x')
        plt.xlabel('iter')
        plt.ylabel('x')
        plt.show()
        #
        #plt.figure('Draw')
        #deviation = self.resampled_reference_path[:, 1]- x[:,1]
        #plt.plot(np.arange(self.iter_length), deviation)
        #plt.title('deviation of y')
        #plt.xlabel('iter')
        #plt.ylabel('y')
        #plt.show()
#
        #plt.figure('Draw')
        #plt.plot(np.arange(self.iter_length), x[:,2])
        #plt.title('steering angle')
        #plt.xlabel('iter')
        #plt.ylabel('steering angle')
        #plt.show()
#
        #plt.figure('Draw')
        #plt.plot(np.arange(self.iter_length), x[:,3])
        #plt.title('Velocity')
        #plt.xlabel('iter')
        #plt.ylabel('velocity')
        #plt.show()
#
        #plt.figure('Draw')
        #plt.plot(np.arange(self.iter_length), self.orientation- x[:,4])
        #plt.title('deviation of Orientation')
        #plt.xlabel('iter')
        #plt.ylabel('orientation')
        #plt.show()

    def static_obstacle(self,):
        static_obstacle_id = self.scenario.generate_object_id()
        static_obstacle_type = ObstacleType.PARKED_VEHICLE
        static_obstacle_shape = Rectangle(width = 2.0, length = 4.5)
        static_obstacle_initial_state = State(position = np.array([50.24, 53.48]), orientation = 0.02, time_step = 0)
        
        # feed in the required components to construct a static obstacle
        static_obstacles = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape, static_obstacle_initial_state)
        
        # add the static obstacle to the scenario
        #scenario.add_objects(static_obstacle)
        return static_obstacles

    def compute_approximating_circle_radius(ego_length, ego_width):
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

        #the distance between the first and last circle is computed as
        distance_centers = square_length * (3-1)
    
        # Calculate minimum radius
        diagonal_square = np.sqrt((square_length / 2) ** 2 + (ego_width / 2) ** 2)
    
        # Round up value
        if diagonal_square > round(diagonal_square, 1):
            approx_radius = round(diagonal_square, 1) + 0.1
        else:
            approx_radius = round(diagonal_square, 1)
    
        return approx_radius, round(square_length * 2, 1)

    def compute_centers_of_approximation_cicles(self, x_position, y_position, ego_length, orientation):
        #the distance between the first and last circle is computed as
        distance_centers = (ego_length / 3) * (3 - 1)
        
        #compute the center position of first circle (front)
        center_fw = [x_position + (distance_centers / 2) * ca.cos(orientation), y_position + (distance_centers / 2) * ca.sin(orientation)]

        #compute the center position of second circle (rear)
        center_rw = [x_position - (distance_centers / 2) * ca.cos(orientation), y_position - (distance_centers / 2) * ca.sin(orientation)]
        
        return center_fw, center_rw


 
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
        self.plot_and_create_gif(final_states)
        self.two_dim_plot(final_states)


if __name__ == '__main__':
    # define the scenario and planning problem
    path_scenario = "../scenarios/"  # relative dir path, scenarios and mpc_planner.py are in the same directory
    id_scenario = "USA_Lanker-2_18_T-1.xml"  # "ZAM_Tutorial_Urban-3_2.xml"
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario + id_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
    MPC_Planner_instance = MPC_Planner(scenario, planning_problem)
    #MPC_Planner_instance.plan("forcespro")
    MPC_Planner_instance.plan("casadi")
    # TODO introduce Collision Avoidance feature



