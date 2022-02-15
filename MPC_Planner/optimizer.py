import casadi as ca
import numpy as np
<<<<<<< HEAD
####import forcespro
####import forcespro.nlp
from configuration import Vehicle_dynamics
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
=======
import forcespro
import forcespro.nlp
from MPC_Planner.configuration import VehicleDynamics
>>>>>>> develop
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_dc.geometry.util import (chaikins_corner_cutting, compute_curvature_from_polyline, resample_polyline,
                                         compute_pathlength_from_polyline, compute_orientation_from_polyline, compute_polyline_length)
from matplotlib.animation import FuncAnimation
<<<<<<< HEAD
=======
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle, StaticObstacle
from commonroad.geometry.shape import Shape
from MPC_Planner.configuration import compute_centers_of_approximation_circles, compute_approximating_circle_radius
>>>>>>> develop
import sys
import time


class Optimizer(object):
    def __init__(self, configuration, init_values, predict_horizon):
        self.configuration = configuration
        # steering angles
        self.delta_min = configuration.p.steering.min  # -1.066
        self.delta_max = configuration.p.steering.max  # 1.066
        # steering velocity
        self.deltav_min = configuration.p.steering.v_min  # -0.4
        self.deltav_max = configuration.p.steering.v_max  # 0.4
        # velocity
        self.v_min = 0  # highway
        self.v_max = configuration.p.longitudinal.v_max  # 50.8
        # acceleration
        self.a_max = configuration.p.longitudinal.a_max  # 11.5

        self.resampled_path_points = configuration.reference_path
        self.iter_length = configuration.iter_length
        self.init_position, self.init_velocity, self.init_acceleration, self.init_orientation = init_values[0], init_values[1], init_values[2], init_values[3]
        self.delta_t = configuration.delta_t
        self.desired_velocity = configuration.desired_velocity
        self.orientation = configuration.orientation
        self.predict_horizon = predict_horizon
        self.weights_setting = configuration.weights_setting  # store weights using a dictionary

        # define the three circles of obstacle
        self.obstacle_circles_centers_tuple = compute_centers_of_approximation_circles(configuration.static_obstacle["position_x"],
                                                                                       configuration.static_obstacle["position_y"],
                                                                                       configuration.static_obstacle["length"],
                                                                                       configuration.static_obstacle["width"],
                                                                                       configuration.static_obstacle["orientation"])

        # get approximate radius for ego and obstacle vehicles
        self.radius_obstacle, _ = compute_approximating_circle_radius(configuration.static_obstacle["length"], configuration.static_obstacle["width"])
        self.radius_ego, _ = compute_approximating_circle_radius(configuration.p.l, configuration.p.w)

    def equal_constraints(self, *args, **kwargs):
        pass

    def inequal_constraints(self, *args, **kwargs):
        pass

    def cost_function(self,  *args, **kwargs):
        pass

    def solver(self):
        pass

    def optimize(self):
        pass


<<<<<<< HEAD
#class ForcesproOptimizer(Optimizer):
#   def __init__(self, resampled_path_points, iter_length, init_values, delta_t, desired_velocity, orientation, p, predict_horizon):
#       super(ForcesproOptimizer, self).__init__(resampled_path_points, iter_length, init_values, delta_t, desired_velocity, orientation, p, predict_horizon)
#       # self. = ... # add own attributes
#
#   @staticmethod
#   def equal_constraints():
#       # We use an explicit RK4 integrator here to discretize continuous dynamics
#       integrator_stepsize = 0.1
#       return lambda z: forcespro.nlp.integrate(Vehicle_dynamics.KS_casadi, z[2:7], z[0:2],
#                                                integrator=forcespro.nlp.integrators.RK4,
#                                                stepsize=integrator_stepsize)
#
#   def inequal_constraint(self):
#       # Inequality constraints
#       # z = [deltaDot,aLong,xPos,yPos,delta,v,psi]
#       #  upper/lower variable bounds lb <= z <= ub
#       #                     inputs      |  states
#       #                deltaDot aLong    x    y   delta    v    psi
#       # model.lb = np.array([-0.4, -11.5, -10, -30, -1.066, 0., -np.inf])  # 1.066 = 61 degree
#       # model.ub = np.array([+0.4, +11.5, 100, 100, +1.066, 50.8, np.inf])
#       #                     inputs      |  states
#       #                       deltaDot            aLong    x    y   delta                 v      psi
#       low_bound = np.array([self.deltav_min, -self.a_max, -10, -30, self.delta_min, self.v_min, -np.inf])  # 1.066 = 61 degree
#       upper_bound = np.array([self.deltav_max, self.a_max, 100, 100, self.delta_max, self.v_max, np.inf])
#       return low_bound, upper_bound
#
#   @staticmethod
#   def cost_function(z, current_target):
#       """Least square costs on deviating from the path and on the inputs
#       z = [deltaDot,aLong,xPos,yPos,delta,v,psi]
#       current_target = [point on path x, point on path y, desired velocity, current orientation]
#       """
#       return (400.0 * (z[2] - current_target[0]) ** 2  # costs on deviating on the path in x-direction
#               + 400.0 * (z[3] - current_target[1]) ** 2  # costs on deviating on the path in y-direction
#               + 200 * z[4] ** 2  # penalty on steering angle
#               + 200 * (z[5] - current_target[2]) ** 2  # penalty on velocity
#               + 0.1 * (z[6] - current_target[3]) ** 2  # penalty on heading angle
#               + 0.1 * z[0] ** 2  # penalty on input velocity of steering angle
#               + 0.1 * z[1] ** 2)  # penalty on input longitudinal acceleration
#
#   @staticmethod
#   def cost_functionN(z, current_target):
#       """Increased least square costs for last stage on deviating from the path and on the inputs F and phi
#       z = [F,phi,xPos,yPos,v,theta,delta]
#       current_target = [point on path x, point on path y, desired velocity, current orientation]
#       """
#       return (800.0 * (z[2] - current_target[0]) ** 2  # costs on deviating on the path in x-direction
#               + 800.0 * (z[3] - current_target[1]) ** 2  # costs on deviating on the path in y-direction
#               + 400 * z[4] ** 2
#               + 400 * (z[5] - current_target[2]) ** 2
#               + 0.2 * (z[6] - current_target[3]) ** 2
#               + 0.2 * z[0] ** 2  # penalty on input velocity of steering angle
#               + 0.2 * z[1] ** 2)  # penalty on input longitudinal acceleration
#
#   def solver(self):
#       """Generates and returns a FORCESPRO solver that calculates a path based on constraints and dynamics while minimizing an objective function"""
#       # Model Definition
#       # define problem dimensions
#       model = forcespro.nlp.SymbolicModel()
#       model.N = self.predict_horizon  # horizon length
#       model.nvar = 7  # number of variables
#       model.neq = 5  # number of equality constraints
#       model.npar = 4  # number of runtime parameters
#
#       # define constraints
#       model.eq = self.equal_constraints()
#       model.lb, model.ub = self.inequal_constraint()
#
#       # define cost function
#       model.objective = self.cost_function
#       model.objectiveN = self.cost_functionN
#
#       # Indices on LHS of dynamical constraint - for efficiency reasons, make sure the matrix E has structure [0 I] where I is the identity matrix.
#       model.E = np.concatenate([np.zeros((5, 2)), np.eye(5)], axis=1)
#
#       # Initial condition on vehicle states x
#       model.xinitidx = range(2, 7)  # use this to specify on which variables initial conditions are imposed
#
#       # Solver Generation
#       # Set solver options
#       codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
#       codeoptions.maxit = 200  # Maximum number of iterations
#       codeoptions.printlevel = 0  # Use printlevel = 2 to print progress (but not for timings)
#       codeoptions.optlevel = 0  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
#       codeoptions.cleanup = False
#       codeoptions.timing = 1
#       codeoptions.nlp.hessian_approximation = 'bfgs'
#       codeoptions.solvemethod = 'SQP_NLP'  # choose the solver method Sequential Quadratic Programming
#       codeoptions.nlp.bfgs_init = 2.5 * np.identity(7)  # np.identity creat a square matrix with 1 in the diagonal line
#       codeoptions.sqp_nlp.maxqps = 1  # maximum number of quadratic problems to be solved
#       codeoptions.sqp_nlp.reg_hessian = 5e-9  # increase this if exitflag=-8
#       # change this to your server or leave uncommented for using the standard embotech server at https://forces.embotech.com, codeoptions.server = 'https://forces.embotech.com'
#       # Creates code for symbolic model formulation given above, then contacts server to generate new solver
#       solver = model.generate_solver(options=codeoptions)
#
#       return model, solver
#
#   def optimize(self):
#       # generate forcespro model and solver
#       model, solver = self.solver()
#
#       # Simulation
#       sim_length = self.iter_length
#
#       # Variables for storing simulation data
#       x = np.zeros((5, sim_length + 1))  # states
#       u = np.zeros((2, sim_length))  # inputs
#
#       # Set initial guess to start solver from
#       x0i = np.zeros((model.nvar, 1))  # model.nvar = 7 变量个数  shape of x0i = [7, 1]
#       x0 = np.transpose(np.tile(x0i, (1, model.N)))  # horizon length 10,  shape of x0 = [10, 7]
#
#       xinit = np.transpose(np.array([self.init_position[0], self.init_position[1], 0., self.init_velocity, self.init_orientation]))  # Set initial states
#       # state x = [xPos,yPos,delta,v,psi]
#       x[:, 0] = xinit
#
#       problem = {"x0": x0,
#                  "xinit": xinit}
#
#       start_pred = np.reshape(problem["x0"], (7, model.N))  # first prediction corresponds to initial guess
#
#       # generate plot with initial values
#       # self.createPlot(x, u, start_pred, sim_length, model, self.resampled_path_points.T, xinit)  # draw the resampled path, interpolate with same distance between two points
#
#       # Simulation
#       for k in range(sim_length):
#           print("k=", k)
#
#           # Set initial condition
#           problem["xinit"] = x[:, k]
#
#           # Set runtime parameters (here, the next N points on the path)
#           # next_path_points = MPC_Planner_instance.extract_next_path_points(resampled_path_points.T, x[0:2, k], model.N)
#           # return N points, which nearest with x[0:2, k], not include x[0:2, k] itself，shape=2*N
#           next_path_points = self.resampled_path_points.T[:, k + 1:k + 1 + model.N]
#           next_path_orientation = self.orientation[k + 1:k + 1 + model.N]
#           while next_path_points.shape[1] != model.N:
#               next_path_points = np.hstack((next_path_points, self.resampled_path_points[-1].reshape(2, -1)))
#               next_path_orientation = np.hstack((next_path_orientation, self.orientation[-1]))
#
#           next_path_points = np.vstack((next_path_points, np.ones(model.N) * self.desired_velocity, next_path_orientation))
#           problem["all_parameters"] = np.reshape(np.transpose(next_path_points), (4 * model.N, 1))
#           # shape = 2N * 1  【x y x y x y...】.T
#
#           # Time to solve the NLP!
#           output, exitflag, info = solver.solve(problem)
#           print("exitflag = ", exitflag)
#
#           # Make sure the solver has exited properly.
#           assert exitflag == 1, "bad exitflag"  # does not succeed， return AssertionError: bad exitflag
#           sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"
#                            .format(info.it, info.solvetime))
#
#           # Extract output
#           temp = np.zeros((np.max(model.nvar), model.N))  # initialization temp.shape=7*N
#           for i in range(0, model.N):
#               temp[:, i] = output['x{0:02d}'.format(i + 1)]
#           pred_u = temp[0:2, :]  # N predicted inputs, 2*N
#           pred_x = temp[2:7, :]  # N predicted states, 5*N
#
#           # Apply optimized input u of first stage to system and save simulation data
#           u[:, k] = pred_u[:, 0]
#           # add Gaussian noise onto inputs
#           # noise_mean = np.array([0, 0])
#           # noise_std = np.array([0.01, 0.01])  # The larger standard deviation is, the wider range of noise is.
#           # noise = np.random.normal(noise_mean, noise_std, (2,))
#           # u[:, k] = pred_u[:, 0] + noise
#           x[:, k + 1] = np.transpose(model.eq(np.concatenate((u[:, k], x[:, k]))))  # with k-th step 's x and u，update states at (k+1)-th
#
#           # plot results of current simulation step
#           # self.updatePlots(x, u, pred_x, pred_u, model, k)
#
#           # if k == sim_length - 1:
#           #     fig = plt.gcf()
#           #     ax_list = fig.axes
#           #     ax_list[0].get_lines().pop(-1).remove()  # remove old prediction of trajectory
#           #     ax_list[0].legend(['desired trajectory', 'init pos', 'car trajectory'], loc='lower right')
#           #     plt.show()
#           # else:
#           #     plt.draw()
#       return x
#
#   @staticmethod
#   def createPlot(x, u, start_pred, sim_length, model, path_points, xinit):
#       """Creates a plot and adds the initial data provided by the arguments"""
#       # Create empty plot
#       fig = plt.figure()
#       plt.clf()  # 用其所有轴清除整个当前图形
#       gs = GridSpec(5, 2, figure=fig)  # creat a 5*2 grid
#
#       # Plot trajectory
#       ax_pos = fig.add_subplot(gs[:, 0])
#       l0, = ax_pos.plot(np.transpose(path_points[0, :]), np.transpose(path_points[1, :]), 'rx', markersize=5)  # first line
#       l1, = ax_pos.plot(xinit[0], xinit[1], 'bx')
#       plt.title('Position')
#       # plt.axis('equal')
#       # for scenario 1
#       # plt.xlim([-20., 200.])
#       # plt.ylim([-3.5, 2.5])
#       # for scenario 2
#       plt.xlim([-10., 60.])
#       plt.ylim([-20., 60.])
#       plt.xlabel('x-coordinate')
#       plt.ylabel('y-coordinate')
#       l2, = ax_pos.plot(x[0, 0], x[1, 0], 'b-')
#       l3, = ax_pos.plot(start_pred[2, :], start_pred[3, :], 'g-')  # start_pred is z
#       ax_pos.legend([l0, l1, l2, l3], ['desired trajectory', 'init pos', 'car trajectory', 'predicted car traj.'],
#                     loc='lower right')
#
#       # Plot steering angle
#       ax_delta = fig.add_subplot(5, 2, 2)
#       plt.grid("both")  # show the grid
#       plt.title('steering angle')
#       plt.xlim([0., sim_length - 1])
#       plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.ub[4], model.ub[4]])), 'r:')
#       plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.lb[4], model.lb[4]])), 'r:')
#       ax_delta.plot(np.rad2deg(x[2, 0]), '-b')
#       ax_delta.plot(np.rad2deg(start_pred[4, :]), 'g-')
#
#       # Plot velocity
#       ax_v = fig.add_subplot(5, 2, 4)
#       plt.grid("both")
#       plt.title('velocity')
#       plt.ylim([0., 20])
#       plt.xlim([0., sim_length - 1])
#       plt.plot([0, sim_length - 1], np.transpose([model.ub[5], model.ub[5]]), 'r:')
#       plt.plot([0, sim_length - 1], np.transpose([model.lb[5], model.lb[5]]), 'r:')
#       ax_v.plot(0., x[3, 0], 'b-')
#       ax_v.plot(start_pred[5, :], 'g-')
#
#       # Plot heading angle
#       ax_psi = fig.add_subplot(5, 2, 6)
#       plt.grid("both")
#       plt.title('Heading angle')
#       plt.xlim([0., sim_length - 1])
#       plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.ub[6], model.ub[6]])), 'r:')
#       plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.lb[6], model.lb[6]])), 'r:')
#       ax_psi.plot(np.rad2deg(x[4, 0]), 'b-')
#       ax_psi.plot(np.rad2deg(start_pred[6, :]), 'g-')
#
#       # Plot velocity of steering angle
#       ax_deltaDot = fig.add_subplot(5, 2, 8)
#       plt.grid("both")
#       plt.title('Velocity of steering angle')
#       plt.xlim([0., sim_length - 1])
#       plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.ub[0], model.ub[0]])), 'r:')
#       plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.lb[0], model.lb[0]])), 'r:')
#       ax_deltaDot.step(0, np.rad2deg(u[0, 0]), 'b-')  # step函数可以认为是plot函数绘制阶梯图
#       ax_deltaDot.step(range(model.N), start_pred[0, :], 'g-')
#
#       # Plot longitudinal acceleration
#       ax_aLong = fig.add_subplot(5, 2, 10)
#       plt.grid("both")
#       plt.title('Longitudinal acceleration')
#       plt.xlim([0., sim_length - 1])
#       plt.plot([0, sim_length - 1], np.transpose([model.ub[1], model.ub[1]]), 'r:')
#       plt.plot([0, sim_length - 1], np.transpose([model.lb[1], model.lb[1]]), 'r:')
#       ax_aLong.step(0., u[1, 0], 'b-')
#       ax_aLong.step(range(model.N), start_pred[1, :], 'g-')
#
#       plt.tight_layout()
#
#       # Make plot fullscreen. Comment out if platform dependent errors occur.
#       mng = plt.get_current_fig_manager()
#
#   @staticmethod
#   def updatePlots(x, u, pred_x, pred_u, model, k):
#       """ Deletes old data sets in the current plot and adds the new data sets
#       given by the arguments x, u and predicted_z to the plot.
#       x: matrix consisting of a set of state column vectors
#       u: matrix consisting of a set of input column vectors
#       pred_x: predictions for the next N state vectors
#       pred_u: predictions for the next N input vectors
#       model: model struct required for the code generation of FORCESPRO
#       k: simulation step
#       """
#       fig = plt.gcf()  # 获取当前图表 get the current figures
#       ax_list = fig.axes  # axes可以理解为 子区域
#
#       # Delete old data in plot
#       ax_list[0].get_lines().pop(-1).remove()  # remove old prediction of trajectory
#       ax_list[0].get_lines().pop(-1).remove()  # remove old trajectory
#
#       ax_list[1].get_lines().pop(-1).remove()  # remove old prediction of steering angle
#       ax_list[1].get_lines().pop(-1).remove()  # remove old steering angle
#       ax_list[2].get_lines().pop(-1).remove()  # remove old prediction of velocity
#       ax_list[2].get_lines().pop(-1).remove()  # remove old velocity
#       ax_list[3].get_lines().pop(-1).remove()  # remove old prediction of heading angle
#       ax_list[3].get_lines().pop(-1).remove()  # remove old heading angle
#       ax_list[4].get_lines().pop(-1).remove()  # remove old prediction of velocity of steering angle
#       ax_list[4].get_lines().pop(-1).remove()  # remove old velocity of steering angle
#       ax_list[5].get_lines().pop(-1).remove()  # remove old prediction of longitudinal acceleration
#       ax_list[5].get_lines().pop(-1).remove()  # remove old longitudinal acceleration
#
#       # Update plot with current simulation data
#       ax_list[0].plot(x[0, 0:k + 2], x[1, 0:k + 2], '-b')  # plot new trajectory
#       ax_list[0].plot(pred_x[0, 1:], pred_x[1, 1:], 'g-')  # plot new prediction of trajectory
#       ax_list[1].plot(np.rad2deg(x[2, 0:k + 2]), 'b-')  # plot new steering angle
#       ax_list[1].plot(range(k + 1, k + model.N), np.rad2deg(pred_x[2, 1:]), 'g-')  # plot new prediction of steering angle
#       ax_list[2].plot(x[3, 0:k + 2], 'b-')  # plot new velocity
#       ax_list[2].plot(range(k + 1, k + model.N), pred_x[3, 1:], 'g-')  # plot new prediction of velocity
#       ax_list[3].plot(np.rad2deg(x[4, 0:k + 2]), 'b-')  # plot new heading angle
#       ax_list[3].plot(range(k + 1, k + model.N), np.rad2deg(pred_x[4, 1:]), 'g-')  # plot new prediction of heading angle
#       ax_list[4].step(range(0, k + 1), np.rad2deg(u[0, 0:k + 1]), 'b-')  # plot new steering rate
#       ax_list[4].step(range(k, k + model.N), np.rad2deg(pred_u[0, :]), 'g-')  # plot new prediction of steering rate
#       ax_list[5].step(range(0, k + 1), u[1, 0:k + 1], 'b-')  # plot new acceleration
#       ax_list[5].step(range(k, k + model.N), pred_u[1, :], 'g-')  # plot new prediction of acceleration
#
#       plt.pause(0.05)
#
=======
class ForcesproOptimizer(Optimizer):
    def __init__(self, configuration, init_values, predict_horizon):
        super(ForcesproOptimizer, self).__init__(configuration, init_values, predict_horizon)
        # self. = ... # add own attributes

    @staticmethod
    def equal_constraints():
        # We use an explicit RK4 integrator here to discretize continuous dynamics
        integrator_stepsize = 0.1
        return lambda z: forcespro.nlp.integrate(VehicleDynamics.KS_casadi, z[2:7], z[0:2],
                                                 integrator=forcespro.nlp.integrators.RK4,
                                                 stepsize=integrator_stepsize)

    def inequal_constraint(self):
        # Inequality constraints
        # z = [deltaDot,aLong,xPos,yPos,delta,v,psi]
        #  upper/lower variable bounds lb <= z <= ub
        #                     inputs      |  states
        #                deltaDot aLong    x    y   delta    v    psi
        # model.lb = np.array([-0.4, -11.5, -10, -30, -1.066, 0., -np.inf])  # 1.066 = 61 degree
        # model.ub = np.array([+0.4, +11.5, 100, 100, +1.066, 50.8, np.inf])
        #                     inputs      |  states
        #                       deltaDot            aLong    x              y                        delta             v      psi
        z_low_bound = np.array([self.deltav_min, -self.a_max, -np.inf, -np.inf, self.delta_min, self.v_min, -np.inf])  # 1.066 = 61 degree
        z_upper_bound = np.array([self.deltav_max, self.a_max, np.inf, np.inf, self.delta_max, self.v_max, np.inf])

        circles_distance_low_bound = np.tile(np.array([(self.radius_ego + self.radius_obstacle) ** 2]), 9)  # in all 9 constraints for circles distance
        circles_distance_upper_bound = np.tile(np.array([np.inf]), 9)
        return z_low_bound, z_upper_bound, circles_distance_low_bound, circles_distance_upper_bound

    def circles_distance_inequality(self, z, params):
        # three circles represent ego and obstacle vehicle.
        # The distance between circles of two vehicles should be larger than 2*r --> 3 * 3 = 9 constraints in all
        ego_circles_centers_tuple = compute_centers_of_approximation_circles(z[2], z[3], self.configuration.p.l, self.configuration.p.w, z[6])
        return ca.vertcat((ego_circles_centers_tuple[0][0] - params[4]) ** 2 + (ego_circles_centers_tuple[0][1] - params[5]) ** 2,
                          (ego_circles_centers_tuple[0][0] - params[6]) ** 2 + (ego_circles_centers_tuple[0][1] - params[7]) ** 2,
                          (ego_circles_centers_tuple[0][0] - params[8]) ** 2 + (ego_circles_centers_tuple[0][1] - params[9]) ** 2,
                          (ego_circles_centers_tuple[1][0] - params[4]) ** 2 + (ego_circles_centers_tuple[1][1] - params[5]) ** 2,
                          (ego_circles_centers_tuple[1][0] - params[6]) ** 2 + (ego_circles_centers_tuple[1][1] - params[7]) ** 2,
                          (ego_circles_centers_tuple[1][0] - params[8]) ** 2 + (ego_circles_centers_tuple[1][1] - params[9]) ** 2,
                          (ego_circles_centers_tuple[2][0] - params[4]) ** 2 + (ego_circles_centers_tuple[2][1] - params[5]) ** 2,
                          (ego_circles_centers_tuple[2][0] - params[6]) ** 2 + (ego_circles_centers_tuple[2][1] - params[7]) ** 2,
                          (ego_circles_centers_tuple[2][0] - params[8]) ** 2 + (ego_circles_centers_tuple[2][1] - params[9]) ** 2)

    def cost_function(self, z, params):
        """Least square costs on deviating from the path and on the inputs
        z = [deltaDot,aLong,xPos,yPos,delta,v,psi]
        params = [point on path x, point on path y, desired velocity, current orientation,
                  obstacle_center_circle_x, obstacle_center_circle_y,
                  obstacle_front_circle_x, obstacle_front_circle_y,
                  obstacle_rear_circle_x, obstacle_rear_circle_y]
        """
        return (self.weights_setting["weight_x"] * (z[2] - params[0]) ** 2  # costs on deviating on the path in x-direction
                + self.weights_setting["weight_y"] * (z[3] - params[1]) ** 2  # costs on deviating on the path in y-direction
                + self.weights_setting["weight_steering_angle"] * z[4] ** 2  # penalty on steering angle
                + self.weights_setting["weight_velocity"] * (z[5] - params[2]) ** 2  # penalty on velocity
                + self.weights_setting["weight_heading_angle"] * (z[6] - params[3]) ** 2  # penalty on heading angle
                + self.weights_setting["weight_velocity_steering_angle"] * z[0] ** 2  # penalty on input velocity of steering angle
                + self.weights_setting["weight_long_acceleration"] * z[1] ** 2)  # penalty on input longitudinal acceleration

    def cost_functionN(self, z, params):
        """Increased least square costs for last stage on deviating from the path and on the inputs F and phi
        z = [deltaDot,aLong,xPos,yPos,delta,v,psi]
        params = [point on path x, point on path y, desired velocity, current orientation,
                  obstacle_center_circle_x, obstacle_center_circle_y,
                  obstacle_front_circle_x, obstacle_front_circle_y,
                  obstacle_rear_circle_x, obstacle_rear_circle_y]
        """
        return (self.weights_setting["weight_x"] * 2 * (z[2] - params[0]) ** 2  # costs on deviating on the path in x-direction
                + self.weights_setting["weight_y"] * 2 * (z[3] - params[1]) ** 2  # costs on deviating on the path in y-direction
                + self.weights_setting["weight_steering_angle"] * 2 * z[4] ** 2  # penalty on steering angle
                + self.weights_setting["weight_velocity"] * 2 * (z[5] - params[2]) ** 2  # penalty on velocity
                + self.weights_setting["weight_heading_angle"] * 2 * (z[6] - params[3]) ** 2  # penalty on heading angle
                + self.weights_setting["weight_velocity_steering_angle"] * 2 * z[0] ** 2  # penalty on input velocity of steering angle
                + self.weights_setting["weight_long_acceleration"] * 2 * z[1] ** 2)  # penalty on input longitudinal acceleration

    def solver(self):
        """Generates and returns a FORCESPRO solver that calculates a path based on constraints and dynamics while minimizing an objective function"""
        # Model Definition
        # define problem dimensions
        model = forcespro.nlp.SymbolicModel()
        model.N = self.predict_horizon  # horizon length
        model.nvar = 7  # number of variables
        model.neq = 5  # number of equality constraints
        model.nh = 9  # number of inequality constraint functions --> 3*3 = 9 constraints for distance between circles
        model.npar = 10  # number of runtime parameters

        # define constraints
        model.eq = self.equal_constraints()
        model.ineq = self.circles_distance_inequality
        model.lb, model.ub, model.hl, model.hu = self.inequal_constraint()

        # define cost function
        model.objective = self.cost_function
        model.objectiveN = self.cost_functionN

        # Indices on LHS of dynamical constraint - for efficiency reasons, make sure the matrix E has structure [0 I] where I is the identity matrix.
        model.E = np.concatenate([np.zeros((5, 2)), np.eye(5)], axis=1)

        # Initial condition on vehicle states x
        model.xinitidx = range(2, 7)  # use this to specify on which variables initial conditions are imposed

        # Solver Generation
        # Set solver options
        codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
        codeoptions.maxit = 1500  # Maximum number of iterations
        codeoptions.printlevel = 1  # Use printlevel = 2 to print progress (but not for timings)
        codeoptions.optlevel = 0  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
        codeoptions.cleanup = False
        codeoptions.timing = 1
        codeoptions.nlp.hessian_approximation = 'bfgs'
        codeoptions.solvemethod = 'SQP_NLP'  # choose the solver method Sequential Quadratic Programming
        codeoptions.nlp.bfgs_init = 2.5 * np.identity(7)  # np.identity creat a square matrix with 1 in the diagonal line
        codeoptions.sqp_nlp.maxqps = 1  # maximum number of quadratic problems to be solved
        codeoptions.sqp_nlp.reg_hessian = 5e-9  # increase this if exitflag=-8    default: 5e-9
        # change this to your server or leave uncommented for using the standard embotech server at https://forces.embotech.com, codeoptions.server = 'https://forces.embotech.com'
        # Creates code for symbolic model formulation given above, then contacts server to generate new solver
        solver = model.generate_solver(options=codeoptions)

        return model, solver

    def optimize(self):
        # generate forcespro model and solver
        model, solver = self.solver()

        # Simulation
        sim_length = self.iter_length

        # Variables for storing simulation data
        x = np.zeros((5, sim_length + 1))  # states
        u = np.zeros((2, sim_length))  # inputs
        solve_time = np.zeros(sim_length)  # solve time

        # Set initial guess to start solver from
        x0i = np.array([0., self.init_acceleration, self.init_position[0], self.init_position[1], 0., self.init_velocity, self.init_orientation])
        # x0i = np.zeros((model.nvar, 1))  # model.nvar = 7 number of variables  shape of x0i = [7, 1]
        x0 = np.transpose(np.tile(x0i, (1, model.N)))  # horizon length 10,  shape of x0 = [10, 7]

        # xinit = np.transpose(np.array([self.init_position[0], self.init_position[1], 0., self.init_velocity, self.init_orientation]))  # Set initial states
        xinit = np.transpose(np.array([self.init_position[0], self.init_position[1], 0., self.init_velocity, self.init_orientation]))  # Set initial states
        # state x = [xPos,yPos,delta,v,psi]
        x[:, 0] = xinit

        problem = {"x0": x0,
                   "xinit": xinit}

        start_pred = np.reshape(problem["x0"], (7, model.N))  # first prediction corresponds to initial guess

        # Simulation
        time_start = time.time()

        for k in range(sim_length):
            print("k=", k)

            # Set initial condition
            problem["xinit"] = x[:, k]

            # Set runtime parameters (here, the next N points on the path)
            desired_velocity_array = np.ones(sim_length - model.N) * self.desired_velocity
            desired_velocity_last_N_steps = np.linspace(self.desired_velocity, 0, model.N)
            desired_velocity_all_steps = np.hstack((desired_velocity_array, desired_velocity_last_N_steps))
            desired_velocity_array = desired_velocity_all_steps[k + 1:k + 1 + model.N]
            next_path_points = self.resampled_path_points.T[:, k + 1:k + 1 + model.N]
            next_path_orientation = self.orientation[k + 1:k + 1 + model.N]
            while next_path_points.shape[1] != model.N:
                next_path_points = np.hstack((next_path_points, self.resampled_path_points[-1].reshape(2, -1)))
                next_path_orientation = np.hstack((next_path_orientation, self.orientation[-1]))
                desired_velocity_array = np.hstack((desired_velocity_array, desired_velocity_all_steps[-1]))
            # print("next_path_points", next_path_points)
            centers_obstacle = np.array([[self.obstacle_circles_centers_tuple[0][0]],
                                         [self.obstacle_circles_centers_tuple[0][1]],
                                         [self.obstacle_circles_centers_tuple[1][0]],
                                         [self.obstacle_circles_centers_tuple[1][1]],
                                         [self.obstacle_circles_centers_tuple[2][0]],
                                         [self.obstacle_circles_centers_tuple[2][1]]])

            params = np.vstack((next_path_points,
                                desired_velocity_array,
                                next_path_orientation,
                                np.tile(centers_obstacle, (1, model.N))))
            problem["all_parameters"] = np.reshape(np.transpose(params), (10 * model.N, 1))

            # Time to solve the NLP!
            output, exitflag, info = solver.solve(problem)
            print("exitflag = ", exitflag)

            # Make sure the solver has exited properly.
            assert exitflag == 1, "bad exitflag"  # does not succeed， return AssertionError: bad exitflag
            sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"
                             .format(info.it, info.solvetime))

            # Extract output
            temp = np.zeros((np.max(model.nvar), model.N))  # initialization temp.shape=7*N
            for i in range(0, model.N):
                try:
                    temp[:, i] = output['x{0:1d}'.format(i + 1)]  # {0:02d}
                except:
                    temp[:, i] = output['x{0:02d}'.format(i + 1)]
            pred_u = temp[0:2, :]  # N predicted inputs, 2*N
            pred_x = temp[2:7, :]  # N predicted states, 5*N
            # print("pred_x", pred_x[0:2, :])
            # Apply optimized input u of first stage to system and save simulation data
            if not self.configuration.noised:
                u[:, k] = pred_u[:, 0]
            else:
                noise_mean = np.array([0, 0])
                noise_std = np.array([0.08, 0.08])  # The larger standard deviation is, the wider range of noise is.
                noise = np.random.normal(noise_mean, noise_std, (2,))
                u[:, k] = pred_u[:, 0] + noise
            x[:, k + 1] = np.transpose(model.eq(np.concatenate((u[:, k], x[:, k]))))  # with k-th step 's x and u，update states at (k+1)-th
            solve_time[k] = info.solvetime

        time_end = time.time()
        time_per_iter = (time_end - time_start)/sim_length
        print("solve time per iteration", time_per_iter)
        x = np.delete(x, -1, axis=1)
        return x.T, u.T, solve_time

>>>>>>> develop

class CasadiOptimizer(Optimizer):
    def __init__(self, configuration, init_values, predict_horizon):
        super(CasadiOptimizer, self).__init__(configuration, init_values, predict_horizon)
        # self. = ... # add own attributes

    def equal_constraints(self, states, ref_states, controls, f):
        g = [ca.sqrt(((controls[1]) ** 2 + (states[3] * ((ca.tan(states[2]) * states[3]) / 2.578))) ** 2), states[:, 0] - ref_states[:, 0]]  # equal constraints
        for i in range(self.predict_horizon):
            x_next_ = f(states[:, i], controls[:, i])*self.delta_t + states[:, i]
            g.append(states[:, i+1]-x_next_)
        for i in range(self.predict_horizon+1):
            ego_circles_centers_tuple = compute_centers_of_approximation_circles(states[0,i], states[1,i], self.configuration.p.l, self.configuration.p.w, states[4,i])
            g.append(ca.sqrt((ego_circles_centers_tuple[0][0] - self.obstacle_circles_centers_tuple[0][0]) ** 2 + (ego_circles_centers_tuple[0][1] - self.obstacle_circles_centers_tuple[0][1]) ** 2)) # should be bigger than 2r
            g.append(ca.sqrt((ego_circles_centers_tuple[0][0] - self.obstacle_circles_centers_tuple[0][0]) ** 2 + (ego_circles_centers_tuple[0][1] - self.obstacle_circles_centers_tuple[0][1]) ** 2))
            g.append(ca.sqrt((ego_circles_centers_tuple[0][0] - self.obstacle_circles_centers_tuple[0][0]) ** 2 + (ego_circles_centers_tuple[0][1] - self.obstacle_circles_centers_tuple[0][1]) ** 2))
            g.append(ca.sqrt((ego_circles_centers_tuple[1][0] - self.obstacle_circles_centers_tuple[1][0]) ** 2 + (ego_circles_centers_tuple[1][1] - self.obstacle_circles_centers_tuple[1][1]) ** 2))
            g.append(ca.sqrt((ego_circles_centers_tuple[1][0] - self.obstacle_circles_centers_tuple[1][0]) ** 2 + (ego_circles_centers_tuple[1][1] - self.obstacle_circles_centers_tuple[1][1]) ** 2))
            g.append(ca.sqrt((ego_circles_centers_tuple[1][0] - self.obstacle_circles_centers_tuple[1][0]) ** 2 + (ego_circles_centers_tuple[1][1] - self.obstacle_circles_centers_tuple[1][1]) ** 2))
            g.append(ca.sqrt((ego_circles_centers_tuple[2][0] - self.obstacle_circles_centers_tuple[2][0]) ** 2 + (ego_circles_centers_tuple[2][1] - self.obstacle_circles_centers_tuple[2][1]) ** 2))
            g.append(ca.sqrt((ego_circles_centers_tuple[2][0] - self.obstacle_circles_centers_tuple[2][0]) ** 2 + (ego_circles_centers_tuple[2][1] - self.obstacle_circles_centers_tuple[2][1]) ** 2))
            g.append(ca.sqrt((ego_circles_centers_tuple[2][0] - self.obstacle_circles_centers_tuple[2][0]) ** 2 + (ego_circles_centers_tuple[2][1] - self.obstacle_circles_centers_tuple[2][1]) ** 2))
        return g

    def inequal_constraints(self):
        # states constraints
        lbg = []
        ubg = []
        lbg.append(0.0)
        ubg.append(self.a_max)
        # 5 states
        for _ in range(self.predict_horizon+1):
            lbg.append(0.0)
            lbg.append(0.0)
            lbg.append(0.0)
            lbg.append(0.0)
            lbg.append(0.0)
            ubg.append(0.0)
            ubg.append(0.0)
            ubg.append(0.0)
            ubg.append(0.0)
            ubg.append(0.0)
        # 9 comparisons between centers
        for _ in range(self.predict_horizon+1):
            lbg.append((self.radius_ego + self.radius_obstacle))
            lbg.append((self.radius_ego + self.radius_obstacle))
            lbg.append((self.radius_ego + self.radius_obstacle))
            lbg.append((self.radius_ego + self.radius_obstacle))
            lbg.append((self.radius_ego + self.radius_obstacle))
            lbg.append((self.radius_ego + self.radius_obstacle))
            lbg.append((self.radius_ego + self.radius_obstacle))
            lbg.append((self.radius_ego + self.radius_obstacle))
            lbg.append((self.radius_ego + self.radius_obstacle))
            ubg.append(np.inf)
            ubg.append(np.inf)
            ubg.append(np.inf)
            ubg.append(np.inf)
            ubg.append(np.inf)
            ubg.append(np.inf)
            ubg.append(np.inf)
            ubg.append(np.inf)
            ubg.append(np.inf)
        lbx = []
        ubx = []
        for _ in range(self.predict_horizon):
            lbx.append(self.deltav_min)
            ubx.append(self.deltav_max)
            lbx.append(-np.inf)
            ubx.append(self.a_max)
        for _ in range(self.predict_horizon+1):  # note that this is different with the method using structure
            lbx.append(-np.inf)
            # lbx.append(-1.2501)
            lbx.append(-np.inf)
            lbx.append(self.delta_min)
            lbx.append(self.v_min)
            lbx.append(-np.inf)
            ubx.append(np.inf)
            ubx.append(np.inf)
            ubx.append(self.delta_max)
            ubx.append(self.v_max)
            ubx.append(np.inf)
        return lbg, ubg, lbx, ubx

    def cost_function(self, states, controls, reference_states):
        obj = 0
        Q = np.array([[2.3, 0.0, 0.0, 0.0, 0.0], [0.0, 2.3, 0.0, 0.0, 0.0], [0.0, 0.0, 500, 0.0, 0.0], [0.0, 0.0, 0.0, 0.1, 0.0], [0.0, 0.0, 0.0, 0.0, 160]]) # for ZAM Over
        R = np.array([[0.8, 0.0], [0.0, 0.8]])
        P = np.diag([82.21, 82.21, 100.95, 0.01, 110.04]) 
        # cost
        for i in range(self.predict_horizon):
            # obj = obj + ca.mtimes([(X[:, i]-P[3:]).T, Q, X[:, i]-P[3:]]) + ca.mtimes([U[:, i].T, R, U[:, i]])
            obj = obj + (states[:, i] - reference_states[:, i+1]).T @ Q @ (states[:, i] - reference_states[:, i+1]) + controls[:, i].T @ R @ controls[:, i]
            + (states[:, -1] - reference_states[:, -1]).T @ P @ (states[:, -1] - reference_states[:, -1])
        return obj

    def solver(self):
        # define prediction horizon
        horizon = self.predict_horizon
        # set states variables
        sx = ca.SX.sym('sx')
        sy = ca.SX.sym('sy')
        delta = ca.SX.sym('delta')
        vel = ca.SX.sym('vel')
        Psi = ca.SX.sym('Psi')
        states = ca.vertcat(*[sx, sy, delta, vel, Psi])
        num_states = states.size()[0]
        # set control variables
        u0 = ca.SX.sym('u0')
        u1 = ca.SX.sym('u1')
        controls = ca.vertcat(*[u0, u1])
        num_controls = controls.size()[0]
        # get euqations from dynamics.py
        d = VehicleDynamics()
        rhs = d.KS_casadi(states, controls)
        f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])
        # for MPC
        U = ca.SX.sym('U', num_controls, horizon)
        X = ca.SX.sym('X', num_states, horizon + 1)
        U_ref = ca.SX.sym('U_ref', num_controls, horizon)
        X_ref = ca.SX.sym('X_ref', num_states, horizon+1)

        # define cost function
        cost_function = self.cost_function(X, U, X_ref)

        # states constrains
        g = self.equal_constraints(X, X_ref, U, f)

        opt_variables = ca.vertcat( ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))
        opt_params = ca.vertcat(ca.reshape(U_ref, -1, 1), ca.reshape(X_ref, -1, 1))

        nlp_prob = {'f': cost_function, 'x': opt_variables, 'p': opt_params, 'g': ca.vcat(g)}  # here also can use ca.vcat(g) or ca.vertcat(*g)
        opts_setting = {'ipopt.max_iter': 100, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-8, 'ipopt.acceptable_obj_change_tol': 1e-6, }

        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

        return solver, f

    def optimize(self):
        # model parameters
        num_states = 5
        num_controls = 2
        # get MPC optimizer from optimizer_casadi
        # mpc_obj = MPC_car(state_dim=num_states, T=0.1, N=N)
        lbg, ubg, lbx, ubx = self.inequal_constraints()

        t0 = 0.0
        # initial state
        init_state = np.array([self.init_position[0], self.init_position[1], 0.0, self.init_velocity, self.init_orientation]).reshape(-1, 1)
        current_state = init_state.copy()
        u0 = np.array([0.0, 0.0]*self.predict_horizon).reshape(-1, 2).T  # np.ones((N, 2)) # controls
        next_trajectories = np.tile(current_state.reshape(1, -1), self.predict_horizon+1).reshape(self.predict_horizon+1, -1)
        next_states = next_trajectories.copy()
        next_controls = np.zeros((self.predict_horizon, 2))
        x_c = []  # contains for the history of the state
        u_c = []
        t_c = [t0]  # for the time
        xx = []

        traj = []
        ref = []
        mpciter = 0
        start_time = time.time()
        index_t = []
        # simulation time

        for i in range(self.iter_length):
            xs = np.array([self.resampled_path_points[i, 0], self.resampled_path_points[i, 1], 0, self.desired_velocity, self.orientation[i]]).reshape(-1, 1)
            # set parameter
            c_p = np.concatenate((next_controls.reshape(-1, 1), next_trajectories.reshape(-1, 1)))
            init_control = np.concatenate((u0.T.reshape(-1, 1), next_states.T.reshape(-1, 1)))
            t_ = time.time()
            sol, f = self.solver()
            res = sol(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
            index_t.append(time.time() - t_)
            estimated_opt = res['x'].full() # the feedback is in the series [u0, x0, u1, x1, ...]
            u0 = estimated_opt[:int(num_controls*self.predict_horizon)].reshape(self.predict_horizon, num_controls).T # + np.random.normal(0,0.2,20).reshape(num_controls, self.predict_horizon)
            # + np.random.normal(0,0.1,20).reshape(num_controls, self.predict_horizon)# (n_controls, N)
            # + np.random.normal(0,1,20).reshape(num_controls, self.predict_horizon) # add a gaussian noise
            x_m = estimated_opt[int(num_controls*self.predict_horizon):].reshape(self.predict_horizon+1, num_states).T# [n_states, N]
            # + np.random.normal(0,1,20).reshape(num_states, N) # add a gaussian noise
            x_c.append(x_m.T)
            u_c.append(u0[:, 0])
            t_c.append(t0)
            t0, current_state, u0, next_states = self.shift_movement(t0, current_state, u0, x_m, f)
            current_state = ca.reshape(current_state, -1, 1)
            current_state = current_state.full()
            xx.append(current_state)
            next_trajectories, next_controls = self.desired_command_and_trajectory(i, current_state, self.predict_horizon)
            traj.append(current_state)
            ref.append(xs)
            mpciter = mpciter + 1
        t_v = np.array(index_t)
        print(t_v.mean())
        print((time.time() - start_time) / mpciter)

        u = np.array(u_c)
        # plot reference path and actual path
<<<<<<< HEAD
        traj_s = np.array(traj)
        traj_s = np.squeeze(traj_s)
        traj_r = np.array(ref)
=======
        traj_s = np.array(np.squeeze(traj))
        traj_s = np.insert(traj_s, 0, init_state.T, axis=0)
        traj_s = np.delete(traj_s, -1, axis=0)
        print(traj_s.shape)
        traj_r = np.array(np.squeeze(ref))
>>>>>>> develop
        plt.figure()
        plt.plot(traj_s[:, 0], traj_s[:, 1], label='real path')
        plt.plot(traj_r[:, 0], traj_r[:, 1], label='reference path')
        plt.plot()
        plt.xlabel('x position [m]')
        plt.ylabel('y position [m]')
        plt.legend(frameon=False, loc="lower right", fontsize='large')
        plt.savefig('path_zam')
        plt.show()

<<<<<<< HEAD
        ## animation in 3d
        #x, y, z = traj_s[:, 0].flatten(), traj_s[:, 1].flatten(), traj_r[:, 2].flatten()
        #helix = np.vstack((x, y, z))
#
        #def update_lines(num, dataLines, lines):
        #    for line, data in zip(lines, dataLines):
        #        line.set_data(data[0:2, :num])
        #        line.set_3d_properties(data[2, :num])
        #    return lines
#
        ## Attach 3D axis to the figure
        #fig = plt.figure()
        #ax = fig.add_subplot(projection='3d')
        #plt.rcParams['font.size'] = 15
        ## ax = fig.add_axes([0, 0, 1, 1], projection='3d')
        #data = [helix]
        #lines = [ax.plot(data[0][0, 0:1], data[0][1, 0:1], data[0][2, 0:1], '.', c='red', markersize=5)[0]]
#
        ## Set the axes properties
        #ax.set_xlim3d([-30, 10])
        #ax.set_xlabel('X')
#
        #ax.set_ylim3d([-1.0, 80])
        #ax.set_ylabel('Y')
#
        #ax.set_zlim3d([0, 1.2])
        #ax.set_zlabel('Z')
#
        #ax.set_title('Vertical Trajectory')
        #ax.plot(traj_r[:, 0].flatten(), traj_r[:, 1].flatten(), traj_r[:, 2].flatten(), 'b')  # x,y,z
        #ani = FuncAnimation(fig, update_lines, fargs=(data, lines), interval=10, blit=False)
        #plt.show()

        return traj_s
=======
        return traj_s, u, t_v
>>>>>>> develop

    def shift_movement(self, t0, x0, u, x_f, f):
        f_value = f(x0, u[:, 0])
        st = x0 + self.delta_t * f_value.full()
        t = t0 + self.delta_t
        u_end = np.concatenate((u[:, 1:], u[:, -1:]), axis=1)
        x_f = np.concatenate((x_f[:, 1:], x_f[:, -1:]), axis=1)

        return t, st, u_end.T, x_f

    def desired_command_and_trajectory(self, i, x0_, N_):
        x_ = x0_.reshape(1, -1).tolist()[0]
        u_ = []
        # states for the next N_ trajectories
        if i >= self.iter_length-self.predict_horizon:# assum iter=70, horizon=10 when i>=60
            for k in range(N_): 
                x_ref_ = self.resampled_path_points[i+k+1-(i-(self.iter_length-self.predict_horizon)+1), 0]
                y_ref_ = self.resampled_path_points[i+k+1-(i-(self.iter_length-self.predict_horizon)+1), 1]
                delta_ref_ = 0.0
                v_ref_ = self.desired_velocity
                psi_ref_ = self.orientation[i+k+1-(i-(self.iter_length-self.predict_horizon)+1)]
                x_.append(x_ref_)
                x_.append(y_ref_)
                x_.append(delta_ref_)
                x_.append(v_ref_)
                x_.append(psi_ref_)
                u_.append(0)
                u_.append(0)
        else: 
            for k in range(N_): 
                x_ref_ = self.resampled_path_points[i+k+1, 0]
                y_ref_ = self.resampled_path_points[i+k+1, 1]
                delta_ref_ = 0.0
                v_ref_ = self.desired_velocity
                psi_ref_ = self.orientation[i+k+1]
                x_.append(x_ref_)
                x_.append(y_ref_)
                x_.append(delta_ref_)
                x_.append(v_ref_)
                x_.append(psi_ref_)
                u_.append(0)
                u_.append(0)
        # return pose and command
        x_ = np.array(x_).reshape(N_+1, -1)
        u_ = np.array(u_).reshape(N_, -1)

        return x_, u_
