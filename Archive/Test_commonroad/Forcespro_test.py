import sys
sys.path.insert(0, '/home/xin/MPFAV/Forcespro') # On Unix
import numpy as np
import casadi
import forcespro
import forcespro.nlp

# Example script for getting started with FORCESPRO NLP solver.

# This example solves an optimization problem for a car with the simple
# continuous-time, nonlinear dynamics (bicycle model):

#    dxPos/dt = v*cos(theta + beta)
#    dyPos/dt = v*sin(theta + beta)
#    dv/dt = F/m
#    dtheta/dt = v/l_r*sin(beta)
#    ddelta/dt = phi

#    with:
#    beta = arctan(l_r/(l_f + l_r)*tan(delta))

# where xPos,yPos are the position, v the velocity in heading angle theta
# of the car, and delta is the steering angle relative to the heading
# angle. The inputs are acceleration force F and steering rate phi. The
# physical constants m, l_r and l_f denote the car's mass and the distance
# from the car's center of gravity to the rear wheels and the front wheels.

# The car starts from standstill with a certain heading angle, and the
# optimization problem is to minimize the distance of the car's position
# to a given set of points on a path with respect to time.

# Quadratic costs for the acceleration force and steering rate are added to
# the objective to avoid excessive maneouvers.

# There are bounds on all variables except theta.

# Variables are collected stage-wise into

#     z = [F phi xPos yPos v theta delta].

# This example models the task as a MPC problem using the SQP method.

# See also FORCES_NLP

# (c) Embotech AG, Zurich, Switzerland, 2013-2021.


import sys
import numpy as np
import casadi
import forcespro
import forcespro.nlp
import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.gridspec import GridSpec
import casadi


def continuous_dynamics(x, u):
    """Defines dynamics of the car, i.e. equality constraints.
    parameters:
    state x = [xPos,yPos,v,theta,delta]
    input u = [F,phi]
    """
    # set physical constants
    l_r = 0.5  # distance rear wheels to center of gravitiy of the car
    l_f = 0.5  # distance front wheels to center of gravitiy of the car
    m = 1.0  # mass of the car

    # set parameters
    beta = casadi.arctan(l_r / (l_f + l_r) * casadi.tan(x[4]))

    # calculate dx/dt
    return casadi.vertcat(x[2] * casadi.cos(x[3] + beta),  # dxPos/dt = v*cos(theta+beta)
                          x[2] * casadi.sin(x[3] + beta),  # dyPos/dt = v*sin(theta+beta)
                          u[0] / m,  # dv/dt = F/m
                          x[2] / l_r * casadi.sin(beta),  # dtheta/dt = v/l_r*sin(beta)
                          u[1])  # ddelta/dt = phi


def obj(z, current_target):
    """Least square costs on deviating from the path and on the inputs F and phi
    z = [F,phi,xPos,yPos,v,theta,delta]
    current_target = point on path that is to be headed for
    """
    return (100.0 * (z[2] - current_target[0]) ** 2  # costs on deviating on the
            #                                              path in x-direction
            + 100.0 * (z[3] - current_target[1]) ** 2  # costs on deviating on the
            #                                               path in y-direction
            + 0.1 * z[0] ** 2  # penalty on input F
            + 0.1 * z[1] ** 2)  # penalty on input phi


def objN(z, current_target):
    """Increased least square costs for last stage on deviating from the path and
    on the inputs F and phi
    z = [F,phi,xPos,yPos,v,theta,delta]
    current_target = point on path that is to be headed for
    """
    return (200.0 * (z[2] - current_target[0]) ** 2  # costs on deviating on the
            #                                              path in x-direction
            + 200.0 * (z[3] - current_target[1]) ** 2  # costs on deviating on the
            #                                               path in y-direction
            + 0.2 * z[0] ** 2  # penalty on input F
            + 0.2 * z[1] ** 2)  # penalty on input phi


def calc_points_on_ellipse(num_points):
    """Desired trajectory on ellipoid represented by 2D points"""
    dT = 2 * np.pi / num_points
    t = np.arange(dT, (num_points + 1) * dT, dT)
    path_points = np.array([0.5 * np.cos(t),
                            2.0 * np.sin(t)])
    return path_points


def find_closest_point(points, ref_point):
    """Find the index of the closest point in points from the current car position
    points = array of points on path
    ref_point = current car position
    """
    num_points = points.shape[1]
    diff = np.transpose(points) - ref_point
    diff = np.transpose(diff)
    squared_diff = np.power(diff, 2)
    squared_dist = squared_diff[0, :] + squared_diff[1, :]
    return np.argmin(squared_dist)


def extract_next_path_points(path_points, pos, N):
    """Extract the next N points on the path for the next N stages starting from
    the current car position pos
    """
    idx = find_closest_point(path_points, pos)
    num_points = path_points.shape[1]
    num_ellipses = np.ceil((idx + N + 1) / num_points)
    path_points = np.tile(path_points, (1, int(num_ellipses)))
    return path_points[:, idx + 1:idx + N + 1]


def generate_pathplanner():
    """Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function
    """
    # Model Definition
    # ----------------

    # Problem dimensions
    model = forcespro.nlp.SymbolicModel()
    model.N = 10  # horizon length
    model.nvar = 7  # number of variables
    model.neq = 5  # number of equality constraints
    model.npar = 2  # number of runtime parameters

    # Objective function
    model.objective = obj
    model.objectiveN = objN  # increased costs for the last stage
    # The function must be able to handle symbolic evaluation,
    # by passing in CasADi symbols. This means certain numpy funcions are not
    # available.

    # We use an explicit RK4 integrator here to discretize continuous dynamics
    integrator_stepsize = 0.1
    model.eq = lambda z: forcespro.nlp.integrate(continuous_dynamics, z[2:7], z[0:2],
                                                 integrator=forcespro.nlp.integrators.RK4,
                                                 stepsize=integrator_stepsize)
    # Indices on LHS of dynamical constraint - for efficiency reasons, make
    # sure the matrix E has structure [0 I] where I is the identity matrix.
    model.E = np.concatenate([np.zeros((5, 2)), np.eye(5)], axis=1)

    # Inequality constraints
    #  upper/lower variable bounds lb <= z <= ub
    #                     inputs                 |  states
    #                     F          phi            x    y     v    theta         delta
    model.lb = np.array([-5., np.deg2rad(-90.), -2., -2., 0., -np.inf, -0.48 * np.pi])
    model.ub = np.array([+5., np.deg2rad(+90.), 2., 2., 4., np.inf, 0.48 * np.pi])

    # Initial condition on vehicle states x
    model.xinitidx = range(2, 7)  # use this to specify on which variables initial conditions
    # are imposed

    # Solver generation
    # -----------------

    # Set solver options
    codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
    codeoptions.maxit = 200  # Maximum number of iterations
    codeoptions.printlevel = 0  # Use printlevel = 2 to print progress (but
    #                             not for timings)
    codeoptions.optlevel = 0  # 0 no optimization, 1 optimize for size,
    #                             2 optimize for speed, 3 optimize for size & speed
    codeoptions.cleanup = False
    codeoptions.timing = 1
    codeoptions.nlp.hessian_approximation = 'bfgs'
    codeoptions.solvemethod = 'SQP_NLP'  # choose the solver method Sequential
    #                              Quadratic Programming
    codeoptions.nlp.bfgs_init = 2.5 * np.identity(7)
    codeoptions.sqp_nlp.maxqps = 1  # maximum number of quadratic problems to be solved
    codeoptions.sqp_nlp.reg_hessian = 5e-9  # increase this if exitflag=-8
    # change this to your server or leave uncommented for using the
    # standard embotech server at https://forces.embotech.com
    # codeoptions.server = 'https://forces.embotech.com'

    # Creates code for symbolic model formulation given above, then contacts
    # server to generate new solver
    solver = model.generate_solver(options=codeoptions)

    return model, solver


def createPlot(x, u, start_pred, sim_length, model, path_points, xinit):
    """Creates a plot and adds the initial data provided by the arguments"""

    # Create empty plot
    fig = plt.figure()
    plt.clf()
    gs = GridSpec(5, 2, figure=fig)

    # Plot trajectory
    ax_pos = fig.add_subplot(gs[:, 0])
    l0, = ax_pos.plot(np.transpose(path_points[0, :]), np.transpose(path_points[1, :]), 'rx')
    l1, = ax_pos.plot(xinit[0], xinit[1], 'bx')
    plt.title('Position')
    # plt.axis('equal')
    plt.xlim([-1., 1.])
    plt.ylim([-3.5, 2.5])
    plt.xlabel('x-coordinate')
    plt.ylabel('y-coordinate')
    l2, = ax_pos.plot(x[0, 0], x[1, 0], 'b-')
    l3, = ax_pos.plot(start_pred[2, :], start_pred[3, :], 'g-')
    ax_pos.legend([l0, l1, l2, l3], ['desired trajectory', 'init pos', 'car trajectory', \
                                     'predicted car traj.'], loc='lower right')

    # Plot velocity
    ax_vel = fig.add_subplot(5, 2, 2)
    plt.grid("both")
    plt.title('Velocity')
    plt.xlim([0., sim_length - 1])
    plt.plot([0, sim_length - 1], np.transpose([model.ub[4], model.ub[4]]), 'r:')
    plt.plot([0, sim_length - 1], np.transpose([model.lb[4], model.lb[4]]), 'r:')
    ax_vel.plot(0., x[2, 0], '-b')
    ax_vel.plot(start_pred[4, :], 'g-')

    # Plot heading angle
    ax_theta = fig.add_subplot(5, 2, 4)
    plt.grid("both")
    plt.title('Heading angle')
    plt.ylim([0., 900.])
    plt.xlim([0., sim_length - 1])
    plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.ub[5], model.ub[5]])), 'r:')
    plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.lb[5], model.lb[5]])), 'r:')
    ax_theta.plot(np.rad2deg(x[3, 0]), 'b-')
    ax_theta.plot(np.rad2deg(start_pred[5, :]), 'g-')

    # Plot steering angle
    ax_delta = fig.add_subplot(5, 2, 6)
    plt.grid("both")
    plt.title('Steering angle')
    plt.xlim([0., sim_length - 1])
    plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.ub[6], model.ub[6]])), 'r:')
    plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.lb[6], model.lb[6]])), 'r:')
    ax_delta.plot(np.rad2deg(x[4, 0]), 'b-')
    ax_delta.plot(np.rad2deg(start_pred[6, :]), 'g-')

    # Plot acceleration force
    ax_F = fig.add_subplot(5, 2, 8)
    plt.grid("both")
    plt.title('Acceleration force')
    plt.xlim([0., sim_length - 1])
    plt.plot([0, sim_length - 1], np.transpose([model.ub[0], model.ub[0]]), 'r:')
    plt.plot([0, sim_length - 1], np.transpose([model.lb[0], model.lb[0]]), 'r:')
    ax_F.step(0, u[0, 0], 'b-')
    ax_F.step(range(model.N), start_pred[0, :], 'g-')

    # Plot steering rate
    ax_phi = fig.add_subplot(5, 2, 10)
    plt.grid("both")
    plt.title('Steering rate')
    plt.xlim([0., sim_length - 1])
    plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.ub[1], model.ub[1]])), 'r:')
    plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.lb[1], model.lb[1]])), 'r:')
    ax_phi.step(0., np.rad2deg(u[1, 0]), 'b-')
    ax_phi.step(range(model.N), start_pred[1, :], 'g-')

    plt.tight_layout()

    # Make plot fullscreen. Comment out if platform dependent errors occur.
    mng = plt.get_current_fig_manager()


def updatePlots(x, u, pred_x, pred_u, model, k):
    """Deletes old data sets in the current plot and adds the new data sets
    given by the arguments x, u and predicted_z to the plot.
    x: matrix consisting of a set of state column vectors
    u: matrix consisting of a set of input column vectors
    pred_x: predictions for the next N state vectors
    pred_u: predictions for the next N input vectors
    model: model struct required for the code generation of FORCESPRO
    k: simulation step
    """
    fig = plt.gcf()
    ax_list = fig.axes

    # Delete old data in plot
    ax_list[0].get_lines().pop(-1).remove()  # remove old prediction of trajectory
    ax_list[0].get_lines().pop(-1).remove()  # remove old trajectory

    ax_list[1].get_lines().pop(-1).remove()  # remove old prediction of velocity
    ax_list[1].get_lines().pop(-1).remove()  # remove old velocity
    ax_list[2].get_lines().pop(-1).remove()  # remove old prediction of heading angle
    ax_list[2].get_lines().pop(-1).remove()  # remove old heading angle
    ax_list[3].get_lines().pop(-1).remove()  # remove old prediction of steering angle
    ax_list[3].get_lines().pop(-1).remove()  # remove old steering angle
    ax_list[4].get_lines().pop(-1).remove()  # remove old prediction of acceleration force
    ax_list[4].get_lines().pop(-1).remove()  # remove old acceleration force
    ax_list[5].get_lines().pop(-1).remove()  # remove old prediction of steering rate
    ax_list[5].get_lines().pop(-1).remove()  # remove old steering rate

    # Update plot with current simulation data
    ax_list[0].plot(x[0, 0:k + 2], x[1, 0:k + 2], '-b')  # plot new trajectory
    ax_list[0].plot(pred_x[0, 1:], pred_x[1, 1:], 'g-')  # plot new prediction of trajectory
    ax_list[1].plot(x[2, 0:k + 2], 'b-')  # plot new velocity
    ax_list[1].plot(range(k + 1, k + model.N), pred_x[2, 1:], 'g-')  # plot new prediction of velocity
    ax_list[2].plot(np.rad2deg(x[3, 0:k + 2]), 'b-')  # plot new heading angle
    ax_list[2].plot(range(k + 1, k + model.N), np.rad2deg(pred_x[3, 1:]), 'g-')  # plot new prediciton of heading angle
    ax_list[3].plot(np.rad2deg(x[4, 0:k + 2]), 'b-')  # plot new steering angle
    ax_list[3].plot(range(k + 1, k + model.N), np.rad2deg(pred_x[4, 1:]), 'g-')  # plot new prediction of steering angle
    ax_list[4].step(range(0, k + 1), u[0, 0:k + 1], 'b-')  # plot new acceleration force
    ax_list[4].step(range(k, k + model.N), pred_u[0, :], 'g-')  # plot new prediction of acceleration force
    ax_list[5].step(range(0, k + 1), np.rad2deg(u[1, 0:k + 1]), 'b-')  # plot new steering rate
    ax_list[5].step(range(k, k + model.N), np.rad2deg(pred_u[1, :]), 'g-')  # plot new prediction of steering rate

    plt.pause(0.05)


def main():
    # generate code for estimator
    model, solver = generate_pathplanner()

    # Simulation
    # ----------
    sim_length = 80  # simulate 8sec

    # Variables for storing simulation data
    x = np.zeros((5, sim_length + 1))  # states
    u = np.zeros((2, sim_length))  # inputs

    # Set initial guess to start solver from
    x0i = np.zeros((model.nvar, 1))
    x0 = np.transpose(np.tile(x0i, (1, model.N)))
    # Set initial condition
    xinit = np.transpose(np.array([0.8, 0., 0., np.deg2rad(90), 0.]))
    x[:, 0] = xinit

    problem = {"x0": x0,
               "xinit": xinit}

    # Create 2D points on ellipse which the car is supposed to follow
    num_points = 80
    path_points = calc_points_on_ellipse(num_points)

    start_pred = np.reshape(problem["x0"], (7, model.N))  # first prdicition corresponds to initial guess

    # generate plot with initial values
    createPlot(x, u, start_pred, sim_length, model, path_points, xinit)

    # Simulation
    for k in range(sim_length):

        # Set initial condition
        problem["xinit"] = x[:, k]

        # Set runtime parameters (here, the next N points on the path)
        next_path_points = extract_next_path_points(path_points, x[0:2, k], model.N)
        problem["all_parameters"] = np.reshape(np.transpose(next_path_points), \
                                               (2 * model.N, 1))

        # Time to solve the NLP!
        output, exitflag, info = solver.solve(problem)

        # Make sure the solver has exited properly.
        assert exitflag == 1, "bad exitflag"
        sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n" \
                         .format(info.it, info.solvetime))

        # Extract output
        temp = np.zeros((np.max(model.nvar), model.N))
        for i in range(0, model.N):
            temp[:, i] = output['x{0:02d}'.format(i + 1)]
        pred_u = temp[0:2, :]
        pred_x = temp[2:7, :]

        # Apply optimized input u of first stage to system and save simulation data
        u[:, k] = pred_u[:, 0]
        x[:, k + 1] = np.transpose(model.eq(np.concatenate((u[:, k], x[:, k]))))

        # plot results of current simulation step
        updatePlots(x, u, pred_x, pred_u, model, k)

        if k == sim_length - 1:
            fig = plt.gcf()
            ax_list = fig.axes
            ax_list[0].get_lines().pop(-1).remove()  # remove old prediction of trajectory
            ax_list[0].legend(['desired trajectory', 'init pos', 'car trajectory'], \
                              loc='lower right')
            plt.show()
        else:
            plt.draw()


if __name__ == "__main__":
    main()


# """
# Description of problem:  ----> obstacle avoidance
# x_dot = v * cos(theta)    # x-position of car
# y_dot  = v * sin(theta)   # y-position of car
# v_dot = F / m             # v is the linear velocity    mass m = 1 kg
# theta_dot = s / L         # theta is the heading angle   wheel base L = 1m
#
# control input: acceleration F and steering torque s
#
# This is a NLP problem.
# Define stage variable z by stacking the input and differential state variables.
# z = [F, s, x, y, v, theta].T
# """
#
#
# # define vehicle model
# def continuous_dynamics(x, u):
#     m, length = 1, 1
#     return np.array([x[2] * casadi.cos(x[3]),  # x_dot = v * cos(theta)
#                      x[2] * casadi.sin(x[3]),  # y_dot  = v * sin(theta)
#                      u[0] / m,                 # v_dot = F / m
#                      u[1] / length])           # theta_dot = s / L
#
#
# model = forcespro.nlp.SymbolicModel()
#
# model.N = 50    # horizon length
# model.nvar = 6  # number of stage variables
# model.neq = 4   # number of equality constraints
# model.nh = 2    # number of nonlinear inequality constraints
# model.npar = 0  # number of runtime parameters
#
# # define the object function
# # maximize progress in the 𝑦 direction; with quadratic penalties on the inputs 𝐹 and 𝑠
# # --> f(z) = -100 * y + 0.1 * square(z1) + 0.01 * square(z2)
# model.objective = lambda z: -100 * z[3] + 0.1 * z[0]**2 + 0.01 * z[1]**2
#
# # Use an explicit Runge-Kutta integrator of order 4 integrator here to discretize continuous dynamics
# integrator_stepsize = 0.1
# model.eq = lambda z: forcespro.nlp.integrate(continuous_dynamics, z[2:6], z[0:2],
#                                              integrator=forcespro.nlp.integrators.RK4, stepsize=integrator_stepsize)
#
# # Use the selection matrix 𝐸 to link the stage variables 𝑧𝑖 with the states 𝑥 and inputs 𝑢 of the
# # continuous dynamics function
# model.E = np.concatenate([np.zeros((4, 2)), np.eye(4)], axis=1)
#
# # define inequalities constraints
# # 1<= x**2 + y**2 <= 9
# # 0.95**2 <= (x+2)**2 + (y-2.5)**2
# model.ineq = lambda z: np.array([z[2] ** 2 + z[3] ** 2, (z[2] + 2) ** 2 + (z[3] - 2.5) ** 2])
# model.hu = np.array([9, +np.inf])
# model.hl = np.array([1, 0.95 ** 2])
#
# # constraints
# # -5<=F<= 5 N;   -1 <=s<= 1 Nm;  -3<=x<=0 m; 0<=y<=3 m; 0<=v<=2 m/s; 0<=theta<=pi rad
# # init constraints: x_init = -2m; y_init = 0m; v_init=0m/s; theta_init=2.0944 rad
# # final constraints: v_init = 0m/s; theta_init = 0 rad
# #                    F    s   x  y  v   theta
# model.lb = np.array([-5, -1, -3, 0, 0,      0])
# model.ub = np.array([+5, +1, 0,  3, 2, +np.inf])
#
# xinit = np.array([-2, 0, 0, np.deg2rad(120)])
# model.xinitidx = range(2, 6)
# xfinal = np.array([0, np.deg2rad(0)])
# model.xfinalidx = range(4, 6)
#
# # set solver options
# codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
# codeoptions.maxit = 200     # Maximum number of iterations
# codeoptions.printlevel = 2  # # Use printlevel = 2 to print progress (but not for timings)
# codeoptions.optlevel = 0    # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
# solver = model.generate_solver(codeoptions)
#
# # Set initial guess to start solver from:
# x0i = (model.lb + model.ub) / 3.0
# x0 = np.transpose(np.tile(x0i, (1, model.N))) # np.tile把x0i的矩阵 复制(1, model.N)个形状
# problem = {"x0": x0,
#            "xinit": xinit,
#            "xfinal": xfinal}
#
# # solve the NLP!
# output, exitflag, info = solver.solve(problem)
# # Make sure the solver has exited properly.
# assert exitflag == 1, "bad exitflag"
# print("FORCES took {} iterations and {} seconds to solve the problem.".format(info.it, info.solvetime))
#
#


# # load parameters
# p = parameters_vehicle2()
# g = 9.81  # [m/s^2]
#
# # set options --------------------------------------------------------------
# tStart = 0  # start time
# tFinal = 1  # start time
#
# delta0 = 0
# vel0 = 15
# Psi0 = 0
# dotPsi0 = 0
# beta0 = 0
# sy0 = 0
# initialState = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]  # initial state for simulation
# x0_KS = init_ks(initialState)  # initial state for kinematic single-track model
#
#
# def func_ks(x, t, u, p):
#     """
#     #x1 = x-position in a global coordinate system
#     #x2 = y-position in a global coordinate system
#     #x3 = steering angle of front wheels
#     #x4 = velocity in x-direction
#     #x5 = yaw angle
#
#     #u1 = steering angle velocity of front wheels
#     #u2 = longitudinal acceleration"""
#     f = vehicle_dynamics_ks(x, u, p)
#     return f
#
# def forcespro_solver():
#     """Variables are collected stage-wise into
#         z = [u1 u2 xPos yPos delta v phi].
# """
#     # define the model
#     model = forcespro.nlp.SymbolicModel()
#     model.N = 10  # horizon length
#     model.nvar = 7  # number of variables
#     model.neq = 5  # number of equality constraints
#     model.nh = 2  # number of inequality constraint functions
#     model.npar = 2  # number of runtime parameters
#
#     model.objective = lambda z: 100 * casadi.fabs(z[2] -0.) \
#                                 + 100 * casadi.fabs(z[3] - 3.) \
#                                 + 0.1 * z[0]**2 + 0.01 * z[1]**2
#     integrator_stepsize = 0.1
#     model.eq = lambda z: forcespro.nlp.integrate(func_ks, z[2:7], z[0:2],
#                                                  integrator=forcespro.nlp.integrators.RK4,
#                                                  stepsize=integrator_stepsize)
#     p = parameters_vehicle2()
#     model.lb = np.array([p.steering.v_min, - p.longitudinal.a_max, -3., 0., p.steering.min, p.longitudinal.v_min, np.deg2rad(-40.)])
#     model.ub = np.array([p.steering.v_max, p.longitudinal.a_max, 0., 3., p.steering.max, p.longitudinal.v_max, np.deg2rad(+40.)])
