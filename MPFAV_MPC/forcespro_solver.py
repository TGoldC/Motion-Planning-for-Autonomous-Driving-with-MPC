import numpy as np
import casadi
import forcespro
import forcespro.nlp
import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.gridspec import GridSpec
import casadi
from dynamics import Vehicle_dynamics
from configuration import ReferencePath
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
import sys
sys.path.append("..")


def continuous_dynamics(x, u):
    """Defines dynamics of the car, i.e. equality constraints.
    parameters:
    state x = [xPos,yPos,delta,v,psi]
    input u = [deltaDot,aLong]
    """
    # calculate dx/dt
    p = parameters_vehicle2()
    l = p.a + p.b
    return casadi.vertcat(x[3] * casadi.cos(x[4]),
                          x[3] * casadi.sin(x[4]),
                          u[0],
                          u[1],
                          x[3] / l * casadi.tan(x[2]))


def obj(z, current_target):
    """Least square costs on deviating from the path and on the inputs
    z = [deltaDot,aLong,xPos,yPos,delta,v,psi]
    current_target = point on path that is to be headed for
    """
    return (200.0 * (z[2] - current_target[0]) ** 2  # costs on deviating on the path in x-direction
            + 200.0 * (z[3] - current_target[1]) ** 2  # costs on deviating on the path in y-direction
            + 0.1 * z[0] ** 2  # penalty on input velocity of steering angle
            + 0.1 * z[1] ** 2)  # penalty on input longitudinal acceleration


def objN(z,current_target):
    """Increased least square costs for last stage on deviating from the path and
    on the inputs F and phi
    z = [F,phi,xPos,yPos,v,theta,delta]
    current_target = point on path that is to be headed for
    """
    return (400.0*(z[2]-current_target[0])**2  # costs on deviating on the path in x-direction
            + 400.0*(z[3]-current_target[1])**2  # costs on deviating on the path in y-direction
            + 0.2*z[0]**2  # penalty on input velocity of steering angle
            + 0.2*z[1]**2)  # penalty on input longitudinal acceleration


def find_closest_point(points, ref_point):
    """Find the index of the closest point in points from the current car position
    points = array of points on path
    ref_point = current car position
    """
    num_points = points.shape[1]    # 这里的points是 2*N 的
    diff = np.transpose(points) - ref_point
    diff = np.transpose(diff)
    squared_diff = np.power(diff, 2)
    squared_dist = squared_diff[0, :] + squared_diff[1, :]
    return np.argmin(squared_dist)


def extract_next_path_points(path_points, pos, N):  # pos是car当前位置，先找到最近的index，然后返回其之后的N个点
    """Extract the next N points on the path for the next N stages starting from
    the current car position pos
    """
    idx = find_closest_point(path_points, pos)
    num_points = path_points.shape[1]  # 有多少个点
    num_ellipses = np.ceil((idx+N+1)/num_points)  # np.ceil() 向上取整，单一椭圆的话，就是1
    path_points = np.tile(path_points, (1, int(num_ellipses)))  # np.tile复制 1*int(num_ellipses) 多份，正常是1
    return path_points[:, idx+1:idx+N+1]  # return the next N points 2*N


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
    #                     inputs      |  states
    #                deltaDot aLong    x    y   delta    v    psi
    model.lb = np.array([-0.4, -11.5, -10, -20, -1.066, 0., -np.pi])     # 1.066 = 61 degree
    model.ub = np.array([+0.4, +11.5, 100, 60, +1.066, 50.8, np.pi])
    # z = [deltaDot,aLong,xPos,yPos,delta,v,psi]
    # Initial condition on vehicle states x
    model.xinitidx = range(2, 7)  # use this to specify on which variables initial conditions are imposed

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
    codeoptions.nlp.bfgs_init = 2.5 * np.identity(7)  # np.identity 创建方阵，对角线为1
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
    plt.clf()  # 用其所有轴清除整个当前图形
    gs = GridSpec(5, 2, figure=fig)  # 生成一个5*2的框格

    # Plot trajectory
    ax_pos = fig.add_subplot(gs[:, 0])
    l0, = ax_pos.plot(np.transpose(path_points[0, :]), np.transpose(path_points[1, :]), 'rx', markersize=5)  # 第一条线
    l1, = ax_pos.plot(xinit[0], xinit[1], 'bx')
    plt.title('Position')
    # plt.axis('equal')
    # for scenario 1
    plt.xlim([-20., 200.])
    plt.ylim([-3.5, 2.5])
    # for scenario 2
    # plt.xlim([-10., 60.])
    # plt.ylim([-20., 60.])
    plt.xlabel('x-coordinate')
    plt.ylabel('y-coordinate')
    l2, = ax_pos.plot(x[0, 0], x[1, 0], 'b-')
    l3, = ax_pos.plot(start_pred[2, :], start_pred[3, :], 'g-')  # start_pred is z
    ax_pos.legend([l0, l1, l2, l3], ['desired trajectory', 'init pos', 'car trajectory', 'predicted car traj.'],
                  loc='lower right')

    # Plot steering angle
    ax_delta = fig.add_subplot(5, 2, 2)
    plt.grid("both")  # 显示网格
    plt.title('steering angle')
    plt.xlim([0., sim_length - 1])
    plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.ub[4], model.ub[4]])), 'r:')
    plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.lb[4], model.lb[4]])), 'r:')
    ax_delta.plot(np.rad2deg(x[2, 0]), '-b')
    ax_delta.plot(np.rad2deg(start_pred[4, :]), 'g-')

    # Plot velocity
    ax_v = fig.add_subplot(5, 2, 4)
    plt.grid("both")
    plt.title('velocity')
    plt.ylim([0., 5.])
    plt.xlim([0., sim_length - 1])
    plt.plot([0, sim_length - 1], np.transpose([model.ub[5], model.ub[5]]), 'r:')
    plt.plot([0, sim_length - 1], np.transpose([model.lb[5], model.lb[5]]), 'r:')
    ax_v.plot(0., x[3, 0], 'b-')
    ax_v.plot(start_pred[5, :], 'g-')

    # Plot heading angle
    ax_psi = fig.add_subplot(5, 2, 6)
    plt.grid("both")
    plt.title('Heading angle')
    plt.xlim([0., sim_length - 1])
    plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.ub[6], model.ub[6]])), 'r:')
    plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.lb[6], model.lb[6]])), 'r:')
    ax_psi.plot(np.rad2deg(x[4, 0]), 'b-')
    ax_psi.plot(np.rad2deg(start_pred[6, :]), 'g-')

    # Plot velocity of steering angle
    ax_deltaDot = fig.add_subplot(5, 2, 8)
    plt.grid("both")
    plt.title('Velocity of steering angle')
    plt.xlim([0., sim_length - 1])
    plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.ub[0], model.ub[0]])), 'r:')
    plt.plot([0, sim_length - 1], np.rad2deg(np.transpose([model.lb[0], model.lb[0]])), 'r:')
    ax_deltaDot.step(0, np.rad2deg(u[0, 0]), 'b-')  # step函数可以认为是plot函数绘制阶梯图
    ax_deltaDot.step(range(model.N), start_pred[0, :], 'g-')

    # Plot longitudinal acceleration
    ax_aLong = fig.add_subplot(5, 2, 10)
    plt.grid("both")
    plt.title('Longitudinal acceleration')
    plt.xlim([0., sim_length - 1])
    plt.plot([0, sim_length - 1], np.transpose([model.ub[1], model.ub[1]]), 'r:')
    plt.plot([0, sim_length - 1], np.transpose([model.lb[1], model.lb[1]]), 'r:')
    ax_aLong.step(0., u[1, 0], 'b-')
    ax_aLong.step(range(model.N), start_pred[1, :], 'g-')

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
    fig = plt.gcf()  # 获取当前图表 get the current figures
    ax_list = fig.axes  # axes可以理解为 子区域

    # Delete old data in plot
    ax_list[0].get_lines().pop(-1).remove()  # remove old prediction of trajectory
    ax_list[0].get_lines().pop(-1).remove()  # remove old trajectory

    ax_list[1].get_lines().pop(-1).remove()  # remove old prediction of steering angle
    ax_list[1].get_lines().pop(-1).remove()  # remove old steering angle
    ax_list[2].get_lines().pop(-1).remove()  # remove old prediction of velocity
    ax_list[2].get_lines().pop(-1).remove()  # remove old velocity
    ax_list[3].get_lines().pop(-1).remove()  # remove old prediction of heading angle
    ax_list[3].get_lines().pop(-1).remove()  # remove old heading angle
    ax_list[4].get_lines().pop(-1).remove()  # remove old prediction of velocity of steering angle
    ax_list[4].get_lines().pop(-1).remove()  # remove old velocity of steering angle
    ax_list[5].get_lines().pop(-1).remove()  # remove old prediction of longitudinal acceleration
    ax_list[5].get_lines().pop(-1).remove()  # remove old longitudinal acceleration

    # Update plot with current simulation data
    ax_list[0].plot(x[0, 0:k + 2], x[1, 0:k + 2], '-b')  # plot new trajectory
    ax_list[0].plot(pred_x[0, 1:], pred_x[1, 1:], 'g-')  # plot new prediction of trajectory
    ax_list[1].plot(np.rad2deg(x[2, 0:k + 2]), 'b-')  # plot new steering angle
    ax_list[1].plot(range(k + 1, k + model.N), np.rad2deg(pred_x[2, 1:]), 'g-')  # plot new prediction of steering angle
    ax_list[2].plot(x[3, 0:k + 2], 'b-')  # plot new velocity
    ax_list[2].plot(range(k + 1, k + model.N), pred_x[3, 1:], 'g-')  # plot new prediction of velocity
    ax_list[3].plot(np.rad2deg(x[4, 0:k + 2]), 'b-')  # plot new heading angle
    ax_list[3].plot(range(k + 1, k + model.N), np.rad2deg(pred_x[4, 1:]), 'g-')  # plot new prediction of heading angle
    ax_list[4].step(range(0, k + 1), np.rad2deg(u[0, 0:k + 1]), 'b-')  # plot new steering rate
    ax_list[4].step(range(k, k + model.N),  np.rad2deg(pred_u[0, :]), 'g-')  # plot new prediction of steering rate
    ax_list[5].step(range(0, k + 1), u[1, 0:k + 1], 'b-')  # plot new acceleration
    ax_list[5].step(range(k, k + model.N), pred_u[1, :], 'g-')  # plot new prediction of acceleration

    plt.pause(0.05)


def main():
    # generate code for estimator
    model, solver = generate_pathplanner()

    # Simulation
    # ----------
    sim_length = 300  # simulate 8sec

    # Variables for storing simulation data
    x = np.zeros((5, sim_length + 1))  # states
    u = np.zeros((2, sim_length))  # inputs

    # Set initial guess to start solver from
    x0i = np.zeros((model.nvar, 1))  # model.nvar = 7 变量个数  shape of x0i = [7, 1]
    x0 = np.transpose(np.tile(x0i, (1, model.N)))  # # horizon length 10,  shape of x0 = [10, 7]

    # # generate reference path for car to follow-----> scenario 1
    path_points = ReferencePath(path_scenario="/home/xin/PycharmProjects/MPFAV_MPC/scenarios/",
                                id_scenario="ZAM_Tutorial_Urban-3_2.xml").reference_path.T  # transpose
    # generate reference path for car to follow-----> scenario 2
    # path_points = ReferencePath(path_scenario="/home/xin/PycharmProjects/MPFAV_MPC/scenarios/",
    #                             id_scenario="USA_Lanker-2_18_T-1.xml").reference_path.T  # transpose

    xinit = np.transpose(np.array([path_points[0, 0], path_points[1, 0], 0., 0., np.deg2rad(0)]))  # Set initial states
    # state x = [xPos,yPos,delta,v,psi]
    x[:, 0] = xinit

    problem = {"x0": x0,
               "xinit": xinit}

    start_pred = np.reshape(problem["x0"], (7, model.N))  # first prdicition corresponds to initial guess

    # generate plot with initial values
    createPlot(x, u, start_pred, sim_length, model, path_points, xinit)

    # Simulation
    for k in range(sim_length):
        print("k=", k)

        # Set initial condition
        problem["xinit"] = x[:, k]

        # Set runtime parameters (here, the next N points on the path)
        next_path_points = extract_next_path_points(path_points, x[0:2, k], model.N)
        # 返回离x[0:2, k]最近的点的之后的N个点 不包括本身，shape=2*N
        problem["all_parameters"] = np.reshape(np.transpose(next_path_points), (2 * model.N, 1))
        # shape = 2N * 1  【x y x y x y...】.T

        # Time to solve the NLP!
        output, exitflag, info = solver.solve(problem)
        print("exitflag = ", exitflag)

        # Make sure the solver has exited properly.
        assert exitflag == 1, "bad exitflag"  # 不成立， 返回AssertionError: bad exitflag
        sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"
                         .format(info.it, info.solvetime))

        # Extract output
        temp = np.zeros((np.max(model.nvar), model.N))  # 初始化 temp.shape=7*N
        for i in range(0, model.N):
            temp[:, i] = output['x{0:02d}'.format(i + 1)]
        pred_u = temp[0:2, :]   # inputs的N个预测值 2*N
        pred_x = temp[2:7, :]   # states的N个预测值 5*N

        # Apply optimized input u of first stage to system and save simulation data
        u[:, k] = pred_u[:, 0]
        x[:, k + 1] = np.transpose(model.eq(np.concatenate((u[:, k], x[:, k]))))  # 通过第k个step的state和u，来更新k+1时刻的state

        # plot results of current simulation step
        updatePlots(x, u, pred_x, pred_u, model, k)

        if k == sim_length - 1:
            fig = plt.gcf()
            ax_list = fig.axes
            ax_list[0].get_lines().pop(-1).remove()  # remove old prediction of trajectory
            ax_list[0].legend(['desired trajectory', 'init pos', 'car trajectory'], loc='lower right')
            plt.show()
        else:
            plt.draw()


if __name__ == "__main__":
    main()