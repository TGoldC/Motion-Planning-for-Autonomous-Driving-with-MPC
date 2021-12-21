import numpy as np
import casadi
import forcespro
import forcespro.nlp
import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.gridspec import GridSpec
import casadi
from dynamics import Vehicle_dynamics
from reference_path import ReferencePath
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from commonroad.common.file_reader import CommonRoadFileReader
import sys
sys.path.append("..")


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
    # plt.xlim([-20., 200.])
    # plt.ylim([-3.5, 2.5])
    # for scenario 2
    plt.xlim([-10., 60.])
    plt.ylim([-20., 60.])
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
    # define the scenario and planning problem
    # # generate reference path for car to follow-----> scenario 1
    # path_points = ReferencePath(path_scenario="/home/xin/PycharmProjects/MPFAV_MPC/scenarios/",
    #                             id_scenario="ZAM_Tutorial_Urban-3_2.xml").reference_path.T  # transpose
    # generate reference path for car to follow-----> scenario 2
    path_scenario = "/home/xin/PycharmProjects/MPFAV_MPC/scenarios/"
    id_scenario = "USA_Lanker-2_18_T-1.xml"
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario + id_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    reference_path_instance = ReferencePath(scenario, planning_problem)


    path_points = reference_path_instance.reference_path.T  # transpose
    init_orientation = reference_path_instance.init_orientation
    desired_velocity = reference_path_instance.desired_velocity

    # generate code for estimator
    model, solver = generate_pathplanner()

    # Simulation
    # ----------
    reference_path_distance = reference_path_instance.accumulated_distance_in_reference_path[-1]
    # print(reference_path_instance.accumulated_distance_in_reference_path)
    desired_time = reference_path_distance / desired_velocity
    sim_length = int(desired_time/0.1)
    # sim_length = 300  # simulate 30sec

    # Variables for storing simulation data
    x = np.zeros((5, sim_length + 1))  # states
    u = np.zeros((2, sim_length))  # inputs

    # Set initial guess to start solver from
    x0i = np.zeros((model.nvar, 1))  # model.nvar = 7 变量个数  shape of x0i = [7, 1]
    x0 = np.transpose(np.tile(x0i, (1, model.N)))  # # horizon length 10,  shape of x0 = [10, 7]

    xinit = np.transpose(np.array([path_points[0, 0], path_points[1, 0], 0., 0., init_orientation]))  # Set initial states
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

        # Objective function   因为object function 有变量是随着时间的变化而变化的，所以要写在 main里的for 循环中
        # model.objective = obj
        print("current desired distance", desired_velocity*k*0.1)
        print("desired_index", reference_path_instance.find_nearest_point_in_reference_path(k*0.1))
        currrent_target = path_points.T[reference_path_instance.find_nearest_point_in_reference_path(k*0.1)+6]
        model.objective = lambda z, currrent_target=currrent_target: (200.0 * (z[2] - currrent_target[0]) ** 2  # costs on deviating on the path in x-direction
                                        + 200.0 * (z[3] - currrent_target[1]) ** 2  # costs on deviating on the path in y-direction
                                        + 0.1 * z[4] ** 2  # penalty on steering angle
                                        + 200 * (z[5] - desired_velocity) ** 2  # penalty on velocity
                                        + 0.1 * z[6] ** 2
                                        + 0.1 * z[0] ** 2  # penalty on input velocity of steering angle
                                        + 0.1 * z[1] ** 2)  # penalty on input longitudinal acceleration
        # model.objectiveN = objN  # increased costs for the last stage
        model.objectiveN = lambda z, currrent_target=currrent_target: (400.0 * (z[2] - currrent_target[0]) ** 2  # costs on deviating on the path in x-direction
                                      + 400.0 * (z[3] - currrent_target[1]) ** 2  # costs on deviating on the path in y-direction
                                      + 0.2 * z[4] ** 2  # penalty on steering angle
                                      + 200 * (z[5] - desired_velocity) ** 2  # penalty on velocity
                                      + 0.2 * z[6] ** 2
                                      + 0.2 * z[0] ** 2  # penalty on input velocity of steering angle
                                      + 0.2 * z[1] ** 2)  # penalty on input longitudinal acceleration
        # The function must be able to handle symbolic evaluation,
        # by passing in CasADi symbols. This means certain numpy funcions are not available.

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
