import numpy as np
import matplotlib.pyplot as plt
from configuration import Vehicle_dynamics
from commonroad.common.file_reader import CommonRoadFileReader
from mpc_planner import *
from optimizer_forcespro import *
import sys
sys.path.append("..")


def main():
    # define the scenario and planning problem
    path_scenario = "/home/xin/PycharmProjects/forcespro_mpc/scenarios/"
    id_scenario = "USA_Lanker-2_18_T-1.xml"  # "ZAM_Tutorial_Urban-3_2.xml"
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario + id_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
    MPC_Planner_instance = MPC_Planner(scenario, planning_problem)
    # reference path from Route Planner
    path_points = MPC_Planner_instance.reference_path.T  # transpose
    # resample the original reference path
    resampled_path_points, iter_length, sim_time = MPC_Planner_instance.resample_reference_path()

    # get init values and desired velocity
    init_position, init_velocity, init_acceleration, init_orientation = MPC_Planner_instance.get_init_value()
    desired_velocity, delta_t = MPC_Planner_instance.get_desired_velocity_and_delta_t()

    # generate forcespro model and solver
    solver_forcespro_instance = SolverForcespro()
    model, solver = solver_forcespro_instance.generate_pathplanner()

    # Simulation
    # ----------

    # sim_length = iter_length  # simulate 30sec
    sim_length = resampled_path_points.shape[0]  # 167  N*2

    # Variables for storing simulation data
    x = np.zeros((5, sim_length + 1))  # states
    u = np.zeros((2, sim_length))  # inputs

    # Set initial guess to start solver from
    x0i = np.zeros((model.nvar, 1))  # model.nvar = 7 变量个数  shape of x0i = [7, 1]
    x0 = np.transpose(np.tile(x0i, (1, model.N)))  # horizon length 10,  shape of x0 = [10, 7]

    xinit = np.transpose(np.array([init_position[0], init_position[1], 0., init_velocity, init_orientation]))  # Set initial states
    # state x = [xPos,yPos,delta,v,psi]
    x[:, 0] = xinit

    problem = {"x0": x0,
               "xinit": xinit}

    start_pred = np.reshape(problem["x0"], (7, model.N))  # first prdicition corresponds to initial guess

    # generate plot with initial values
    solver_forcespro_instance.createPlot(x, u, start_pred, sim_length, model, path_points, xinit)

    # Simulation
    for k in range(sim_length):
        print("k=", k)

        # Objective function   因为object function 有变量是随着时间的变化而变化的，所以要写在 main里的for 循环中
        # model.objective = obj
        # current_position = resampled_path_points[k, :]
        # cost_func = CostFuncForcespro()
        # model.objective = cost_func.objective(current_position, desired_velocity)
        # model.objectiveN = cost_func.objectiveN(current_position, desired_velocity)

        # Set initial condition
        problem["xinit"] = x[:, k]

        # Set runtime parameters (here, the next N points on the path)
        next_path_points = MPC_Planner_instance.extract_next_path_points(resampled_path_points.T, x[0:2, k], model.N)
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
        solver_forcespro_instance.updatePlots(x, u, pred_x, pred_u, model, k)

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
