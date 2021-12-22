import casadi as ca
import numpy as np
from dynamics import Vehicle_dynamics
from cost_func import CostFuncCasadi
from constraints import Constraints
import forcespro
import forcespro.nlp
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


class SolverCasadi:
    def __init__(self, state_dim, T=0.1, N=10):
        # define prediction horizon and sampling time
        self.Ts = T
        self.horizon = N
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
        d = Vehicle_dynamics()
        rhs = d.KS(states, controls, type='casadi')
        self.f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])
        ## for MPC
        U = ca.SX.sym('U', num_controls, self.horizon)
        X = ca.SX.sym('X', num_states, self.horizon + 1)
        P = ca.SX.sym('P', num_states + num_states)
        # X_ref = ca.SX.sym('X_ref', num_states, self.horizon+1)

        X[:, 0] = P[:5]  # initial condition

        #### define the relationship within the horizon
        for i in range(self.horizon):
            f_value = self.f(X[:, i], U[:, i])
            X[:, i + 1] = X[:, i] + f_value * T

        self.ff = ca.Function('ff', [U, P], [X], ['input_U', 'reference_state'], ['horizon_states'])

        cost = CostFuncCasadi()
        obj = cost.object_func(X, U, P, N)

        # self.Q = np.array([[5.0, 0.0, 0.0, 0.0, 0.0],[0.0, 5.0, 0.0, 0.0, 0.0],[0.0, 0.0, 100, 0.0, 0.0], [0.0, 0.0, 0.0, 1, 0.0], [0.0, 0.0, 0.0, 0.0, 1]])
        # self.R = np.array([[1, 0.0], [0.0, 1]])
        #
        # obj = 0
        ### cost
        # for i in range(N):
        #    # obj = obj + ca.mtimes([(X[:, i]-P[3:]).T, Q, X[:, i]-P[3:]]) + ca.mtimes([U[:, i].T, R, U[:, i]])
        #
        #    obj = obj + (X[:, i]-P[5:]).T @ self.Q @ (X[:, i]-P[5:]) + U[:, i].T @ self.R @ U[:, i]
        #    + (X[:, -1]-P[5:]).T @ self.Q @ (X[:, -1]-P[5:])

        con = Constraints()
        g = con.equal_constraints(X, self.horizon)
        ## states constrains
        # g = [] # equal constrains
        # for i in range(self.horizon+1):
        #    g.append(X[2, i])
        #    g.append(X[3, i])

        nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p': P, 'g': ca.vcat(g)}  # here also can use ca.vcat(g) or ca.vertcat(*g)
        opts_setting = {'ipopt.max_iter': 100, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-8, 'ipopt.acceptable_obj_change_tol': 1e-6, }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

    def shift_movement(self, t0, x0, u, f):
        f_value = f(x0, u[:, 0])
        st = x0 + self.Ts * f_value
        t = t0 + self.Ts
        u_end = ca.horzcat(u[:, 1:], u[:, -1])

        return t, st, u_end.T


class SolverForcespro:
    def __init__(self):
        pass

    @staticmethod
    def obj(z, current_target):
        """Least square costs on deviating from the path and on the inputs
        z = [deltaDot,aLong,xPos,yPos,delta,v,psi]
        current_target = point on path that is to be headed for
        """
        return (400.0 * (z[2] - current_target[0]) ** 2  # costs on deviating on the path in x-direction
                + 400.0 * (z[3] - current_target[1]) ** 2  # costs on deviating on the path in y-direction
                + 200 * z[4] ** 2  # penalty on steering angle
                + 200 * z[5] ** 2  # penalty on velocity
                + 0.1 * z[6] ** 2  # penalty on heading angle
                + 0.1 * z[0] ** 2  # penalty on input velocity of steering angle
                + 0.1 * z[1] ** 2)  # penalty on input longitudinal acceleration

    @staticmethod
    def objN(z, current_target):
        """Increased least square costs for last stage on deviating from the path and
        on the inputs F and phi
        z = [F,phi,xPos,yPos,v,theta,delta]
        current_target = point on path that is to be headed for
        """
        return (400.0 * (z[2] - current_target[0]) ** 2  # costs on deviating on the path in x-direction
                + 400.0 * (z[3] - current_target[1]) ** 2  # costs on deviating on the path in y-direction
                + 100 * z[4] ** 2
                + 200 * z[5] ** 2
                + 0.2 * z[6] ** 2
                + 0.2 * z[0] ** 2  # penalty on input velocity of steering angle
                + 0.2 * z[1] ** 2)  # penalty on input longitudinal acceleration

    def generate_pathplanner(self):
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

        con = Constraints()
        model.eq = con.forcespro_equal_constraint()
        model.lb, model.ub = con.forcespro_inequal_constraint()

        model.objective = self.obj
        model.objectiveN = self.objN

        # Indices on LHS of dynamical constraint - for efficiency reasons, make
        # sure the matrix E has structure [0 I] where I is the identity matrix.
        model.E = np.concatenate([np.zeros((5, 2)), np.eye(5)], axis=1)

        # Initial condition on vehicle states x
        model.xinitidx = range(2, 7)  # use this to specify on which variables initial conditions are imposed

        # Solver generation
        # -----------------

        # Set solver options
        codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
        codeoptions.maxit = 200  # Maximum number of iterations
        codeoptions.printlevel = 0  # Use printlevel = 2 to print progress (but not for timings)
        codeoptions.optlevel = 0  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
        codeoptions.cleanup = False
        codeoptions.timing = 1
        codeoptions.nlp.hessian_approximation = 'bfgs'
        codeoptions.solvemethod = 'SQP_NLP'  # choose the solver method Sequential Quadratic Programming
        codeoptions.nlp.bfgs_init = 2.5 * np.identity(7)  # np.identity 创建方阵，对角线为1
        codeoptions.sqp_nlp.maxqps = 1  # maximum number of quadratic problems to be solved
        codeoptions.sqp_nlp.reg_hessian = 5e-9  # increase this if exitflag=-8
        # change this to your server or leave uncommented for using the standard embotech server at https://forces.embotech.com, codeoptions.server = 'https://forces.embotech.com'
        # Creates code for symbolic model formulation given above, then contacts server to generate new solver
        solver = model.generate_solver(options=codeoptions)

        return model, solver

    @staticmethod
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
        plt.ylim([0., 20])
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

    @staticmethod
    def updatePlots(x, u, pred_x, pred_u, model, k):
        """ Deletes old data sets in the current plot and adds the new data sets
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
        ax_list[4].step(range(k, k + model.N), np.rad2deg(pred_u[0, :]), 'g-')  # plot new prediction of steering rate
        ax_list[5].step(range(0, k + 1), u[1, 0:k + 1], 'b-')  # plot new acceleration
        ax_list[5].step(range(k, k + model.N), pred_u[1, :], 'g-')  # plot new prediction of acceleration

        plt.pause(0.05)
