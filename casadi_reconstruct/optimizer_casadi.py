import casadi as ca
import numpy as np
from configuration import Vehicle_dynamics

class MPC_car():
    def __init__(self, state_dim, T=0.1, N=10):
        #steering angles
        self.delta_min = -1.066
        self.delta_max = 1.066
        #steering velocity
        self.deltav_min = -0.4
        self.deltav_max = 0.4
        #velocity
        self.v_min = -13.6
        self.v_max = 50.8
        #define prediction horizon and sampling time
        self.Ts = T
        self.horizon = N       
        #set states variables
        sx = ca.SX.sym('sx')
        sy = ca.SX.sym('sy')
        delta = ca.SX.sym('delta')
        vel = ca.SX.sym('vel')
        Psi = ca.SX.sym('Psi')
        states = ca.vertcat(*[sx, sy, delta, vel, Psi])
        num_states = states.size()[0]
        #set control variables
        u0 = ca.SX.sym('u0')
        u1 = ca.SX.sym('u1')
        controls = ca.vertcat(*[u0, u1])
        num_controls = controls.size()[0]
        #get euqations from dynamics.py
        d = Vehicle_dynamics()
        rhs = d.KS(states, controls, type = 'casadi')
        self.f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])
        ## for MPC
        U = ca.SX.sym('U', num_controls, self.horizon)
        X = ca.SX.sym('X', num_states, self.horizon+1)
        P = ca.SX.sym('P', num_states+num_states)
        #X_ref = ca.SX.sym('X_ref', num_states, self.horizon+1)

        X[:, 0] = P[:5] # initial condition

        #### define the relationship within the horizon
        for i in range(self.horizon):
            f_value = self.f(X[:, i], U[:, i])
            X[:, i+1] = X[:, i] + f_value*T
    
        self.ff = ca.Function('ff', [U, P], [X], ['input_U', 'reference_state'], ['horizon_states'])

        obj = self.object_func(X, U, P, N)

        #self.Q = np.array([[5.0, 0.0, 0.0, 0.0, 0.0],[0.0, 5.0, 0.0, 0.0, 0.0],[0.0, 0.0, 100, 0.0, 0.0], [0.0, 0.0, 0.0, 1, 0.0], [0.0, 0.0, 0.0, 0.0, 1]])
        #self.R = np.array([[1, 0.0], [0.0, 1]])
#
        #obj = 0 
        ### cost
        #for i in range(N):
        #    # obj = obj + ca.mtimes([(X[:, i]-P[3:]).T, Q, X[:, i]-P[3:]]) + ca.mtimes([U[:, i].T, R, U[:, i]])
    #
        #    obj = obj + (X[:, i]-P[5:]).T @ self.Q @ (X[:, i]-P[5:]) + U[:, i].T @ self.R @ U[:, i] 
        #    + (X[:, -1]-P[5:]).T @ self.Q @ (X[:, -1]-P[5:])
       
        g = self.equal_constraints(X, self.horizon)
        ## states constrains
        #g = [] # equal constrains
        #for i in range(self.horizon+1):
        #    g.append(X[2, i])
        #    g.append(X[3, i])
    
        nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p':P, 'g':ca.vcat(g)} # here also can use ca.vcat(g) or ca.vertcat(*g)
        opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6, }
    
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

    def shift_movement(self, t0, x0, u, f):
        f_value = f(x0, u[:, 0])
        st = x0 + self.Ts*f_value
        t = t0 + self.Ts
        u_end = ca.horzcat(u[:, 1:], u[:, -1])
    
        return t, st, u_end.T

    def object_func(self, states, controls, reference_states, horizon):
        obj = 0 
        #states penalty
        Q = np.array([[5.0, 0.0, 0.0, 0.0, 0.0],[0.0, 5.0, 0.0, 0.0, 0.0],[0.0, 0.0, 200, 0.0, 0.0], [0.0, 0.0, 0.0, 1, 0.0], [0.0, 0.0, 0.0, 0.0, 80]])
        #controls penalty
        R = np.array([[0.1, 0.0], [0.0, 0.1]])
        #end term penalty
        P = np.array([[5.0, 0.0, 0.0, 0.0, 0.0],[0.0, 5.0, 0.0, 0.0, 0.0],[0.0, 0.0, 200, 0.0, 0.0], [0.0, 0.0, 0.0, 1, 0.0], [0.0, 0.0, 0.0, 0.0, 80]])
        ## cost
        for i in range(horizon):
            # obj = obj + ca.mtimes([(X[:, i]-P[3:]).T, Q, X[:, i]-P[3:]]) + ca.mtimes([U[:, i].T, R, U[:, i]])
    
            obj = obj + (states[:, i]-reference_states[5:]).T @ Q @ (states[:, i]-reference_states[5:]) + controls[:, i].T @ R @ controls[:, i] 
            + (states[:, -1]-reference_states[5:]).T @ P @ (states[:, -1]-reference_states[5:])
        return obj

    def equal_constraints(self, states, horizon):
        g = [] # equal constraints
        for i in range(horizon+1):
            g.append(states[2, i])
            g.append(states[3, i])
        return g

    def inequal_constraints(self, horizon):
        #states constraints
        lbg = []
        ubg = []
        for _ in range(horizon+1):
            lbg.append(self.delta_min)
            lbg.append(self.v_min)
            ubg.append(self.delta_max)
            ubg.append(self.v_max)    
        #control constraints
        lbx = []
        ubx = []
        for _ in range(horizon):
            lbx.append(self.deltav_min)
            ubx.append(self.deltav_max)
            lbx.append(-np.inf)
            ubx.append(np.inf)
        return lbg, ubg, lbx, ubx