import casadi as ca
import numpy as np
from dynamics import Vehicle_dynamics
from cost_func import Cost_Func_casadi
from constraints import Constraints

class MPC_car():
    def __init__(self, state_dim, T=0.1, N=10):
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

        cost = Cost_Func_casadi()
        obj = cost.object_func(X, U, P, N)

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
       
        con = Constraints()
        g = con.equal_constraints(X, self.horizon)
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