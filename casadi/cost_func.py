import casadi as ca
import numpy as np

class Cost_Func_casadi():
    def object_func(self, states, controls, reference_states, horizon):
        obj = 0 
        Q = np.array([[5.0, 0.0, 0.0, 0.0, 0.0],[0.0, 5.0, 0.0, 0.0, 0.0],[0.0, 0.0, 100, 0.0, 0.0], [0.0, 0.0, 0.0, 1, 0.0], [0.0, 0.0, 0.0, 0.0, 1]])
        R = np.array([[1, 0.0], [0.0, 1]])
        ## cost
        for i in range(horizon):
            # obj = obj + ca.mtimes([(X[:, i]-P[3:]).T, Q, X[:, i]-P[3:]]) + ca.mtimes([U[:, i].T, R, U[:, i]])
    
            obj = obj + (states[:, i]-reference_states[5:]).T @ Q @ (states[:, i]-reference_states[5:]) + controls[:, i].T @ R @ controls[:, i] 
            + (states[:, -1]-reference_states[5:]).T @ Q @ (states[:, -1]-reference_states[5:])
        return obj
