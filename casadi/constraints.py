import casadi as ca
import numpy as np


class Constraints():
    def __init__(self):
        #steering angles
        self.delta_min = -1.066
        self.delta_max = 1.066
        #steering velocity
        self.deltav_min = -0.4
        self.deltav_max = 0.4
        #velocity
        self.v_min = -13.6
        self.v_max = 50.8
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