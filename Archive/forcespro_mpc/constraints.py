import casadi as ca
import numpy as np
import forcespro
import forcespro.nlp
from dynamics import Vehicle_dynamics
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2


class Constraints:
    def __init__(self, p=parameters_vehicle2()):
        # steering angles
        self.delta_min = p.steering.min  # -1.066
        self.delta_max = p.steering.max  # 1.066
        # steering velocity
        self.deltav_min = p.steering.v_min  # -0.4
        self.deltav_max = p.steering.v_max  # 0.4
        # velocity
        self.v_min = 0  # highway
        self.v_max = p.longitudinal.v_max  # 50.8
        # acceleration
        self.a_max = p.longitudinal.a_max  # 11.5

    @staticmethod
    def equal_constraints(states, horizon):
        g = [] # equal constraints
        for i in range(horizon+1):
            g.append(states[2, i])
            g.append(states[3, i])
        return g

    def inequal_constraints(self, horizon):
        # states constraints
        lbg = []
        ubg = []
        for _ in range(horizon+1):
            lbg.append(self.delta_min)
            lbg.append(self.v_min)
            ubg.append(self.delta_max)
            ubg.append(self.v_max)    
        # control constraints
        lbx = []
        ubx = []
        for _ in range(horizon):
            lbx.append(self.deltav_min)
            ubx.append(self.deltav_max)
            lbx.append(-np.inf)
            ubx.append(np.inf)
        return lbg, ubg, lbx, ubx

    @staticmethod
    def forcespro_equal_constraint():
        # We use an explicit RK4 integrator here to discretize continuous dynamics
        integrator_stepsize = 0.1
        return lambda z: forcespro.nlp.integrate(Vehicle_dynamics.KS_casadi, z[2:7], z[0:2],
                                                     integrator=forcespro.nlp.integrators.RK4,
                                                     stepsize=integrator_stepsize)

    def forcespro_inequal_constraint(self):
        # Inequality constraints
        # z = [deltaDot,aLong,xPos,yPos,delta,v,psi]
        #  upper/lower variable bounds lb <= z <= ub
        #                     inputs      |  states
        #                deltaDot aLong    x    y   delta    v    psi
        # model.lb = np.array([-0.4, -11.5, -10, -30, -1.066, 0., -np.inf])  # 1.066 = 61 degree
        # model.ub = np.array([+0.4, +11.5, 100, 100, +1.066, 50.8, np.inf])
        # TODO : add the upper and lower position x and y
        #  upper/lower variable bounds lb <= z <= ub
        #                     inputs      |  states
        #                       deltaDot            aLong    x    y   delta                 v      psi
        low_bound = np.array([self.deltav_min, -self.a_max, -10, -30, self.delta_min, self.v_min, -np.inf])  # 1.066 = 61 degree
        upper_bound = np.array([self.deltav_max, self.a_max, 100, 100, self.delta_max, self.v_max, np.inf])
        return low_bound, upper_bound
