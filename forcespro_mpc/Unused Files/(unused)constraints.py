import numpy as np
from scipy.optimize import Bounds
from scipy.optimize import NonlinearConstraint
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2


class Constraints:
    """constraints for solver"""
    def __init__(self, p):
        self.p = p
        self.l = p.l

        self.pos_x_min = -np.inf     # lower & upper constraint for x1
        self.pos_x_max = np.inf
        self.pos_y_min = -2          # # lower & upper constraint for x2   ?????
        self.pos_y_max = 2
        self.steering_angle_min = self.p.steering.min  # lower & upper constraint for x3 steering angle
        self.steering_angle_max = self.p.steering.max
        self.v_x_min = self.p.longitudinal.v_min       # lower & upper constraint for x4
        self.v_x_max = self.p.longitudinal.v_max
        self.yaw_angle_min = -np.pi  # # lower & upper constraint for x5
        self.yaw_angle_max = np.pi

        self.v_steering_angle_min = self.p.steering.v_min   # # lower & upper constraint for u1
        self.v_steering_angle_max = self.p.steering.v_max

        self.a_max = self.p.longitudinal.a_max  # 11.5

    def upper_lower_bound_x_u(self, z):
        a_long_max, a_long_min = self. friction_circle_constraint(z)
        bounds = Bounds([self.pos_x_min, self.pos_y_min, self.steering_angle_min, self.v_x_min, self.yaw_angle_min, self.v_steering_angle_min, a_long_min],
                        [self.pos_x_max, self.pos_y_max, self.steering_angle_max, self.v_x_max, self.yaw_angle_max, self.v_steering_angle_max, a_long_max])
        return bounds

    def friction_circle_constraint(self, z):
        # a_long **2 + (v * psi_dot)**2 <= a_max**2
        psi_dot = z[3] / (self.l * np.tan(z[2] + 1e-6))
        a_long_max = np.sqrt(self.a_max ** 2 - (z[6] * psi_dot) ** 2)
        return a_long_max, -a_long_max
    """
       def friction_circle_constraint(self, z):  # z is the stack vector of x and u
            #  a_long **2 + (v * psi_dot)**2 <= a_max**2
            psi_dot = z[3]/(self.l * np.tan(z[2]))
            con = lambda z: np.sqrt(z[6]**2 + (z[3]*psi_dot)**2)
            nlc = NonlinearConstraint(con, 0, self.a_max)
            return nlc
    """
    def dynamic_model_constraints(self, z_stack):
        """
        z_stack 是一个很长的一维数组。7个7个一组，要满足车的动态方程
        并且上面的bounds的都得变 ————> 太麻烦了！
        """
        pass