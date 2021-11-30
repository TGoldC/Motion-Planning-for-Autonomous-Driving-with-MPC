import numpy as np
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2


class TIConstraints:
    """Time Invariant constraints"""
    def __init__(self, p):
        self.p = p

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
        self. a_lon_min = -self.p.longitudinal.a_max
        self.a_lon_max = self.p.longitudinal.a_max

    def upper_lower_bound_x_u(self):
        bounds = ((self.pos_x_min, self.pos_x_max),
                  (self.pos_y_min, self.pos_y_max),
                  (self.steering_angle_min, self.steering_angle_max),
                  (self.v_x_min, self.v_x_max),
                  (self.yaw_angle_min, self.yaw_angle_max),
                  (self.v_steering_angle_min, self.v_steering_angle_max),
                  (self.a_lon_min, self.a_lon_max))
        return bounds

