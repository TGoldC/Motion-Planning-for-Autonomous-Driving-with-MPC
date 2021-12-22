import casadi as ca
import numpy as np


class CostFuncCasadi:
    def __init__(self):
        pass

    @staticmethod
    def object_func(states, controls, reference_states, horizon):
        obj = 0 
        Q = np.array([[5.0, 0.0, 0.0, 0.0, 0.0], [0.0, 5.0, 0.0, 0.0, 0.0], [0.0, 0.0, 100, 0.0, 0.0], [0.0, 0.0, 0.0, 1, 0.0], [0.0, 0.0, 0.0, 0.0, 1]])
        R = np.array([[1, 0.0], [0.0, 1]])
        ## cost
        for i in range(horizon):
            # obj = obj + ca.mtimes([(X[:, i]-P[3:]).T, Q, X[:, i]-P[3:]]) + ca.mtimes([U[:, i].T, R, U[:, i]])
    
            obj = obj + (states[:, i]-reference_states[5:]).T @ Q @ (states[:, i]-reference_states[5:]) + controls[:, i].T @ R @ controls[:, i] 
            + (states[:, -1]-reference_states[5:]).T @ Q @ (states[:, -1]-reference_states[5:])
        return obj


class CostFuncForcespro:
    def __init__(self, weights=np.array([0.1,   0.1, 200.0, 200.0,  0.1,   200.0, 0.1])):
        # weights to penalty:  deltaDot aLong xPos  yPos  delta  velocity psi
        self.weights = weights

    def objective(self, current_target, desired_velocity):
        # z = [deltaDot,aLong,xPos,yPos,delta,v,psi]
        # The function must be able to handle symbolic evaluation,
        # by passing in CasADi symbols. This means certain numpy functions are not available.
        return lambda z, _current_target = current_target: (self.weights[2] * (z[2] - _current_target[0]) ** 2  # costs on deviating on the path in x-direction
                                                            + self.weights[3] * (z[3] - _current_target[1]) ** 2  # costs on deviating on the path in y-direction
                                                            + self.weights[4] * z[4] ** 2  # penalty on steering angle
                                                            + self.weights[5] * (z[5] - desired_velocity) ** 2  # penalty on velocity
                                                            + self.weights[6] * z[6] ** 2  # penalty on heading angle
                                                            + self.weights[0] * z[0] ** 2  # penalty on input velocity of steering angle
                                                            + self.weights[1] * z[1] ** 2)  # penalty on input longitudinal acceleration

    def objectiveN(self, current_target, desired_velocity):  # increased costs for the last stage
        return lambda z, _current_target=current_target: (self.weights[2] * 2 * (z[2] - _current_target[0]) ** 2  # costs on deviating on the path in x-direction
                                                          + self.weights[3] * 2 * (z[3] - _current_target[1]) ** 2  # costs on deviating on the path in y-direction
                                                          + self.weights[4] * z[4] ** 2  # penalty on steering angle
                                                          + self.weights[5] * 2 * (z[5] - desired_velocity) ** 2  # penalty on velocity
                                                          + self.weights[6] * z[6] ** 2  # penalty on heading angle
                                                          + self.weights[0] * z[0] ** 2  # penalty on input velocity of steering angle
                                                          + self.weights[1] * z[1] ** 2)  # penalty on input longitudinal acceleration

