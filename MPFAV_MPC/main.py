
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from constraints import Constraints
import numpy as np
from configuration import ReferencePath, ks_dynamics
from scipy.optimize import minimize

"""
def optimize(z0, t):
    p = parameters_vehicle2()
    x_u_constraints = TIConstraints(p)  # x_u_constraints define the upper and lower values of states x and inputs u
    bounds = x_u_constraints.upper_lower_bound_x_u()
    Reference_path = ReferencePath(path_scenario="/home/xin/PycharmProjects/MPFAV_MPC/scenarios/", id_scenario="USA_Peach-2_1_T-1.xml")
    position_init, reference_path = Reference_path.reference_path()  # [0, 0,]    (800,2)
"""


def shift_movement(T, t0, x0, u):  # using optimal control u to move the vehicle
    p = parameters_vehicle2()
    f = vehicle_dynamics_ks(x0, u0, p)
    next_state = x0 + f*T  # move the vehicle
    t = t0 + T
    u_end = np.hstack(u[:, 1:], u[:, -1])
    return t, next_state, u_end


if __name__ == '__main__':
    T = 0.2  # time step
    N = 5   # predictive horizon steps
    n_states = 5  # number of states and number of inputs
    n_controls = 2

    X, U = np.zeros((n_states, N+1)), np.zeros((n_controls, N))

    Reference_path = ReferencePath(path_scenario="/home/xin/PycharmProjects/MPFAV_MPC/scenarios/",
                                   id_scenario="USA_Peach-2_1_T-1.xml")
    reference_path = Reference_path.reference_path  # (800,2)
    position_init = Reference_path.position_init  # [0, 0,]

    # define the initialization
    #               x_pos                y_pos         steering angle   velocity    yaw_angle
    x0 = np.array([position_init[0],   position_init[1],       0,            0,        0])
    #       steering angle of velocity        acceleration_lon
    u0 = np.array([0.1,                           1])
    z0 = np.concatenate((x0, u0), axis=0)  # define a stack vector
    X[:, 0] = x0
    U[:, 0] = u0
    U_hat = np.squeeze(U.reshape((1, -1)))
    # U = U_hat.reshape((2, -1))

# cost function
    Q = np.array([[0.05, 0.0], [0.0, 0.5]])
    R = np.array([[0.5, 0.0], [0.0, 0.05]])
    obj = 0
    for i in range(N):
        index_in_reference_path = Reference_path.find_nearest_point_in_reference_path(T * i)
        path_difference = X[:2, i].reshape(-1, 1)-reference_path[index_in_reference_path].reshape(-1, 1)
        obj = obj + path_difference.T @ Q @ path_difference + U[:2, i].T @ R @ U[:2, i]

# constraints
    Z_stack = X[:, 0]
    for i in range(N):
        # stack all states and inputs together
        Z_stack = np.hstack((Z_stack, U[:, i], X[:, i+1]))  # len(Z_stack) = 5 * (N+1) + 2 * N

    p = parameters_vehicle2()
    Constraints = Constraints(p)

    Constraints_bounds = Constraints.upper_lower_bound_x_u(z0)
    solution = minimize(obj, U_hat, method='SLSQP', options={'ftol': 1e-9, 'disp': True},
                        bounds=Constraints_bounds)
    print(solution.x)

""""
    for i in range(N):
        f_value = np.array(vehicle_dynamics_ks(X[:, i], U[:, i], p))
        X[:, i+1] = X[:, i] + (f_value*T).T     
"""












