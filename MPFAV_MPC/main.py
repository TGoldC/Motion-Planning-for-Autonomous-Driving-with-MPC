
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from constraints import TIConstraints
import numpy as np
from configuration import ReferencePath, ks_dynamics


def optimize(z0, t):
    p = parameters_vehicle2()
    x_u_constraints = TIConstraints(p)  # x_u_constraints define the upper and lower values of states x and inputs u
    bounds = x_u_constraints.upper_lower_bound_x_u()
    Reference_path = ReferencePath(path_scenario="/home/xin/PycharmProjects/MPFAV_MPC/scenarios/", id_scenario="USA_Peach-2_1_T-1.xml")
    position_init, reference_path = Reference_path.reference_path()  # [0, 0,]    (800,2)


def shit_movement(T, t0, x0, u):
    p = parameters_vehicle2()
    f = vehicle_dynamics_ks(x0, u0, p)
    next_state = x0 + f*T  # move the vehicle
    t = t0 + T
    u_end = np.hstack(u[:, 1:], u[:, -1])
    return t, next_state, u_end


if __name__ == '__main__':
    T = 0.2  # time step
    N = 50   # predictive horizon steps
    n_states = 5  # number of states and number of inputs
    n_controls = 2

    X, U = np.zeros((n_states, N+1)), np.zeros((n_controls, N))

    Reference_path = ReferencePath(path_scenario="/home/xin/PycharmProjects/MPFAV_MPC/scenarios/", id_scenario="USA_Peach-2_1_T-1.xml")
    position_init, reference_path = Reference_path.reference_path()  # [0, 0,]    (800,2)

    # define the initialization
    #                 x_pos                 y_pos         steering angle   velocity_x    yaw_angle
    x0 = np.array([position_init[0],   position_init[1],       [0],            [0],        [0]])
    #       steering angle of velocity        acceleration_lon
    u0 = np.array([[0.1],                           [1]])
    z0 = np.concatenate((x0, u0), axis=0)  # define a stack vector
    X[:, 0] = x0
    U[:, 0] = u0

    p = parameters_vehicle2()
    for i in range(N):
        f_value = vehicle_dynamics_ks(X[:, i], U[:, i], p)
        X[:, i+1] = X[:, i] + f_value*T

    Q = np.array([[1.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 5.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, .1, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.1, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.1]])
    R = np.array([[0.5, 0.0], [0.0, 0.05]])
    # cost function
    obj = 0
    for i in range(N):
        obj = obj + (X[:, i]-reference_path[i].T).T @ Q @ (X[:, i]-reference_path[i].T) + U[:, i].T @ R @ U[:, i]




    # for i in range(len(t)):
    #     print("i = ", i)
    #     result = optimize()












