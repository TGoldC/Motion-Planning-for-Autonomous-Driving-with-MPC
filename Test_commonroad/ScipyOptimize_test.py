import numpy as np
from scipy.optimize import minimize
from scipy.optimize import Bounds
from scipy.optimize import LinearConstraint  # 用来定义 线性的不等式 和 等式（上下限取同一个值即可）
from scipy.optimize import NonlinearConstraint
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
import matplotlib.pyplot as plt

N = 3
delta_t = 0.2
num_states = 5
num_inputs = 2
# initial situation
# #            x_pos  y_pos  steering angle velocity    yaw_angle
# x0 = np.array([0,    0,        0,           0,           0])
# x1 = np.array([0,    0,        0,           0,           0])
# x2 = np.array([0,    0,        0,           0,           0])
#       steering angle of velocity        acceleration_lon
# u0 = np.array([0.1, 1])
ref = np.array([[0, 0], [1, 1], [2, 1]])
z = np.zeros(num_states * N + num_inputs * (N-1))   # num_states * N + num_inputs * (N-1) = 5*3 + 2*2=19
# array([x0, u0, x1, u1, x2])
#         5   2   5   2   5
# print(u[0:2]) # [0, 1]


# cost function
def obj(z, *args):
    N = args[0]
    num_states = args[1]
    num_inputs = args[2]
    obj_states = 0
    obj_inputs = 0
    for time_step in range(N):
        obj_states += (z[time_step * (num_states + num_inputs)] - ref[time_step, 0])**2 + \
                        (z[time_step * (num_states + num_inputs) + 1] - ref[time_step, 1])**2
    for time_step in range(N-1):
        obj_inputs += np.linalg.norm(z[(time_step * (num_states + num_inputs) + num_states):((time_step + 1) * (num_states + num_inputs))])**2

    return obj_states + obj_inputs


# constraint
bounds_unit = [[0, 4], [-2, 2], [-1.066, 1.066], [-13.6, 50.8], [-1, 1], [-0.4, 0.4], [-13.6, 13.6]]
bounds = bounds_unit * N   # len(bounds) = 21, not 19, because it contains one more additional control input
for _ in range(num_inputs):
    del bounds[-1]


def cons(*args):
    N = args[0]    # 3
    num_states = args[1]  # 5
    num_inputs = args[2]  # 2
    p = args[3]
    constraints = []
    for time_step in range(N-1):
        constraints.append({'type': 'eq',
                           'fun': lambda z, time_step = time_step: z[((time_step+1) * (num_states + num_inputs)):((time_step+1) * (num_states + num_inputs) + num_states)]
                                            - z[(time_step * (num_states + num_inputs)):(time_step * (num_states + num_inputs) + num_states)]
                                            - np.array(vehicle_dynamics_ks(z[(time_step * (num_states + num_inputs)):(time_step * (num_states + num_inputs) + num_states)],
                                                                           z[(time_step * (num_states + num_inputs) + num_states):((time_step + 1) * (num_states + num_inputs))], p)) * delta_t})
        # np.array(vehicle_dynamics_ks(z[0:5], z[5:7], p)) * delta_t}
        # 'fun': lambda z: z[14:19] - z[7:12] - np.array(vehicle_dynamics_ks(z[7:12], z[12:14], p)) * delta_t}
    return constraints


p = parameters_vehicle2()
cons1 = cons(N, num_states, num_inputs, p)  # return a list of constraints for dynamic model
cons2 = [{'type': 'eq',
        'fun': lambda z: z[7:12] - z[0:5] - np.array(vehicle_dynamics_ks(z[0:5], z[5:7], p)) * delta_t},
        # x1 - x0 - np.array(vehicle_dynamics_ks(x0, u[0:2], p)) * delta_t
        {'type': 'eq',
         'fun': lambda z: z[14:19] - z[7:12] - np.array(vehicle_dynamics_ks(z[7:12], z[12:14], p)) * delta_t}
        # x2 - x1 - np.array(vehicle_dynamics_ks(x1, u[2:4], p)) * delta_t
        ]

print(cons1 == cons2)

init_guess = np.ones(num_states * N + num_inputs * (N-1))
res = minimize(obj, init_guess, args=(N, num_states, num_inputs, p), method='SLSQP', constraints=cons1, options={'ftol': 1e-9, 'disp': True}, bounds=bounds)
print(res.x)

# N = 3
# delta_t = 0.2
# # initial situation
# # #            x_pos  y_pos  steering angle velocity    yaw_angle
# # x0 = np.array([0,    0,        0,           0,           0])
# # x1 = np.array([0,    0,        0,           0,           0])
# # x2 = np.array([0,    0,        0,           0,           0])
# #       steering angle of velocity        acceleration_lon
# # u0 = np.array([0.1, 1])
# ref = np.array([[0, 0], [1, 1], [2, 1]])
# z = np.zeros(19)   # num_states * N + num_inputs * (N-1) = 5*3 + 2*2=19
# # array([x0, u0, x1, u1, x2])
# #         5   2   5   2   5
# # print(u[0:2]) # [0, 1]
#
#
# # cost function
# def obj(z):
#     return (z[0] - ref[0, 0])**2 + (z[1]-ref[0, 1])**2 + (z[7] - ref[1, 0])**2 + (z[8]-ref[1, 1])**2 + \
#       (z[14] - ref[2, 0])**2 + (z[15]-ref[2, 1])**2 + np.linalg.norm(z[5:7])**2 + np.linalg.norm(z[12:14])**2
#
#
# # constraint
# bounds_unit = [[0, 4], [-2, 2], [-1.066, 1.066], [-13.6, 50.8], [-1, 1], [-0.4, 0.4], [-13.6, 13.6]]
# bounds = bounds_unit * 3   # len(bounds) = 21, not 19, because it contains one more additional control input
# del bounds[-1]
# del bounds[-1]
#
# p = parameters_vehicle2()
# cons = [{'type': 'eq',
#         'fun': lambda z: z[7:12] - z[0:5] - np.array(vehicle_dynamics_ks(z[0:5], z[5:7], p)) * delta_t},
#         # x1 - x0 - np.array(vehicle_dynamics_ks(x0, u[0:2], p)) * delta_t
#         {'type': 'eq',
#          'fun': lambda z: z[14:19] - z[7:12] - np.array(vehicle_dynamics_ks(z[7:12], z[12:14], p)) * delta_t}
#         # x2 - x1 - np.array(vehicle_dynamics_ks(x1, u[2:4], p)) * delta_t
#         ]
#
# init_guess = np.ones(19)
# res = minimize(obj, init_guess, method='SLSQP', constraints=cons, options={'ftol': 1e-9, 'disp': True}, bounds=bounds)
# print(res.x)
# path_planned = np.concatenate((res.x[0:2], res.x[7:9], res.x[14:16]), axis=0).reshape(3,2)
# print("path_planned", path_planned)
# print("ref", ref)
#
# plt.figure()
# plt.plot(ref[:, 0], ref[:, 1], label='Reference Path')
# plt.plot(path_planned[:, 0], path_planned[:, 1], label='Solver Path')
# plt.legend()
# plt.show()


# 让 1D array中只包括 u
# N = 3
# delta_t = 0.2
# # initial situation
# #            x_pos  y_pos  steering angle velocity    yaw_angle
# x0 = np.array([0,    0,        0,           0,           0])
# x1 = np.array([0,    0,        0,           0,           0])
# x2 = np.array([0,    0,        0,           0,           0])
# #       steering angle of velocity        acceleration_lon
# # u0 = np.array([0.1, 1])
# ref = np.array([[0, 0], [1, 0], [2, 0]])
# u = np.array([0, 0, 0, 0])   # [steering angle of velocity_firstTimeStep, acceleration_lon_firstTimeStep,
# # steering angle of velocity_secondTimeStep, acceleration_lon_secondTimeStep]
# # print(u[0:2]) # [0, 1]
#
#
# # cost function
# def obj(u, x0, x1, x2):
#     return (x0[0] - ref[0, 0])**2 + (x0[1]-ref[0, 1])**2 + (x1[0] - ref[1, 0])**2 + (x1[1]-ref[1, 1])**2 + \
#       (x2[0] - ref[2, 0])**2 + (x2[1]-ref[2, 1])**2 + np.linalg.norm(u[0:2])**2 + np.linalg.norm(u[2:4])**2
#
#
# # constraint
# bounds = [[-2, 2], [-6, 6], [-2, 2], [-6, 6]]
# p = parameters_vehicle2()
# cons = [{'type': 'eq',
#         'fun': lambda u: x1 - x0 - np.array(vehicle_dynamics_ks(x0, u[0:2], p)) * delta_t},
#         {'type': 'eq',
#          'fun': lambda u: x2 - x1 - np.array(vehicle_dynamics_ks(x1, u[2:4], p)) * delta_t},
#         {'type': 'ineq',
#          'fun': lambda x1: 4 - abs(x1[0])},   # x_position max = 4
#         {'type': 'ineq',
#          'fun': lambda x1: 1 - abs(x1[1])},
#         {'type': 'ineq',
#          'fun': lambda x2: 4 - abs(x2[0])},
#         {'type': 'ineq',
#          'fun': lambda x2: 1 - abs(x2[1])}]
#
# init_guess = np.array([1, 1, 1, 1])
# res = minimize(obj, init_guess, method='SLSQP', constraints=cons, options={'ftol': 1e-9, 'disp': True}, bounds=bounds)
# # res = minimize(obj, init_guess, method='SLSQP', options={'ftol': 1e-9, 'disp': True}, bounds=bounds)
# print(res.x)

