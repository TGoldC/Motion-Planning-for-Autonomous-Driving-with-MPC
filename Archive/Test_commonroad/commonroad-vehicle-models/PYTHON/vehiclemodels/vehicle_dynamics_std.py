import math
from vehiclemodels.utils.steering_constraints import steering_constraints
from vehiclemodels.utils.acceleration_constraints import acceleration_constraints
from vehiclemodels.utils.vehicle_dynamics_ks_cog import vehicle_dynamics_ks_cog
import vehiclemodels.utils.tire_model as tire_model

__author__ = "Gerald Würsching"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "2020a"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"


def vehicle_dynamics_std(x, u_init, p):
    """
    vehicle_dynamics_std - single-track drift model vehicle dynamics

    Syntax:
        f = vehicle_dynamics_std(x,u_init,p)

    Inputs:
        :param x: vehicle state vector
        :param u_init: vehicle input vector
        :param p: vehicle parameter vector

    Outputs:
        :return f: right-hand side of differential equations

    Author:         Gerald Würsching
    Written:        23-October-2020
    Last update:    23-October-2020
                    02-February-2021
    Last revision:  ---
    """

    # set gravity constant
    g = 9.81  # [m/s^2]

    # create equivalent bicycle parameters
    lf = p.a
    lr = p.b
    lwb = p.a + p.b
    m = p.m
    I = p.I_z

    # mix models parameters
    v_s = 0.2
    v_b = 0.05
    v_min = v_s/2

    # states
    # x1 = x-position in a global coordinate system
    # x2 = y-position in a global coordinate system
    # x3 = steering angle of front wheels
    # x4 = velocity at vehicle center
    # x5 = yaw angle
    # x6 = yaw rate
    # x7 = slip angle at vehicle center
    # x8 = front wheel angular speed
    # x9 = rear wheel angular speed

    # u1 = steering angle velocity of front wheels
    # u2 = longitudinal acceleration

    # steering and acceleration constraints
    u = []
    u.append(steering_constraints(x[2], u_init[0], p.steering))  # different name due to side effects of u
    u.append(acceleration_constraints(x[3], u_init[1], p.longitudinal))  # different name due to side effect of u

    # compute lateral tire slip angles
    alpha_f = math.atan((x[3] * math.sin(x[6]) + x[5] * lf) / (x[3] * math.cos(x[6]))) - x[2] if x[3] > v_min else 0
    alpha_r = math.atan((x[3] * math.sin(x[6]) - x[5] * lr) / (x[3] * math.cos(x[6]))) if x[3] > v_min else 0

    # compute vertical tire forces
    F_zf = m * (-u[1] * p.h_s + g * lr) / (lr + lf)
    F_zr = m * (u[1] * p.h_s + g * lf) / (lr + lf)

    # compute front and rear tire speeds
    u_wf = max(0, x[3] * math.cos(x[6]) * math.cos(x[2]) + (x[3] * math.sin(x[6]) + p.a * x[5]) * math.sin(x[2]))
    u_wr = max(0, x[3] * math.cos(x[6]))

    # compute longitudinal tire slip
    s_f = 1 - p.R_w * x[7] / max(u_wf, v_min)
    s_r = 1 - p.R_w * x[8] / max(u_wr, v_min)

    # compute tire forces (Pacejka)
    # pure slip longitudinal forces
    F0_xf = tire_model.formula_longitudinal(s_f, 0, F_zf, p.tire)
    F0_xr = tire_model.formula_longitudinal(s_r, 0, F_zr, p.tire)

    # pure slip lateral forces
    res = tire_model.formula_lateral(alpha_f, 0, F_zf, p.tire)
    F0_yf = res[0]
    mu_yf = res[1]
    res = tire_model.formula_lateral(alpha_r, 0, F_zr, p.tire)
    F0_yr = res[0]
    mu_yr = res[1]

    # combined slip longitudinal forces
    F_xf = tire_model.formula_longitudinal_comb(s_f, alpha_f, F0_xf, p.tire)
    F_xr = tire_model.formula_longitudinal_comb(s_r, alpha_r, F0_xr, p.tire)

    # combined slip lateral forces
    F_yf = tire_model.formula_lateral_comb(s_f, alpha_f, 0, mu_yf, F_zf, F0_yf, p.tire)
    F_yr = tire_model.formula_lateral_comb(s_r, alpha_r, 0, mu_yr, F_zr, F0_yr, p.tire)

    # convert acceleration input to brake and engine torque
    if u[1] > 0:
        T_B = 0.0
        T_E = m * p.R_w * u[1]
    else:
        T_B = m * p.R_w * u[1]
        T_E = 0.

    # system dynamics
    d_v = 1 / m * (-F_yf * math.sin(x[2] - x[6]) + F_yr * math.sin(x[6]) + F_xr * math.cos(x[6]) + F_xf * math.cos(x[2] - x[6]))
    dd_psi = 1 / I * (F_yf * math.cos(x[2]) * lf - F_yr * lr + F_xf * math.sin(x[2]) * lf)
    d_beta = -x[5] + 1 / (m * x[3]) * (F_yf * math.cos(x[2] - x[6]) + F_yr * math.cos(x[6]) - F_xr * math.sin(x[6]) + F_xf * math.sin(x[2] - x[6])) if x[3] > v_min else 0

    # wheel dynamics (negative wheel spin forbidden)
    d_omega_f = 1 / p.I_y_w * (-p.R_w * F_xf + p.T_sb * T_B + p.T_se * T_E) if x[7] >= 0 else 0
    x[7] = max(0, x[7])
    d_omega_r = 1 / p.I_y_w * (-p.R_w * F_xr + (1 - p.T_sb) * T_B + (1 - p.T_se) * T_E) if x[8] >= 0 else 0
    x[8] = max(0, x[8])

    # *** Mix with kinematic model at low speeds ***
    # kinematic system dynamics
    x_ks = [x[0], x[1], x[2], x[3], x[4]]
    f_ks = vehicle_dynamics_ks_cog(x_ks, u, p)
    # derivative of slip angle and yaw rate (kinematic)
    d_beta_ks = (p.b * u[0]) / (lwb * math.cos(x[2]) ** 2 * (1 + (math.tan(x[2]) ** 2 * p.b / lwb) ** 2))
    dd_psi_ks = 1 / lwb * (u[1] * math.cos(x[6]) * math.tan(x[2]) -
                        x[3] * math.sin(x[6]) * d_beta_ks * math.tan(x[2]) +
                        x[3] * math.cos(x[6]) * u[0] / math.cos(x[2]) ** 2)
    # derivative of angular speeds (kinematic)
    d_omega_f_ks = (1 / 0.02) * (u_wf / p.R_w - x[7])
    d_omega_r_ks = (1 / 0.02) * (u_wr / p.R_w - x[8])

    # weights for mixing both models
    w_std = 0.5 * (math.tanh((x[3] - v_s)/v_b) + 1)
    w_ks = 1 - w_std

    # output vector: mix results of dynamic and kinematic model
    f = [x[3] * math.cos(x[6] + x[4]),
         x[3] * math.sin(x[6] + x[4]),
         u[0],
         w_std * d_v + w_ks * f_ks[3],
         w_std * x[5] + w_ks * f_ks[4],
         w_std * dd_psi + w_ks * dd_psi_ks,
         w_std * d_beta + w_ks * d_beta_ks,
         w_std * d_omega_f + w_ks * d_omega_f_ks,
         w_std * d_omega_r + w_ks * d_omega_r_ks]

    return f


# ***********************************
# Test Model
# ***********************************
if __name__ == "__main__":
    from parameters_vehicle2 import parameters_vehicle2
    from init_std import init_std
    from scipy.integrate import odeint, solve_ivp
    import numpy
    import matplotlib.pyplot as plt

    def func_STD(x, t, u, p):
        f = vehicle_dynamics_std(x, u, p)
        return f

    def func_STD_2(t, x, u, p):
        f = vehicle_dynamics_std(x, u, p)
        return f

    def plot_odeint(x_std, t):
        plt.plot([tmp[0] for tmp in x_std], [tmp[1] for tmp in x_std])
        plt.title('position')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.show()

        plt.plot(t, [tmp[2] for tmp in x_std])
        plt.title('steering angle')
        plt.show()

        plt.plot(t, [tmp[3] for tmp in x_std])
        plt.title('velocity')
        plt.show()

        plt.plot(t, [tmp[4] for tmp in x_std])
        plt.title('yaw angle')
        plt.show()

        plt.plot(t, [tmp[5] for tmp in x_std])
        plt.title('yaw rate')
        plt.show()

        plt.plot(t, [tmp[6] for tmp in x_std])
        plt.title('slip angle')
        plt.show()

        plt.plot(t, [tmp[7] for tmp in x_std])
        plt.title('omega front')
        plt.show()

        plt.plot(t, [tmp[8] for tmp in x_std])
        plt.title('omega rear')
        plt.show()

    def plot_solve_ivp(x_std):
        plt.plot(x_std.y[0], x_std.y[1])
        plt.title('position')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.show()

        plt.plot(x_std.t, x_std.y[2])
        plt.title('steering angle')
        plt.show()

        plt.plot(x_std.t, x_std.y[3])
        plt.title('velocity')
        plt.show()

        plt.plot(x_std.t, x_std.y[4])
        plt.title('yaw angle')
        plt.show()

        plt.plot(x_std.t, x_std.y[5])
        plt.title('yaw rate')
        plt.show()

        plt.plot(x_std.t, x_std.y[6])
        plt.title('slip angle')
        plt.show()

        plt.plot(x_std.t, x_std.y[7])
        plt.title('omega front')
        plt.show()

        plt.plot(x_std.t, x_std.y[8])
        plt.title('omega rear')
        plt.show()

    # load parameters
    p = parameters_vehicle2()
    g = 9.81  # [m/s^2]

    # set options --------------------------------------------------------------
    tStart = 0  # start time
    tFinal = 1  # start time

    delta0 = 0.0
    vel0 = 0
    Psi0 = 0
    dotPsi0 = 0
    beta0 = 0
    sy0 = 0
    initialState = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]  # initial state for simulation
    x0_STD = init_std(initialState, p)  # initial state for multi-body model
    # --------------------------------------------------------------------------

    t = numpy.arange(0, tFinal, 0.01)
    u = [0.0, 0.6 * g]

    # using odeint
    # x_std, out_dict = odeint(func_STD, x0_STD, t, args=(u, p), printmessg=True, full_output=True)
    # print(out_dict)
    # plot_odeint(x_std, t)

    # using solve_ivp
    x_std = solve_ivp(func_STD_2, (tStart, tFinal), x0_STD, args=(u, p))
    plot_solve_ivp(x_std)