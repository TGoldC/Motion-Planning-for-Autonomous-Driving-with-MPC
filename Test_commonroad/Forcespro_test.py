import sys
sys.path.insert(0, '/home/xin/MPFAV/Forcespro') # On Unix
import numpy as np
import casadi
import forcespro
import forcespro.nlp



"""
Description of problem:  ----> obstacle avoidance
x_dot = v * cos(theta)    # x-position of car
y_dot  = v * sin(theta)   # y-position of car
v_dot = F / m             # v is the linear velocity    mass m = 1 kg
theta_dot = s / L         # theta is the heading angle   wheel base L = 1m

control input: acceleration F and steering torque s

This is a NLP problem. 
Define stage variable z by stacking the input and differential state variables.
z = [F, s, x, y, v, theta].T
"""


# define vehicle model
def continuous_dynamics(x, u):
    m, length = 1, 1
    return np.array([x[2] * casadi.cos(x[3]),  # x_dot = v * cos(theta)
                     x[2] * casadi.sin(x[3]),  # y_dot  = v * sin(theta)
                     u[0] / m,                 # v_dot = F / m
                     u[1] / length])           # theta_dot = s / L


model = forcespro.nlp.SymbolicModel()

model.N = 50    # horizon length
model.nvar = 6  # number of stage variables
model.neq = 4   # number of equality constraints
model.nh = 2    # number of nonlinear inequality constraints
model.npar = 0  # number of runtime parameters

# define the object function
# maximize progress in the 𝑦 direction; with quadratic penalties on the inputs 𝐹 and 𝑠
# --> f(z) = -100 * y + 0.1 * square(z1) + 0.01 * square(z2)
model.objective = lambda z: -100 * z[3] + 0.1 * z[0]**2 + 0.01 * z[1]**2

# Use an explicit Runge-Kutta integrator of order 4 integrator here to discretize continuous dynamics
integrator_stepsize = 0.1
model.eq = lambda z: forcespro.nlp.integrate(continuous_dynamics, z[2:6], z[0:2],
                                             integrator=forcespro.nlp.integrators.RK4, stepsize=integrator_stepsize)

# Use the selection matrix 𝐸 to link the stage variables 𝑧𝑖 with the states 𝑥 and inputs 𝑢 of the
# continuous dynamics function
model.E = np.concatenate([np.zeros((4, 2)), np.eye(4)], axis=1)

# define inequalities constraints
# 1<= x**2 + y**2 <= 9
# 0.95**2 <= (x+2)**2 + (y-2.5)**2
model.ineq = lambda z: np.array([z[2] ** 2 + z[3] ** 2, (z[2] + 2) ** 2 + (z[3] - 2.5) ** 2])
model.hu = np.array([9, +np.inf])
model.hl = np.array([1, 0.95 ** 2])

# constraints
# -5<=F<= 5 N;   -1 <=s<= 1 Nm;  -3<=x<=0 m; 0<=y<=3 m; 0<=v<=2 m/s; 0<=theta<=pi rad
# init constraints: x_init = -2m; y_init = 0m; v_init=0m/s; theta_init=2.0944 rad
# final constraints: v_init = 0m/s; theta_init = 0 rad
#                    F    s   x  y  v   theta
model.lb = np.array([-5, -1, -3, 0, 0,      0])
model.ub = np.array([+5, +1, 0,  3, 2, +np.inf])

xinit = np.array([-2, 0, 0, np.deg2rad(120)])
model.xinitidx = range(2, 6)
xfinal = np.array([0, np.deg2rad(0)])
model.xfinalidx = range(4, 6)

# set solver options
codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
codeoptions.maxit = 200     # Maximum number of iterations
codeoptions.printlevel = 2  # # Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 0    # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
solver = model.generate_solver(codeoptions)

# Set initial guess to start solver from:
x0i = (model.lb + model.ub) / 3.0
x0 = np.transpose(np.tile(x0i, (1, model.N))) # np.tile把x0i的矩阵 复制(1, model.N)个形状
problem = {"x0": x0,
           "xinit": xinit,
           "xfinal": xfinal}

# solve the NLP!
output, exitflag, info = solver.solve(problem)
# Make sure the solver has exited properly.
assert exitflag == 1, "bad exitflag"
print("FORCES took {} iterations and {} seconds to solve the problem.".format(info.it, info.solvetime))




# # load parameters
# p = parameters_vehicle2()
# g = 9.81  # [m/s^2]
#
# # set options --------------------------------------------------------------
# tStart = 0  # start time
# tFinal = 1  # start time
#
# delta0 = 0
# vel0 = 15
# Psi0 = 0
# dotPsi0 = 0
# beta0 = 0
# sy0 = 0
# initialState = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]  # initial state for simulation
# x0_KS = init_ks(initialState)  # initial state for kinematic single-track model
#
#
# def func_ks(x, t, u, p):
#     """
#     #x1 = x-position in a global coordinate system
#     #x2 = y-position in a global coordinate system
#     #x3 = steering angle of front wheels
#     #x4 = velocity in x-direction
#     #x5 = yaw angle
#
#     #u1 = steering angle velocity of front wheels
#     #u2 = longitudinal acceleration"""
#     f = vehicle_dynamics_ks(x, u, p)
#     return f
#
# def forcespro_solver():
#     """Variables are collected stage-wise into
#         z = [u1 u2 xPos yPos delta v phi].
# """
#     # define the model
#     model = forcespro.nlp.SymbolicModel()
#     model.N = 10  # horizon length
#     model.nvar = 7  # number of variables
#     model.neq = 5  # number of equality constraints
#     model.nh = 2  # number of inequality constraint functions
#     model.npar = 2  # number of runtime parameters
#
#     model.objective = lambda z: 100 * casadi.fabs(z[2] -0.) \
#                                 + 100 * casadi.fabs(z[3] - 3.) \
#                                 + 0.1 * z[0]**2 + 0.01 * z[1]**2
#     integrator_stepsize = 0.1
#     model.eq = lambda z: forcespro.nlp.integrate(func_ks, z[2:7], z[0:2],
#                                                  integrator=forcespro.nlp.integrators.RK4,
#                                                  stepsize=integrator_stepsize)
#     p = parameters_vehicle2()
#     model.lb = np.array([p.steering.v_min, - p.longitudinal.a_max, -3., 0., p.steering.min, p.longitudinal.v_min, np.deg2rad(-40.)])
#     model.ub = np.array([p.steering.v_max, p.longitudinal.a_max, 0., 3., p.steering.max, p.longitudinal.v_max, np.deg2rad(+40.)])
