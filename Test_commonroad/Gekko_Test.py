# Imports
import numpy as np
from gekko import GEKKO
import matplotlib.pyplot as plt

m = GEKKO() # initialize the model

# Set the constants
lwb = 2.471 # wheelbase of vehicle 3
amax = 11.5 # maximum acceleration of vehicle 3
mindelta = -1.023 # minimum steering angle
maxdelta = 1.023 # maximum steering angle
mindeltav = -0.4 # minimum steering velocity
maxdeltav = 0.4 # maximum steering velocity
minv = -11.2 # minimum velocity
maxv = 41.7 #maximum velocity

lwba = m.Const(value=lwb)
amaxa = m.Const(value=amax)
mindeltaa = m.Const(value=mindelta)
maxdeltaa = m.Const(value=maxdelta)
mindeltava = m.Const(value=mindeltav)
maxdeltava = m.Const(value=maxdeltav)
minva = m.Const(value=minv)
maxva = m.Const(value=maxv)

# Set time
startt = 0.0 # select start time of the simulation
endt = 10.0 # select end time of the simulation
dt = 100 # select discretization of the simulation
m.time = np.linspace(startt, endt, dt) # set the time (from start to endt in dt steps)
finalt = int(dt*endt/(endt-startt))-1 # compute the discretization step belonging to the goalt

# Set initial and final state
startstate = [0.0, 0.0, 0.0, 0.0, 0.0]
finalstate = [10.0, 0.0, 0.0, 2.0, 0.0]
#startstate = [1.0, 1.0, -0.2, 1.0, -0.3] # [sx, sy, delta, v, psi]
#finalstate = [10.0, 8.0, 0.3, 0.0, 2.0] # [sx, sy, delta, v, psi]

# Create the state variables
# x1 = sx (position in x-direction)
# x2 = sy (position in y-direction)
# x3 = delta (steering angle)
# x4 = v (velocity in x-direction)
# x5 = psi (heading)

sxa = m.SV(value=startstate[0]) # state variable
sya = m.SV(value=startstate[1])
deltaa = m.SV(value=startstate[2], lb=mindeltaa, ub=maxdeltaa)
va = m.SV(value=startstate[3], lb=minva, ub=maxva)
psia = m.SV(value=startstate[4])

# Create the input variables
# u1 = vdelta (velocity of steering angle)
# u2 = longa (longitudinal acceleration)

vdeltaa = m.CV(value=0, lb=mindeltava, ub=maxdeltava)  # control variable
longaa = m.CV(value=0)

# Define the state space model
# differential equations
m.Equation(sxa.dt() == va * m.cos(psia))
m.Equation(sya.dt() == va * m.sin(psia))
m.Equation(deltaa.dt() == vdeltaa)
m.Equation(va.dt() == longaa)
m.Equation(psia.dt() == (va/lwba)*m.tan(deltaa))

# Add constraint
# Friction circle
m.Equation(m.sqrt(longaa**2+(va*psia.dt())**2) <= amax)

# Add Objectives
m.Obj(1*vdeltaa**2) # minimize steering velocity
m.Obj(1*longaa**2) # minimize longitudinal acceleration

# Fix the final values
#m.fix(sxa, pos = finalt, val=finalstate[0])
#m.fix(sya, pos = finalt, val=finalstate[1])
#m.fix(deltaa, pos = finalt, val=finalstate[2])
#m.fix(va, pos = finalt, val=finalstate[3])
#m.fix(psia, pos = finalt, val=finalstate[4])

p = np.zeros(len(m.time))
p[finalt] = 1000    # p是一个一维数组，只有最后一项是1000
final = m.Param(p)
m.Minimize(final*(sxa-finalstate[0])**2)    # 相当于只对终了时刻的状态 需要它越接近0越好
m.Minimize(final*(sya-finalstate[1])**2)
m.Minimize(final*(deltaa-finalstate[2])**2)
m.Minimize(final*(va-finalstate[3])**2)
m.Minimize(final*(psia-finalstate[4])**2)

# Solve
m.options.IMODE = 6 # MPC
m.solve()

# Plot trajectory
plt.figure()
plt.plot(sxa.value, sya.value)
plt.axis('equal')
plt.title('Trajectory')
plt.savefig('Vehicle_traj.png')

# Plot state variables
plt.figure()
plt.suptitle('State Variables', fontsize=16)
# plot steering angle
plt.subplot(131)
plt.title('Steering Angle/Delta')
plt.plot(m.time, deltaa.value)
# plot velocity
plt.subplot(132)
plt.title('Velocity')
plt.plot(m.time, va.value)
# plot yaw angle
plt.subplot(133)
plt.title('Orientation/Psi')
plt.plot(m.time, psia.value)
plt.subplots_adjust(wspace=0.5)
plt.savefig('Vehicle_states.png')

# plot input
plt.figure()
plt.suptitle('Input', fontsize=16)
plt.subplot(121)
plt.title('Velocity of Steering Angle/v delta')
plt.plot(m.time, vdeltaa.value)
plt.subplot(122)
plt.title('Longitudinal Acceleration/long a')
plt.subplots_adjust(wspace=0.5)
plt.plot(m.time, longaa.value)
plt.savefig('Vehicle_input.png')
plt.show()
