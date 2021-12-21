import numpy as np 
import casadi as ca
from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from vehiclemodels.utils.vehicle_dynamics_ks_cog import vehicle_dynamics_ks_cog
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from vehiclemodels.vehicle_dynamics_std import vehicle_dynamics_std


class Vehicle_dynamics(object):
    def __init__(self, p=parameters_vehicle2()):
        self.l = p.a + p.b
        self.g = 9.81
        self.mu = p.tire.p_dy1 
        self.C_Sf = -p.tire.p_ky1/p.tire.p_dy1  
        self.C_Sr = -p.tire.p_ky1/p.tire.p_dy1  
        self.lf = p.a 
        self.lr = p.b 
        self.h = p.h_s 
        self.m = p.m
        self.I = p.I_z

    @classmethod
    def KS_casadi(self, x, u):
        """Defines dynamics of the car, i.e. equality constraints.
        parameters:
        state x = [xPos,yPos,delta,v,psi]
        input u = [deltaDot,aLong]
        """
        # calculate dx/dt
        p = parameters_vehicle2()
        l = p.a + p.b
        return ca.vertcat(x[3] * ca.cos(x[4]),
                              x[3] * ca.sin(x[4]),
                              u[0],
                              u[1],
                              x[3] / l * ca.tan(x[2]))


    def KS(self, states, controls, type ='casadi'):
        if type == 'casadi':
            f = ca.vertcat(states[3]*ca.cos(states[4]))
            f = ca.vertcat(f, states[3]*ca.sin(states[4]))
            f = ca.vertcat(f, controls[0])
            f = ca.vertcat(f, controls[1])
            f = ca.vertcat(f, states[3]*ca.tan(states[2])/self.l)
        elif type == 'scipy':
            f = vehicle_dynamics_ks(states, controls, self.p)
        return f

    def ST(self, x, u, type ='casadi'):
        if type == 'casadi':
            if abs(x[3]) < 0.1:
                x_ks = [x[0],  x[1],  x[2],  x[3],  x[4]]
                f_ks = vehicle_dynamics_ks_cog(x_ks, u, self.p)
                f = [f_ks[0],  f_ks[1],  f_ks[2],  f_ks[3],  f_ks[4]]
                d_beta = (self.lr * u[0]) / (self.l*ca.cos(x[2])**2 * (1 + (ca.tan(x[2])**2 * self.lr/self.l)**2))
                dd_psi = 1/self.l * (u[1]*ca.cos(x[6])*ca.tan(x[2]) -
                                  x[3]*ca.sin(x[6])*d_beta*ca.tan(x[2]) +
                                  x[3]*ca.cos(x[6])*u[0]/ca.cos(x[2])**2)
                f.append(dd_psi)
                f.append(d_beta)
            else:
                f = [x[3]*ca.cos(x[6] + x[4]), 
                    x[3]*ca.sin(x[6] + x[4]), 
                    u[0], 
                    u[1], 
                    x[5], 
                    -self.mu*self.m/(x[3]*self.I*(self.lr+self.lf))*(self.lf**2*self.C_Sf*(self.g*self.lr-u[1]*self.h) + self.lr**2*self.C_Sr*(self.g*self.lf + u[1]*self.h))*x[5] \
                        +self.mu*self.m/(self.I*(self.lr+self.lf))*(self.lr*self.C_Sr*(self.g*self.lf + u[1]*self.h) - self.lf*self.C_Sf*(self.g*self.lr - u[1]*self.h))*x[6] \
                        +self.mu*self.m/(self.I*(self.lr+self.lf))*self.lf*self.C_Sf*(self.g*self.lr - u[1]*self.h)*x[2], 
                    (self.mu/(x[3]**2*(self.lr+self.lf))*(self.C_Sr*(self.g*self.lf + u[1]*self.h)*self.lr - self.C_Sf*(self.g*self.lr - u[1]*self.h)*self.lf)-1)*x[5] \
                        -self.mu/(x[3]*(self.lr+self.lf))*(self.C_Sr*(self.g*self.lf + u[1]*h) + self.C_Sf*(self.g*self.lr-u[1]*self.h))*x[6] \
                        +self.mu/(x[3]*(self.lr+self.lf))*(self.C_Sf*(self.g*self.lr-u[1]*self.h))*x[2]]
        elif type == 'scipy':
            f = vehicle_dynamics_st(x, u, self.p)
        return f

    def func_STD(x, u, p):
        f = vehicle_dynamics_std(x, u, p)
        return f

