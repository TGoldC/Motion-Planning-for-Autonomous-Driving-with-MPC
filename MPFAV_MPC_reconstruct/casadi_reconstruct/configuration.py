import numpy as np 
import casadi as ca
import matplotlib.pyplot as plt
#commonroad-io
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from vehiclemodels.utils.vehicle_dynamics_ks_cog import vehicle_dynamics_ks_cog
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from vehiclemodels.vehicle_dynamics_std import vehicle_dynamics_std
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route
from commonroad.visualization.mp_renderer import MPRenderer


class Vehicle_dynamics:
    def __init__(self,):
        p = parameters_vehicle2()
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


    def KS(self, states, controls, type ='casadi'):
        if type == 'casadi':
            f = ca.vertcat(states[3]*ca.cos(states[4]))
            f = ca.vertcat(f, states[3]*ca.sin(states[4]))
            f = ca.vertcat(f, controls[0])
            f = ca.vertcat(f, controls[1])
            f = ca.vertcat(f, (ca.tan(states[2])*states[3])/self.l)
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

class ReferencePath:

    def __init__(self, scenario, planning_problem):
        self.scenario = scenario
        self. planning_problem = planning_problem
        self.reference_path = self._generate_reference_path()

        self.desired_velocity, self.delta_t = self.get_desired_velocity_and_delta_t()
        self.accumulated_distance_in_reference_path = self._accumulated_distance_in_reference_path()

    def _generate_reference_path(self):
        """
        position_init: 1D array (x_pos, y_pos); the initial position of the planning problem
        reference_path: the output of route planner, which is considered as reference path
        """
        route_planer = RoutePlanner(self.scenario, self.planning_problem, backend=RoutePlanner.Backend.NETWORKX)
        candidate_route = route_planer.plan_routes()
        reference_path = candidate_route.retrieve_best_route_by_orientation().reference_path  # 800*2
        return reference_path

    def plot(self, plot_scenario_and_planning_problem=True, plot_route=True):
        if plot_scenario_and_planning_problem:
            renderer = MPRenderer(figsize=(12, 12))
            self.scenario.draw(renderer)
            self.planning_problem.draw(renderer)
            renderer.render()
            plt.show()
        if plot_route:
            visualize_route(self.reference_path, draw_route_lanelets=True, draw_reference_path=True, size_x=6)

    def _accumulated_distance_in_reference_path(self):
        return np.cumsum(np.linalg.norm(np.diff(self.reference_path, axis=0), axis=1))

    def find_nearest_point_in_reference_path(self, t_past):  # 其实可以直接resampled the reference path, 之后只用这个resampled的
        """
        :param desired_velocity: velocity in the goal region, which is defined in planning problem
        :param t_past: past time from initial position to current step
        :return: index in reference path. The distance from initial position to this index position is nearest distance
        to desired distance.
        """
        desired_distance = self.desired_velocity * t_past
        index = np.abs(self.accumulated_distance_in_reference_path - desired_distance).argmin()
        return index

    def resample_reference_path(self):
        num_path_points = self.reference_path.shape[0]
        cumsum_distance = self._accumulated_distance_in_reference_path()
        sim_time = int(np.abs(cumsum_distance[-1]) / self.desired_velocity)
        iter_length = int(sim_time / self.delta_t)
        interval_lenth = int(num_path_points / iter_length)
        resampled_reference_path = self.reference_path[::interval_lenth, :]
        #if resampled_reference_path[0]>iter_length:
        #    resampled_reference_path = resampled_reference_path
        return resampled_reference_path, iter_length, sim_time


    def get_init_value(self):

        if hasattr(self.planning_problem.initial_state, 'acceleration'):
            init_acceleration = self.planning_problem.initial_state.acceleration
        else:
            init_acceleration = 0.

        if hasattr(self.planning_problem.initial_state, 'orientation'):
            init_orientation = self.planning_problem.initial_state.orientation
        else:
            init_orientation = 0

        if hasattr(self.planning_problem.initial_state, "position"):
            init_position = self.planning_problem.initial_state.position
        else:
            init_position = np.array([0, 0])

        return init_position, init_acceleration, init_orientation

    def get_desired_velocity_and_delta_t(self):
        # goal state configuration    这一部分不应该写在这个class里面。要参考qp planner来写,写在main里面
        if hasattr(self.planning_problem.goal.state_list[0], 'velocity'):
            if self.planning_problem.goal.state_list[0].velocity.start != 0:
                desired_velocity = (self.planning_problem.goal.state_list[0].velocity.start
                                    + self.planning_problem.goal.state_list[0].velocity.end) / 2
            else:
                desired_velocity = (self.planning_problem.goal.state_list[0].velocity.start
                                    + self.planning_problem.goal.state_list[0].velocity.end) / 2
        else:
            desired_velocity = self.planning_problem.initial_state.velocity

        if not hasattr(self.scenario, 'dt'):
            delta_t = 0.1  # default time step
        else:
            delta_t = self.scenario.dt

        return desired_velocity, delta_t

