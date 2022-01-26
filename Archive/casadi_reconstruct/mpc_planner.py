import os
import time
import numpy as np
import casadi as ca
#import casadi optimizer
from optimizer_casadi import MPC_car

#commonroad-io
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route
from commonroad.visualization.mp_renderer import MPRenderer

# commonroad-curvilinear-coordinatesystem
import commonroad_dc.pycrccosy as pycrccosy
from commonroad_dc.geometry.util import (chaikins_corner_cutting, compute_curvature_from_polyline, resample_polyline,
                                         compute_pathlength_from_polyline, compute_orientation_from_polyline)

#plot and animation
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class MPC_Planner():
    def __init__(self, scenario, planning_problem):
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.reference_path = self._generate_reference_path()
        self.desired_velocity, self.delta_t = self.get_desired_velocity_and_delta_t()

    def get_desired_velocity_and_delta_t(self):
        # goal state configuration
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

    def get_init_values(self):

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

        if hasattr(self.planning_problem, "velocity"):
            init_velocity = self.planning_problem.initial_state.velocity
        else:
            init_velocity = 0

        return init_position, init_velocity, init_acceleration, init_orientation

    def _generate_reference_path(self):
        """
        position_init: 1D array (x_pos, y_pos); the initial position of the planning problem
        reference_path: the output of route planner, which is considered as reference path
        """
        route_planer = RoutePlanner(self.scenario, self.planning_problem, backend=RoutePlanner.Backend.NETWORKX)
        candidate_route = route_planer.plan_routes()
        reference_path = candidate_route.retrieve_best_route_by_orientation().reference_path  # 800*2
        return reference_path
    
    def resample_reference_path(self):
        #sum_distance = pycrccosy.Util.compute_polyline_length(self.reference_path)
        #reference_path = np.array(pycrccosy.Util.chaikins_corner_cutting(self.reference_path))
        step = self.desired_velocity * self.delta_t
        resampled_reference_path = pycrccosy.Util.resample_polyline(self.reference_path, step)
        iter_length = resampled_reference_path.shape[0]
        return resampled_reference_path, iter_length

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

        if hasattr(self.planning_problem, "velocity"):
            init_velocity = self.planning_problem.initial_state.velocity
        else:
            init_velocity = 0

        return init_position, init_velocity, init_acceleration, init_orientation

if __name__ == '__main__':
    #define prediction horizon
    N = 10
    #retrieve reference path from ReferencePath
    path_scenario = "/home/zehua/commonroad/commonroad-route-planner/scenarios/"
    id_scenario = "USA_Lanker-2_18_T-1.xml"
    #DEU_Gar-3_2_T-1
    #USA_Lanker-2_18_T-1 Xin
    #USA_Peach-2_1_T-1
    #ZAM_Tutorial_Urban-3_2.xml
    #ZAM_Zip-1_6_T-1.xml
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario + id_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    reference_path_instance = MPC_Planner(scenario, planning_problem)
    #get desired velocity [m/s] and sampling time[s]
    desired_velocity, delta_T = reference_path_instance.get_desired_velocity_and_delta_t()
    
    #set sampling time [s]
    T = delta_T

    resampled_reference_path, iter_length = reference_path_instance.resample_reference_path()
    reference_path = reference_path_instance.reference_path
    
    #compute orientation fro resampled reference path
    orientation = compute_orientation_from_polyline(resampled_reference_path)

    init_position, init_velocity, init_acceleration, init_orientation = reference_path_instance.get_init_value()
  
    # model parameters
    num_states = 5
    num_controls = 2
    #get MPC optimizer from optimizer_casadi
    mpc_obj = MPC_car(state_dim=num_states, T=0.1, N=N)
    lbg, ubg, lbx, ubx = mpc_obj.inequal_constraints(N)
    ##states constraints
    #lbg = []
    #ubg = []
    #for _ in range(N+1):
    #    lbg.append(-0.91)
    #    lbg.append(-13.9)
    #    ubg.append(0.91)
    #    ubg.append(45.8)
#
    ##control constraints
    #lbx = []
    #ubx = []
    #for _ in range(N):
    #    lbx.append(-0.4)
    #    ubx.append(0.4)
    #    lbx.append(-np.inf)
    #    ubx.append(np.inf)

    
    t0 = 0.0
    # set initial state
    x0 = np.array([init_position[0], init_position[1], 0.0, init_velocity, init_orientation]).reshape(-1, 1)
    # set initial controls
    u0 = np.array([0.0, init_acceleration]*N).reshape(-1, 2)
    #for saving data
    xs = 0
    x_c = [] 
    u_c = []
    t_c = [] 
    xx = []

    traj = []
    ref = []
    mpciter = 0
    start_time = time.time()
    index_t = []
    #simulation time

    #while( mpciter-sim_time/T<0.0 ):
    for i in range(iter_length):
        xs = np.array([resampled_reference_path[i, 0], resampled_reference_path[i, 1], 0, desired_velocity, orientation[i]]).reshape(-1,1)
        ## set parameter
        c_p = np.concatenate((x0, xs))
        init_control = ca.reshape(u0, -1, 1)
        t_ = time.time()
        res = mpc_obj.solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
        index_t.append(time.time()- t_)
        u_sol = ca.reshape(res['x'], num_controls, N) + np.random.normal(0,0.1,20).reshape(num_controls, N) # add a gaussian noise 20 numbers
        #print(u_sol.shape)
        ff_value = mpc_obj.ff(u_sol, c_p) # [n_states, N+1]
        x_c.append(ff_value)
        u_c.append(u_sol[:, 0])
        t_c.append(t0)
        t0, x0, u0 = mpc_obj.shift_movement(t0, x0, u_sol, mpc_obj.f)
        traj.append(x0)
        ref.append(xs)
        mpciter = mpciter + 1
    t_v = np.array(index_t)
    print(t_v.mean())
    #print((time.time() - start_time)/(mpciter))
    
    #plot reference path and actual path
    traj_s = np.array(traj)
    traj_r = np.array(ref)
    plt.figure()
    plt.plot(traj_s[:, 0], traj_s[:, 1])
    plt.plot(traj_r[:, 0], traj_r[:, 1])
    plt.plot()
    plt.axis('equal')
    plt.title('MPC_Path')
    plt.show()

    #animation
    x, y, z = traj_s[:, 0].flatten(), traj_s[:, 1].flatten(), traj_r[:, 2].flatten()
    helix = np.vstack((x, y, z))
    def update_lines(num, dataLines, lines) :
        for line, data in zip(lines, dataLines) :
            line.set_data(data[0:2, :num])
            line.set_3d_properties(data[2,:num])
        return lines
    # Attach 3D axis to the figure
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    plt.rcParams['font.size'] = 15
    #ax = fig.add_axes([0, 0, 1, 1], projection='3d')
    data = [helix]
    lines = [ax.plot(data[0][0,0:1], data[0][1,0:1], data[0][2,0:1], '.', c='red', markersize=5)[0]]

    # Set the axes properties
    ax.set_xlim3d([-30, 10])
    ax.set_xlabel('X')

    ax.set_ylim3d([-1.0, 80])
    ax.set_ylabel('Y')

    ax.set_zlim3d([0, 1.2])
    ax.set_zlabel('Z')

    ax.set_title('Vertical Trajectory')
    #print(traj_s[:,0].shape)
    ax.plot(traj_r[:, 0].flatten(), traj_r[:, 1].flatten(), traj_r[:, 2].flatten(), 'b')#x,y,z
    ani = FuncAnimation(fig, update_lines, fargs=(data, lines), interval=10, blit=False)
    plt.show()