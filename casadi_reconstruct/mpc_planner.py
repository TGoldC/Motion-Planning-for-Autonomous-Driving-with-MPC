import os

from numpy.core.fromnumeric import shape
import casadi as ca
import numpy as np
import time

from optimizer_casadi import MPC_car
from configuration import ReferencePath

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from commonroad.common.file_reader import CommonRoadFileReader
#for resample
import commonroad_dc.pycrccosy as pycrccosy

class MPC_Planner:
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
    def resample_refence_trajectory(self, reference_path,sum_distance, sim_time):
        step = sum_distance/(sim_time/T-2)
        new_polyline = pycrccosy.Util.resample_polyline(reference_path, step)
        return new_polyline

if __name__ == '__main__':
    #define prediction horizon and sampling time [s]
    N = 10
    T = 0.1 
    #retrieve reference path from ReferencePath
    path_scenario = "/home/zehua/commonroad/commonroad-route-planner/scenarios/"
    id_scenario = "USA_Peach-2_1_T-1.xml"
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario + id_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    reference_path_instance = ReferencePath(scenario, planning_problem)
    desired_velocity = reference_path_instance.desired_velocity
    distance = reference_path_instance._accumulated_distance_in_reference_path()
    sum_distance = distance[-1]- distance[0]


    resampled_reference_path, iter_length, sim_time = reference_path_instance.resample_reference_path()
    reference_path = reference_path_instance._generate_reference_path()
    new_polyline = pycrccosy.Util.resample_polyline(reference_path, sum_distance/(sim_time/T-2))
    print(sim_time)
    print(reference_path.shape)
    print(new_polyline.shape)
    print(resampled_reference_path.shape)
    #print(iter_length)
    #print(sim_time)
    init_position, init_acceleration, init_orientation = reference_path_instance.get_init_value()
    #get[800, 2]points from route planner, take [100, 2] points each 8 s
    #waypoints = ReferencePath(path_scenario="/home/zehua/commonroad/commonroad-route-planner/scenarios/",
    #                            id_scenario="USA_Peach-2_1_T-1.xml").reference_path
    #ref_path = waypoints[::4, :]
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
    # initial state
    x0 = np.array([init_position[0], init_position[1], 0.0, init_acceleration, init_orientation]).reshape(-1, 1)
    # initial controls
    u0 = np.array([0.0, 0.0]*N).reshape(-1, 2)
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
    for i in range(int(sim_time/T)):
        xs = np.array([new_polyline[i, 0], new_polyline[i, 1], 0, desired_velocity, 0]).reshape(-1,1)
        ## set parameter
        c_p = np.concatenate((x0, xs))
        init_control = ca.reshape(u0, -1, 1)
        t_ = time.time()
        res = mpc_obj.solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
        index_t.append(time.time()- t_)
        u_sol = ca.reshape(res['x'], num_controls, N) 
        #+ np.random.normal(0,1,20).reshape(num_controls, N) # add a gaussian noise 
        #print(u_sol.shape)
        ff_value = mpc_obj.ff(u_sol, c_p) # [n_states, N+1]
        x_c.append(ff_value)
        u_c.append(u_sol[:, 0])
        t_c.append(t0)
        t0, x0, u0 = mpc_obj.shift_movement(t0, x0, u_sol, mpc_obj.f)
        traj.append(x0)
        ref.append(xs)
        #print(xs)
        mpciter = mpciter + 1
    t_v = np.array(index_t)
    print(t_v.mean())
    print((time.time() - start_time)/(mpciter))
    
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