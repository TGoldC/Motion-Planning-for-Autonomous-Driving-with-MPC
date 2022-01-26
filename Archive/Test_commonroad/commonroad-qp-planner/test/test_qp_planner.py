import os.path
import unittest
import numpy as np
import yaml
import matplotlib.pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_qp_planner.qp_planner import QPPlanner, QPLongReference, QPLongState
from commonroad_qp_planner.initialization import set_up
from commonroad_qp_planner.constraints import LatConstraints, LonConstraints, TVConstraints
from commonroad_qp_planner.utils import open_scenario, plot_result

from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker, create_collision_object

class TestQPOptimizer(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        config_name = "config_files/config_ZAM_Tutorial-1_2_T-1.yaml"
        with open(config_name, 'r') as stream:
            try:
                self.settings = yaml.load(stream, Loader=yaml.Loader)
            except yaml.YAMLError as exc:
                print(exc)
        scenario_path = os.path.abspath(os.path.join(os.path.abspath(__file__), "../../scenarios/ZAM_Tutorial-1_2_T-1.xml"))
        self.scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()
        self.planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    def test_qp_optimizer(self):
        ################
        # Configuration
        ################
        # set up necessary configurations, e.g., curvilinear coordinate system, collision checker, etc.
        vehicle_configuration = set_up(self.settings, self.scenario, self.planning_problem)
        qp_planner = QPPlanner(self.scenario,
                               self.planning_problem,
                               self.settings["general_planning_settings"]["time_horizon"],
                               vehicle_configuration,
                               verbose=True)
        #########################################
        # long. and lat. time-variant constraints
        #########################################
        # create constraints for longitudinal minimum and maximum position
        s_min = []  # minimum position constraint
        s_max = []  # maximum position constraint
        # extract obstacle from scenario
        dyn_obstacles = self.scenario.dynamic_obstacles
        # go through obstacle list and distinguish between following and leading vehicle
        # only works for scenarios with one following and one leading vehicle
        for o in dyn_obstacles:
            o_initial_curvilinear = vehicle_configuration.\
                curvilinear_coordinate_system.convert_to_curvilinear_coords(o.initial_state.position[0],
                                                                            o.initial_state.position[1])
            if o_initial_curvilinear[0] < qp_planner.initial_state.position[0]:
                print('Following vehicle id={}'.format(o.obstacle_id))
                prediction = o.prediction.trajectory.state_list
                for p in prediction:
                    o_pos_curvilinear = vehicle_configuration. \
                        curvilinear_coordinate_system.convert_to_curvilinear_coords(p.position[0],
                                                                                    p.position[1])
                    s_min.append(o_pos_curvilinear[0] + o.obstacle_shape.length / 2. + 2.5)
            else:
                print('Leading vehicle id={}'.format(o.obstacle_id))
                prediction = o.prediction.trajectory.state_list
                for p in prediction:
                    o_pos_curvilinear = vehicle_configuration. \
                        curvilinear_coordinate_system.convert_to_curvilinear_coords(p.position[0],
                                                                                    p.position[1])
                    s_max.append(o_pos_curvilinear[0] - o.obstacle_shape.length / 2. - 2.5)
        s_min = np.array(s_min)
        s_max = np.array(s_max)
        c_long = LonConstraints.construct_constraints(s_min, s_max, s_min, s_max)
        d_min_single = np.full_like(s_min, -2.5)
        d_max_single = np.full_like(s_max, 2.5)
        d_min = np.array((d_min_single, d_min_single, d_min_single)).transpose()
        d_max = np.array((d_max_single, d_max_single, d_max_single)).transpose()
        c_lat = LatConstraints.construct_constraints(d_min, d_max, d_min, d_max)
        # construct time variant constraints
        c_tv = TVConstraints(c_long, c_lat)
        ######################################################
        # reference for the long. optimization (set manually)
        ######################################################
        v_ref = self.settings["vehicle_settings"][self.planning_problem.planning_problem_id]["desired_speed"]
        x_ref = list()
        for i in range(len(s_min)):
            x_ref.append(QPLongState(0, v_ref, 0., 0., 0.))
        reference = QPLongReference(x_ref)

        ######################################
        # trajectory generation and conversion
        ######################################
        trajectory = qp_planner.plan_trajectories(c_tv, reference)
        trajectory_cartesian = qp_planner.transform_trajectory_to_cartesian_coordinates(trajectory)
        ego_vehicle = trajectory_cartesian.convert_to_cr_ego_vehicle(
            vehicle_configuration.width, vehicle_configuration.length,
            vehicle_configuration.wheelbase, vehicle_configuration.vehicle_id)
        ############################
        # visualize updated scenario
        ############################
        fig, axs = plt.subplots(3)
        plot_result(self.scenario, ego_vehicle, axs[0])

        # s_limit
        axs[1].plot(list(range(len(s_min))), s_min, color="red")
        axs[1].plot(list(range(len(s_max))), s_max, color="red")
        axs[1].plot(list(range(len(trajectory.states) - 1)),
                    [state.position[0] for state in trajectory.states[1:]], color="black")
        axs[1].set_xlabel("time step")
        axs[1].set_ylabel("s")

        # d_limit
        axs[2].plot(list(range(len(d_min))), d_min, color="red")
        axs[2].plot(list(range(len(d_max))), d_max, color="red")
        axs[2].plot(list(range(len(trajectory.states) - 1)),
                    [state.position[1] for state in trajectory.states[1:]], color="black")
        axs[2].set_xlabel("time step")
        axs[2].set_ylabel("d")

        plt.show()
        collision_checker = create_collision_checker(self.scenario)
        collision_object = create_collision_object(ego_vehicle)

        self.assertFalse(collision_checker.collide(collision_object))

