from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker, create_collision_object
from MPC_Planner.configuration import Configuration
from MPC_Planner.mpc_planner import MPCPlanner
import os.path
import unittest
import yaml
import sys
sys.path.append("..")


class TestMPCPlanner(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        config_name = "config_files/config_CA_ZAM_Over-1_1.yaml"  # config_LF_ZAM_Over-1_1.yaml  or  config_LF_USA_Lanker-2_18_T-1
        with open(config_name, 'r') as stream:
            try:
                self.settings = yaml.load(stream, Loader=yaml.Loader)
            except yaml.YAMLError as exc:
                print(exc)
        path_scenario = os.path.abspath(os.path.join(os.path.abspath(__file__), "../../scenarios/"))
        id_scenario = "/" + self.settings["scenario_settings"]["scenario_name"] + ".xml"
        self.scenario, self.planning_problem_set = CommonRoadFileReader(path_scenario + id_scenario).open()
        self.planning_problem = list(self.planning_problem_set.planning_problem_dict.values())[0]

    def test_mpc_optimizer(self):
        # configuration
        config = Configuration(self.settings, self.scenario, self.planning_problem).configuration
        mpc_planner = MPCPlanner(scenario=self.scenario,
                                 planning_problem=self.planning_problem,
                                 configuration=config,
                                 predict_horizon=self.settings["general_planning_settings"]["predict_horizon"])

        ego_vehicle_trajectory, ego_vehicle = mpc_planner.plan()
        # create collision checker from scenario
        cc = create_collision_checker(self.scenario)
        # create ego vehicle collision object
        ego_vehicle_co = create_collision_object(ego_vehicle)
        # create the road boundary
        _, road_boundary = create_road_boundary_obstacle(self.scenario)
        # add road boundary to collision checker
        cc.add_collision_object(road_boundary)
        
        # check if ego vehicle collides
        res = cc.collide(ego_vehicle_co)
        print('Collision between the ego vehicle and the road boundary: %s' % res)


if __name__ == '__main__':
    unittest.main()
