import os.path
import unittest
import yaml
from commonroad.common.file_reader import CommonRoadFileReader
from MPC_Planner.configuration import Configuration
from MPC_Planner.mpc_planner import MPCPlanner


class TestMPCPlanner(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        config_name = "config_files/config_USA_Lanker-2_18_T-1.yaml"
        with open(config_name, 'r') as stream:
            try:
                self.settings = yaml.load(stream, Loader=yaml.Loader)
            except yaml.YAMLError as exc:
                print(exc)
        path_scenario = os.path.abspath(os.path.join(os.path.abspath(__file__), "../../scenarios/"))
        id_scenario = "/USA_Lanker-2_18_T-1.xml"  # "ZAM_Tutorial_Urban-3_2.xml"
        self.scenario, self.planning_problem_set = CommonRoadFileReader(path_scenario + id_scenario).open()
        self.planning_problem = list(self.planning_problem_set.planning_problem_dict.values())[0]

    def test_mpc_optimizer(self):
        # configuration
        config = Configuration(self.settings, self.scenario, self.planning_problem).configuration
        mpc_planner = MPCPlanner(scenario=self.scenario,
                                 planning_problem=self.planning_problem,
                                 configuration=config,
                                 predict_horizon=self.settings["general_planning_settings"]["predict_horizon"])

        # plan and save the result-plots
        mpc_planner.plan("forcespro")


if __name__ == '__main__':
    unittest.main()
