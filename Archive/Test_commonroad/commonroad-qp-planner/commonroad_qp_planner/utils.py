import os.path

from commonroad.common.file_reader import CommonRoadFileReader

from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.param_server import ParamServer


def open_scenario(settings):
    scenario_path = os.path.abspath(os.path.join(__file__))
    crfr = CommonRoadFileReader(
        scenario_path + settings['scenario_settings']['scenario_name'] + '.xml')
    scenario, planning_problem_set = crfr.open()
    return scenario, planning_problem_set


def plot_result(scenario, ego_vehicle, ax):

    rnd = MPRenderer(figsize=(20, 10), ax=ax)
    scenario.draw(
        rnd,
        draw_params=ParamServer({"occupancy": {"draw_occupancies": 1}})
    )
    # plot ego trajectory
    ego_vehicle.draw(rnd,
                     draw_params=ParamServer(
                         {"occupancy": {
                             "draw_occupancies": 1,
                             "shape": {"rectangle": {
                                 "facecolor": "black",
                                 "edgecolor": "black"}
                             }},
                             "dynamic_obstacle":
                                 {"vehicle_shape": {
                                     "occupancy": {
                                         "shape": {"rectangle": {
                                             "facecolor": "black",
                                             "edgecolor": "black"}
                                         }}}}}))

    ego_vehicle.prediction.trajectory.draw(rnd, draw_params={
        "trajectory": {"shape": {"rectangle": {"facecolor": "black"}}}})
    rnd.render()
