# import matplotlib
# matplotlib.use('Qt5Agg')
# matplotlib.use('TkAgg')
import os
import time
import unittest
import warnings

import matplotlib.pyplot as plt
import numpy as np
import pytest
import math
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Rectangle, Circle, Polygon
from commonroad.geometry.shape import ShapeGroup
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import Occupancy
from commonroad.scenario.traffic_sign import TRAFFIC_SIGN_WITH_ADDITIONAL_VALUE, TrafficSignIDGermany, \
    TrafficSignIDRussia, TrafficSignIDSpain, TrafficSignIDZamunda, TrafficSignIDUsa, TrafficSignIDChina, TrafficSign, \
    TrafficSignElement
from commonroad.scenario.trajectory import State
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario

from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.param_server import ParamServer, write_default_params
from commonroad.visualization.traffic_sign import text_prop_dict, draw_traffic_light_signs


class TestVisualizationV2(unittest.TestCase):

    def setUp(self) -> None:
        self.rnd = MPRenderer()
        self.cwd_path = os.path.dirname(os.path.abspath(__file__))
        full_path = os.path.dirname(os.path.abspath(__file__))
        self.out_path = self.cwd_path + "/../.pytest_cache"
        self.ngsim_scen_1 = full_path + '/../test_scenarios/USA_Peach-4_8_T-1.xml'
        self.ngsim_scen_2 = full_path + '/../test_scenarios/USA_US101-4_1_T-1.xml'
        if not os.path.isdir(self.out_path):
            os.makedirs(self.out_path)
        else:
            for (dirpath, dirnames, filenames) in os.walk(self.out_path):
                for file in filenames:
                    if file.endswith('.mp4') or file.endswith('.gif') or file.endswith('.png'):
                        os.remove(os.path.join(dirpath, file))

    def test_primitive(self):
        params = ParamServer()

        rect = Rectangle(1, 2, np.array([1.5, 2.0]), math.pi * 0.25)
        poly = Polygon(np.array([[.2, .2], [.2, .4], [.3, .6], [.4, .4], [.4, .2], [.2, .2]]))
        circ = Circle(2, np.array([3, 3]))

        rect.draw(self.rnd, params, tuple())
        poly.draw(self.rnd, params, tuple())
        circ.draw(self.rnd, params, tuple())
        self.rnd.render()

    def test_scenario(self):

        # test draw_object for all possible object types
        scenario, planning_problem_set = CommonRoadFileReader(self.ngsim_scen_1).open()
        scenario: Scenario = scenario

        rnd = MPRenderer()
        with pytest.warns(None) as record_warnings:
            scenario.lanelet_network.draw(rnd)
            rnd.render()
            # visualization
            circ = Circle(2.0, np.array([10.0, 0.0]))
            obs = StaticObstacle(1000, ObstacleType.CAR, circ,
                                 initial_state=State(position=np.array([0, 0]), orientation=0.4))
            scenario.add_objects(obs)
            for obs in scenario.static_obstacles:
                obs.draw(rnd)
            rnd.render()

            draw_params = ParamServer({'scenario': {'dynamic_obstacle': {'occupancy': {'draw_occupancy': True}}}})
            rnd.draw_list(scenario.dynamic_obstacles, draw_params=draw_params)
            rnd.render()

            rnd.draw_list(scenario.dynamic_obstacles[0].prediction.occupancy_set)
            scenario.draw(rnd)
            rnd.render()

            rnd.clear()
            circ.draw(rnd)
            rnd.render()

            rnd.clear()
            rect = Rectangle(2.0, 4.0, np.array([2.0, 2.0]))
            rect.draw(rnd)
            rnd.render()

            rnd.clear()
            poly = Polygon(np.array([[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]]))
            poly.draw(rnd)
            rnd.render()

            sg = ShapeGroup([circ, rect])
            sg.draw(rnd)
            rnd.render()

            rnd.clear()
            occ = Occupancy(1, rect)
            occ.draw(rnd)
            rnd.render()

        plt.close('all')

    def test_focus_obstacle(self):

        # test draw_object for all possible object types
        scenario, planning_problem_set = CommonRoadFileReader(self.ngsim_scen_1).open()
        scenario: Scenario = scenario
        x0 = -40
        rnd = MPRenderer(plot_limits=[x0,40,-40,40])
        with pytest.warns(None) as record_warnings:
            scenario.lanelet_network.draw(rnd)

            draw_params = ParamServer({'focus_obstacle_id': 1239, "time_begin":0,
                                       'scenario': {'dynamic_obstacle': {'occupancy': {'draw_occupancy': True}}}})
            rnd.draw_list(scenario.dynamic_obstacles, draw_params=draw_params)
            assert rnd.plot_limits_focused[0] != 40

            draw_params = ParamServer({'focus_obstacle_id': False, "time_begin":0,
                                       'scenario': {'dynamic_obstacle': {'occupancy': {'draw_occupancy': True}}}})
            rnd.clear(keep_static_artists=True)
            rnd.draw_list(scenario.dynamic_obstacles, draw_params=draw_params)
            assert rnd.plot_limits_focused[0] == x0

        plt.close('all')

    def test_plotting_all_traffic_signs(self):
        traffic_sign_types = [TrafficSignIDZamunda, TrafficSignIDGermany, TrafficSignIDUsa,
                 TrafficSignIDSpain]
        rnd = MPRenderer()
        with warnings.catch_warnings(record=True) as w:
            for ts_type in traffic_sign_types:
                for v in ts_type:
                    # print(v)
                    ts = TrafficSign(100, [TrafficSignElement(v, ["test1","test2"])],
                                     first_occurrence={0},
                                     position=np.array([0.0, 0.0]),
                                virtual=False)
                    draw_traffic_light_signs(ts,draw_params=rnd.draw_params, call_stack=tuple(), rnd=rnd)

            self.assertEqual(len(w), 0, msg="The following warnings were raised:\n" +
                                            "\n".join(str(w_tmp.message) for w_tmp in w))

    def test_planning(self):
        # test draw_object for all possible object types
        full_path = os.path.dirname(os.path.abspath(__file__))
        # print(full_path)
        # filename = full_path +
        # '/../../../../../scenarios/cooperative/C-USA_Lanker-2_4_T-1.xml'
        filename = full_path + '/../test_scenarios/test_reading_all.xml'
        scenario, planning_problem_set = CommonRoadFileReader(filename).open()
        planning_problem_set: PlanningProblemSet = planning_problem_set

        with pytest.warns(None) as record_warnings:
            planning_problem_set.draw(self.rnd)
            self.rnd.render()
            self.rnd.clear()
            problem = list(planning_problem_set.planning_problem_dict.values())[0]
            problem.draw(self.rnd)
            problem.goal.draw(self.rnd)
            problem.initial_state.draw(self.rnd)
            self.rnd.render()

        assert (len(record_warnings) == 0)
        plt.close('all')

    def test_trajectory_unique_colors(self):
        scenario, planning_problem_set = CommonRoadFileReader(self.ngsim_scen_2).open()
        traj = list(map(lambda x: x.prediction.trajectory, scenario.dynamic_obstacles))
        params = {'trajectory': {'unique_colors': True}}
        scenario.lanelet_network.draw(self.rnd)
        self.rnd.draw_list(scenario.dynamic_obstacles)
        self.rnd.draw_trajectories(traj, params, tuple())
        self.rnd.render(show=True)

    def test_visual_appearance(self):
        tt = 0
        nrun = 20
        plt.close('all')
        # plt.ioff()
        # set_non_blocking()
        scenario, planning_problem_set = CommonRoadFileReader(self.ngsim_scen_2).open()
        scenario: Scenario = scenario
        # plt.autoscale(False)
        plt.style.use('classic')
        inch_in_cm = 2.54
        figsize = [20, 15]
        plt.figure(figsize=(figsize[0] / inch_in_cm, figsize[1] / inch_in_cm))
        plt.gca().set(title='occupancies should be be plotted with opacity, '
                            'plot limits: [-50,60,-50,50]')
        plt.gca().autoscale_view(False, False, False)
        self.rnd = MPRenderer(plot_limits=[-50, 60, -50, 50])
        for i in range(0, nrun):
            t1 = time.time()
            draw_params = {
                'planning_problem_set': {'draw_ids': [list(planning_problem_set.planning_problem_dict.keys())[0]]},
                'time_begin': 15, 'time_end': 25,
                'dynamic_obstacle': {'occupancy': {'draw_occupancies': 0, 'shape': {'rectangle': {'facecolor': 'g'}}}}}
            scenario.draw(self.rnd, draw_params=draw_params)
            # plt.tight_layout()
            planning_problem_set.draw(self.rnd, draw_params=draw_params)
            self.rnd.render(show=False, filename='/tmp/{}.png'.format(i))
            self.rnd.clear()
            tt += time.time() - t1  # plt.close()

        print('time: {}'.format(tt / nrun))

    def test_traffic_sign_plotting_properties(self):
        vis_dict = text_prop_dict()
        for t_id in TRAFFIC_SIGN_WITH_ADDITIONAL_VALUE:
            self.assertTrue(TrafficSignIDGermany[t_id].value in vis_dict,
                            f"Create plot settings for traffic sign {t_id} with ID {TrafficSignIDGermany[t_id].value}")

    def test_stylesheet(self):
        # Write default params to file
        json_filename = 'test_params.json'
        write_default_params(json_filename)
        # Now modify stylesheet file and read it
        params = ParamServer.from_json(json_filename)
        # Use for drawing
        scenario, planning_problem_set = CommonRoadFileReader(self.ngsim_scen_2).open()
        scenario.draw(self.rnd, draw_params=params)

    # Deactivated as ffmpeg not installe on CI
    def test_video(self):
        scenario, _ = CommonRoadFileReader(self.ngsim_scen_1).open()  #
        t0 = time.time()
        self.rnd.plot_limits = [-40,40,-40,40]
        for _ in range(1):
            self.rnd.create_video([scenario], os.path.join(self.out_path, str(scenario.scenario_id) + ".mp4"),
            draw_params={'time_begin': 0, 'time_end': 10,
                         "focus_obstacle_id": 520,
                         "dynamic_obstacle": {"show_label": False,
                                              "draw_icon": True,
                                              "draw_shape": True}})

        print(time.time() - t0)

    # def test_visualize_all_scenarios(self):
    #     scenarios_2020a = "TODO"
    #     scenarios_2018b = "TODO"
    #
    #     factory_2020a = scenarios_2020a + "/scenario-factory"
    #     hand_crafted_2020a = scenarios_2020a + "/hand-crafted"
    #     ngsim_lankershim_2020a = scenarios_2020a + "/NGSIM/Lankershim"
    #     ngsim_us101_2020a = scenarios_2020a + "/NGSIM/US101"
    #     ngsim_peachtree_2020a = scenarios_2020a + "/NGSIM/Peachtree"
    #     bicycle_2020a = scenarios_2020a + "/THI-Bicycle"
    #
    #     # cooperative_2018b = scenarios_2018b + "/cooperative"
    #     # bicycle_2018b = scenarios_2018b + "/THI-Bicycle"
    #     # sumo_2018b = scenarios_2018b + "/SUMO"
    #     # hand_crafted_2018b = scenarios_2018b + "/hand-crafted"
    #     # ngsim_lankershim_2018b = scenarios_2018b + "/NGSIM/Lankershim"
    #     # ngsim_us101_2018b = scenarios_2018b + "/NGSIM/US101"
    #     # ngsim_peachtree_2018b = scenarios_2018b + "/NGSIM/Peachtree"
    #
    #     rnd = MPRenderer()
    #     # for scenario in os.listdir(hand_crafted_2018b):
    #     #     full_path = hand_crafted_2018b + "/" + scenario
    #     #     print(full_path)
    #     #     scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #     #     scenario.draw(rnd)
    #     #     planning_problem_set.draw(rnd)
    #     #     rnd.render()
    #     #     rnd.clear()
    #     #
    #     # for scenario in os.listdir(ngsim_lankershim_2018b):
    #     #     full_path = ngsim_lankershim_2018b + "/" + scenario
    #     #     print(full_path)
    #     #     scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #     #     scenario.draw(rnd)
    #     #     planning_problem_set.draw(rnd)
    #     #     rnd.render()
    #     #     rnd.clear()
    #     #
    #     # for scenario in os.listdir(ngsim_us101_2018b):
    #     #     full_path = ngsim_us101_2018b + "/" + scenario
    #     #     print(full_path)
    #     #     scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #     #     scenario.draw(rnd)
    #     #     planning_problem_set.draw(rnd)
    #     #     rnd.render()
    #     #     rnd.clear()
    #     #
    #     # for scenario in os.listdir(ngsim_peachtree_2018b):
    #     #     full_path = ngsim_peachtree_2018b + "/" + scenario
    #     #     print(full_path)
    #     #     scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #     #     scenario.draw(rnd)
    #     #     planning_problem_set.draw(rnd)
    #     #     rnd.render()
    #     #     rnd.clear()
    #     #
    #     # for scenario in os.listdir(cooperative_2018b):
    #     #     full_path = cooperative_2018b + "/" + scenario
    #     #     print(full_path)
    #     #     scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #     #     scenario.draw(rnd)
    #     #     planning_problem_set.draw(rnd)
    #     #     rnd.render()
    #     #     rnd.clear()
    #     #
    #     # for scenario in os.listdir(bicycle_2018b):
    #     #     full_path = bicycle_2018b + "/" + scenario
    #     #     print(full_path)
    #     #     scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #     #     scenario.draw(rnd)
    #     #     planning_problem_set.draw(rnd)
    #     #     rnd.render()
    #     #     rnd.clear()
    #     #
    #     # for scenario in os.listdir(bicycle_2018b):
    #     #     full_path = bicycle_2018b + "/" + scenario
    #     #     print(full_path)
    #     #     scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #     #     scenario.draw(rnd)
    #     #     planning_problem_set.draw(rnd)
    #     #     rnd.render()
    #     #     rnd.clear()
    #     #
    #     # for scenario in os.listdir(bicycle_2018b):
    #     #     full_path = bicycle_2018b + "/" + scenario
    #     #     print(full_path)
    #     #     scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #     #     scenario.draw(rnd)
    #     #     planning_problem_set.draw(rnd)
    #     #     rnd.render()
    #     #     rnd.clear()
    #     #
    #     # for scenario in os.listdir(sumo_2018b):
    #     #     full_path = sumo_2018b + "/" + scenario
    #     #     print(full_path)
    #     #     scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #     #     scenario.draw(rnd)
    #     #     planning_problem_set.draw(rnd)
    #     #     rnd.render()
    #     #     rnd.clear()
    #
    #     for scenario in os.listdir(factory_2020a):
    #         full_path = factory_2020a + "/" + scenario
    #         print(full_path)
    #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #         scenario.draw(rnd)
    #         planning_problem_set.draw(rnd)
    #         rnd.render()
    #         rnd.clear()
    #     for scenario in os.listdir(hand_crafted_2020a):
    #         full_path = hand_crafted_2020a + "/" + scenario
    #         print(full_path)
    #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #         scenario.draw(rnd)
    #         planning_problem_set.draw(rnd)
    #         rnd.render()
    #         rnd.clear()
    #
    #     for scenario in os.listdir(ngsim_lankershim_2020a):
    #         full_path = ngsim_lankershim_2020a + "/" + scenario
    #         print(full_path)
    #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #         scenario.draw(rnd)
    #         planning_problem_set.draw(rnd)
    #         rnd.render()
    #         rnd.clear()
    #
    #     for scenario in os.listdir(ngsim_us101_2020a):
    #         full_path = ngsim_us101_2020a + "/" + scenario
    #         print(full_path)
    #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #         scenario.draw(rnd)
    #         planning_problem_set.draw(rnd)
    #         rnd.render()
    #         rnd.clear()
    #
    #     for scenario in os.listdir(ngsim_peachtree_2020a):
    #         full_path = ngsim_peachtree_2020a + "/" + scenario
    #         print(full_path)
    #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #         scenario.draw(rnd)
    #         planning_problem_set.draw(rnd)
    #         rnd.render()
    #         rnd.clear()
    #
    #     for scenario in os.listdir(bicycle_2020a):
    #         full_path = bicycle_2020a + "/" + scenario
    #         print(full_path)
    #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #         scenario.draw(rnd)
    #         planning_problem_set.draw(rnd)
    #         rnd.render()
    #         rnd.clear()


if __name__ == '__main__':
    unittest.main()
