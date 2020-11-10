# matplotlib.use('Qt5Agg')
import matplotlib
# matplotlib.use('TkAgg')
import os
import time
import unittest
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pytest
import math
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Rectangle, Circle, Polygon
from commonroad.geometry.shape import ShapeGroup
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import Occupancy
from commonroad.scenario.trajectory import State
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
# from commonroad.visualization.draw_dispatch_cr import draw_object,
# _retrieve_value

from commonroad.visualization.renderer import MPRenderer
from commonroad.visualization.param_server import ParamServer, \
    write_default_params


class TestVisualization(unittest.TestCase):

    def setUp(self) -> None:
        self.rnd = MPRenderer()

    def test_primitive(self):
        rnd = MPRenderer(None, None)
        params = ParamServer()

        rect = Rectangle(1, 2, np.array([1.5, 2.0]), math.pi * 0.25)
        poly = Polygon(np.array(
                [[.2, .2], [.2, .4], [.3, .6], [.4, .4], [.4, .2], [.2, .2]]))
        circ = Circle(2, np.array([3, 3]))

        rect.draw(rnd, params, tuple())
        poly.draw(rnd, params, tuple())
        circ.draw(rnd, params, tuple())
        rnd.render()

    def test_scenario(self):

        # test draw_object for all possible object types

        full_path = os.path.dirname(os.path.abspath(__file__))
        filename = full_path + '/../common/USA_Lanker-1_1_T-1.xml'
        # filename = full_path + '/../common/test_reading_all.xml'
        scenario, planning_problem_set = CommonRoadFileReader(filename).open()
        scenario: Scenario = scenario

        rnd = MPRenderer()
        with pytest.warns(None) as record_warnings:
            scenario.lanelet_network.draw(rnd)
            rnd.render()
            # visualization
            circ = Circle(2.0, np.array([10.0, 0.0]))
            obs = StaticObstacle(1000, ObstacleType.CAR, circ,
                                 initial_state=State(position=np.array([0, 0]),
                                                     orientation=0.4))
            scenario.add_objects(obs)
            for obs in scenario.static_obstacles:
                obs.draw(rnd)
            rnd.render()

            draw_params = ParamServer({
                    'scenario': {
                            'dynamic_obstacle': {
                                    'occupancy': {'draw_occupancy': True}}}})
            rnd.draw_list(scenario.dynamic_obstacles, draw_params=draw_params)
            rnd.render()

            rnd.draw_list(
                    scenario.dynamic_obstacles[0].prediction.occupancy_set)
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
            poly = Polygon(
                    np.array([[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]]))
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

    def test_planning(self):
        # test draw_object for all possible object types
        full_path = os.path.dirname(os.path.abspath(__file__))
        # print(full_path)
        # filename = full_path +
        # '/../../../../../scenarios/cooperative/C-USA_Lanker-2_4_T-1.xml'
        filename = full_path + '/../common/test_reading_all.xml'
        scenario, planning_problem_set = CommonRoadFileReader(filename).open()
        planning_problem_set: PlanningProblemSet = planning_problem_set

        with pytest.warns(None) as record_warnings:
            planning_problem_set.draw(self.rnd)
            self.rnd.render()
            self.rnd.clear()
            problem = list(planning_problem_set.planning_problem_dict.values())[
                0]
            problem.draw(self.rnd)
            problem.goal.draw(self.rnd)
            problem.initial_state.draw(self.rnd)
            self.rnd.render()

        assert (len(record_warnings) == 0)
        plt.close('all')

    def test_trajectory_unique_colors(self):
        full_path = os.path.dirname(os.path.abspath(__file__))
        filename = full_path + '/../common/USA_US101-3_3_T-1.xml'
        scenario, planning_problem_set = CommonRoadFileReader(filename).open()
        traj = list(
            map(lambda x: x.prediction.trajectory, scenario.dynamic_obstacles))
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
        full_path = os.path.dirname(os.path.abspath(__file__))
        filename = full_path + '/../common/USA_US101-3_3_T-1.xml'
        scenario, planning_problem_set = CommonRoadFileReader(filename).open()
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
                    'planning_problem_set': {
                            'draw_ids': [list(
                                    planning_problem_set.planning_problem_dict.keys())[
                                             0]]
                    },
                    'time_begin':           15,
                    'time_end':             25,
                    'dynamic_obstacle':     {
                            'occupancy': {
                                    'draw_occupancies': 0,
                                    'shape':            {
                                            'rectangle': {'facecolor': 'g'}
                                    }
                            }
                    }
            }
            scenario.draw(self.rnd, draw_params=draw_params)
            # plt.tight_layout()
            planning_problem_set.draw(self.rnd, draw_params=draw_params)
            self.rnd.render(show=False, filename='/tmp/{}.png'.format(i))
            self.rnd.clear()
            tt += time.time() - t1  # plt.close()

        print('time: {}'.format(tt / nrun))

    def test_stylesheet(self):
        # Write default params to file
        json_filename = 'test_params.json'
        write_default_params(json_filename)
        # Now modify stylesheet file and read it
        params = ParamServer.from_json(json_filename)
        # Use for drawing
        full_path = os.path.dirname(os.path.abspath(__file__))
        filename = full_path + '/../common/USA_US101-3_3_T-1.xml'
        scenario, planning_problem_set = CommonRoadFileReader(filename).open()
        scenario.draw(self.rnd, draw_params=params)


# #
#     def plot_limits(self, lims):
#         plt.clf()
#         full_path = os.path.dirname(os.path.abspath(__file__))
#         filename = full_path +
#         '/../../../../scenarios/cooperative/C-USA_Lanker-2_4_T-1.xml'
#         scenario, planning_problem_set = CommonRoadFileReader(filename).open()
#         scenario: Scenario = scenario
#         set_non_blocking()
#         plt.style.use('classic')
#         inch_in_cm = 2.54
#         figsize = [30, 8]
#         fig = plt.figure(figsize=(figsize[0] / inch_in_cm, figsize[1] / inch_in_cm))
#         # fig, axx= plt.subplots()
#         plt.gca().set(title='one vehicle should be green and its occupancy
#         be plotted with opacity')
#         plt.gca().axis('equal')
#         # plt.gca().autoscale()
#         plt.tight_layout()
#         handles = {}
#         draw_object(scenario, plot_limits=lims, handles=handles)
#         fig.canvas.draw()
#         import time
#         t2 = time.time()
#
#         for ii in range(0,10):
#
#             # draw_object(scenario, lims,handles=handles)
#             # plt.draw()
#             # plt.pause(0.001)
#             lims[0] = lims[0] + 5
#             lims[1] = lims[1] + 5
#             redraw_obstacles(scenario,handles,plot_limits=lims, figure_handle=fig)
#             # fig.canvas.draw()
#             # fig.canvas.flush_events()
#
#         print(time.time() - t2)
#         iii=1
#         # plt.close()
#
#     def test_plot_limits(self):
#
#         self.plot_limits([-50, 40, -20, 50])
#
#         # t1=time.time()
#         # for i in range(0,1):
#         #     self.plot_limits(None)
#         # print(time.time()-t1)

    # def test_visualize_all_scenarios(self):
    #     scenarios = "update"
    #     factory = scenarios + "/scenario-factory"
    #     #cooperative = scenarios + "/cooperative"
    #     hand_crafted = scenarios + "/hand-crafted"
    #     ngsim_lankershim = scenarios + "/NGSIM/Lankershim"
    #     ngsim_us101 = scenarios + "/NGSIM/US101"
    #     ngsim_peachtree = scenarios + "/NGSIM/Peachtree"
    #     #sumo = scenarios + "/SUMO"
    #     bicycle = scenarios + "/THI-Bicycle"
    #
    #     for scenario in os.listdir(hand_crafted):
    #         full_path = hand_crafted + "/" + scenario
    #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #         draw_object(scenario)
    #         draw_object(planning_problem_set)
    #         plt.pause(0.0001)
    #         plt.clf()
    #
    #     for scenario in os.listdir(ngsim_lankershim):
    #         full_path = ngsim_lankershim + "/" + scenario
    #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #         draw_object(scenario)
    #         draw_object(planning_problem_set)
    #         plt.pause(0.0001)
    #         plt.clf()
    #
    #     for scenario in os.listdir(ngsim_us101):
    #         full_path = ngsim_us101 + "/" + scenario
    #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #         draw_object(scenario)
    #         draw_object(planning_problem_set)
    #         plt.pause(0.0001)
    #         plt.clf()
    #
    #     # for scenario in os.listdir(cooperative):
    #     #     full_path = cooperative + "/" + scenario
    #     #     CommonRoadFileReader(full_path).open()
    #     #
    #     # for scenario in os.listdir(sumo):
    #     #     full_path = sumo + "/" + scenario
    #     #     CommonRoadFileReader(full_path).open()
    #
    #     for scenario in os.listdir(bicycle):
    #         full_path = bicycle + "/" + scenario
    #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #         draw_object(scenario)
    #         draw_object(planning_problem_set)
    #         plt.pause(0.0001)
    #         plt.clf()
    #
    #     for scenario in os.listdir(factory):
    #         full_path = factory + "/" + scenario
    #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()
    #         draw_object(scenario)
    #         draw_object(planning_problem_set)
    #         plt.pause(0.0001)
    #         plt.clf()


if __name__ == '__main__':
    unittest.main()
