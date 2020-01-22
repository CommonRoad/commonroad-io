import os
import time
import unittest
import matplotlib as mpl
mpl.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import pylustrator
import pytest
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Rectangle, Circle, Polygon
from commonroad.geometry.shape import ShapeGroup
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import Occupancy
from commonroad.scenario.trajectory import State
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.visualization.plot_helper import *
from commonroad.visualization.draw_dispatch_cr import draw_object, _retrieve_value


class TestVisualization(unittest.TestCase):
    def setUp(self):
        self.full_path = os.path.dirname(os.path.abspath(__file__))
        self.filename_urban = os.path.join(self.full_path,  '../common/test_reading_intersection_traffic_sign.xml')

    def test_intersection_plot(self):
        "Test plotting an intersection."
        scenario, pp = CommonRoadFileReader(self.filename_urban).open()
        plt.close('all')
        plt.figure()
        draw_object(scenario.lanelet_network,
                    draw_params={'lanelet_network':{'draw_intersection':True}},
                    legend={('lanelet_network','intersection','incoming_lanelets_color'):'Incoming lanelets',
                            ('lanelet_network','intersection','successors_left_color'):'Successors left',
                            ('lanelet_network','intersection','successors_straight_color'):'Successors straight',
                            ('lanelet_network','intersection','successors_right_color'):'Successors right'})

        plt.autoscale()
        plt.axis('equal')
        # plt.pause(50)
        plt.show()
        print('done')

# def test_read_svg():
#     path = '/home/klischat/GIT_REPOS/commonroad-io/commonroad/visualization/traffic_signs/306.svg'
#     path2 = '/home/klischat/GIT_REPOS/commonroad-io/commonroad/visualization/traffic_signs/310.svg'
#     plt.figure()
#     pylustrator.load(path, offset=[5, 0.5])
#     pylustrator.load(path2, offset=[10, 0.5])
#
#     plt.show()
