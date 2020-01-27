import os
# import  matplotlib as mpl
# mpl.use('TkAgg')
import matplotlib.pyplot as plt
import unittest
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object


class TestVisualization(unittest.TestCase):
    def setUp(self):
        self.full_path = os.path.dirname(os.path.abspath(__file__))
        self.filename_urban = os.path.join(self.full_path,  '../common/test_reading_intersection_traffic_sign.xml')

    def test_intersection_plot(self):
        "Uses all options for plotting objects related to intersections or trafic sign/lights."
        scenario, pp = CommonRoadFileReader(self.filename_urban).open()
        plt.close('all')
        plt.figure()
        draw_object(scenario.lanelet_network,
                    draw_params={'time_begin': 20, 'lanelet_network':{'draw_intersections':True, 'draw_traffic_signs':True}},
                    legend={('lanelet_network','intersection','incoming_lanelets_color'):'Incoming lanelets',
                            ('lanelet_network','intersection','successors_left_color'):'Successors left',
                            ('lanelet_network','intersection','successors_straight_color'):'Successors straight',
                            ('lanelet_network','intersection','successors_right_color'):'Successors right',
                            ('lanelet_network','traffic_light','green_color'):'Traffic light green',
                            ('lanelet_network','traffic_light','yellow_color'):'Traffic light yellow',
                            ('lanelet_network','traffic_light','red_color'):'Traffic light red'})

        plt.autoscale()
        plt.axis('equal')
        # plt.pause(50)
        # plt.show()

# def test_read_svg():
#     path = '/home/klischat/GIT_REPOS/commonroad-io/commonroad/visualization/traffic_signs/306.svg'
#     path2 = '/home/klischat/GIT_REPOS/commonroad-io/commonroad/visualization/traffic_signs/310.svg'
#     plt.figure()
#     pylustrator.load(path, offset=[5, 0.5])
#     pylustrator.load(path2, offset=[10, 0.5])
#
#     plt.show()
