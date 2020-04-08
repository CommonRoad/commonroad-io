import os
import warnings

import matplotlib as mpl
import pytest
from commonroad.scenario.traffic_sign import TrafficSign, TrafficSignElement, TrafficSignIDGermany, TrafficSignIDUsa,\
TrafficSignIDZamunda, TrafficSignIDRussia, TrafficSignIDSpain, TrafficSignIDChina
import numpy as np

# mpl.use('TkAgg')
import matplotlib.pyplot as plt
import unittest
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object


class TestVisualization(unittest.TestCase):
    def setUp(self):
        self.full_path = os.path.dirname(os.path.abspath(__file__))
        self.filename_urban = os.path.join(self.full_path,  '../common/test_reading_intersection_traffic_sign.xml')
        self.filename_lanelet = os.path.join(self.full_path, '../common/test_reading_lanelets.xml')
        self.filename_test_all = os.path.join(self.full_path, '../common/test_reading_all.xml')

    def test_intersection_plot(self):
        "Uses all options for plotting objects related to intersections or traffic sign/lights."
        scenario, pp = CommonRoadFileReader(self.filename_urban).open()
        plt.close('all')
        plt.figure()
        mpl.rcParams['lines.scale_dashes'] = False
        draw_object(scenario.lanelet_network,
                    draw_params={'time_begin': 20, 'lanelet_network':{'draw_intersections':True, 'draw_traffic_signs':True,
                                                                      },
                                 'lanelet':{'draw_lane_marking':True,
                                            'show_label':True}},
                    legend={('lanelet_network','intersection','incoming_lanelets_color'):'Incoming lanelets',
                            ('lanelet_network','intersection','successors_left_color'):'Successors left',
                            ('lanelet_network','intersection','successors_straight_color'):'Successors straight',
                            ('lanelet_network','intersection','successors_right_color'):'Successors right',
                            ('lanelet_network','traffic_light','green_color'):'Traffic light green',
                            ('lanelet_network','traffic_light','yellow_color'):'Traffic light yellow',
                            ('lanelet_network','traffic_light','red_color'):'Traffic light red'})

        plt.autoscale()
        plt.axis('equal')
        plt.show()

    def test_traffic_signs(self):
        "Uses all options for plotting objects related to intersections or traffic sign/lights."
        scenario, pp = CommonRoadFileReader(self.filename_urban).open()
        plt.close('all')
        plt.figure()
        draw_params = {'time_begin': 20,
                                 'lanelet_network': {'draw_intersections': False, 'draw_traffic_signs': True,
                                                     'traffic_sign':{'show_label':False,'show_traffic_signs':'all',
                                                                     'scale_factor': 0.15}},
                                 'lanelet': {'draw_lane_marking': False,
                                             'show_label': False}}
        draw_object(scenario.lanelet_network,
                    draw_params=draw_params)
        ts = TrafficSign(traffic_sign_id=100000,traffic_sign_elements=
        [TrafficSignElement(TrafficSignIDUsa.MAX_SPEED,additional_values=['50']),
         TrafficSignElement(TrafficSignIDGermany.MIN_SPEED, additional_values=['30']),
         TrafficSignElement(TrafficSignIDGermany.MAX_SPEED,additional_values=['80']),
         TrafficSignElement(TrafficSignIDGermany.NO_OVERTAKING_START,additional_values=['80']),
         TrafficSignElement(TrafficSignIDGermany.STOP,additional_values=['80'])], position=np.array([159.,-88.]),
                         virtual=False, first_occurrence={})
        draw_object(ts,draw_params={'traffic_sign':{'scale_factor': 0.3, 'kwargs':{'arrowprops':{'arrowstyle':"simple"}}}})
        plt.autoscale()
        plt.axis('equal')
        plt.show()

    def test_all_signs(self):
        """Check if all traffic signs can be plotted."""
        kwargs = {'traffic_sign_id':1,'first_occurrence':1,'position': np.array([0.,0.])}
        plt.figure(figsize=[10,10])

        with pytest.warns(None) as record:
            for value in TrafficSignIDGermany:
                kwargs['position'] = kwargs['position'] + np.array([0.,5.])
                draw_object(TrafficSign(traffic_sign_elements=[TrafficSignElement(value, ['foo'])], **kwargs))

            for value in TrafficSignIDUsa:
                kwargs['position'] = kwargs['position'] + np.array([0.,5.])
                draw_object(TrafficSign(traffic_sign_elements=[TrafficSignElement(value, ['foo'])], **kwargs))

            for value in TrafficSignIDRussia:
                kwargs['position'] = kwargs['position'] + np.array([0.,5.])
                draw_object(TrafficSign(traffic_sign_elements=[TrafficSignElement(value, ['foo'])], **kwargs))

            for value in TrafficSignIDSpain:
                kwargs['position'] = kwargs['position'] + np.array([0.,5.])
                draw_object(TrafficSign(traffic_sign_elements=[TrafficSignElement(value, ['foo'])], **kwargs))

            for value in TrafficSignIDChina:
                kwargs['position'] = kwargs['position'] + np.array([0.,5.])
                draw_object(TrafficSign(traffic_sign_elements=[TrafficSignElement(value, ['foo'])], **kwargs))

            for value in TrafficSignIDZamunda:
                kwargs['position'] = kwargs['position'] + np.array([0.,5.])
                draw_object(TrafficSign(traffic_sign_elements=[TrafficSignElement(value, ['foo'])], **kwargs))


        # uncomment to check plots
        # plt.xlim([-1., 1.])
        # plt.draw()
        # plt.autoscale()
        # plt.ylim([-1., kwargs['position'][1] + 1])
        # plt.pause(100)

        # check for warnings
        for r in record:
            print('Found warning:', r.message)
        assert len(record) == 0, record



    def test_signal_states(self):
        "Uses all options for plotting objects related to intersections or traffic sign/lights."
        scenario, pp = CommonRoadFileReader(self.filename_test_all).open()
        plt.close('all')
        plt.figure()
        draw_params = {'time_begin': 1}
        draw_object(scenario.obstacle_by_id(2),
                    draw_params=draw_params)
        plt.autoscale()
        plt.axis('equal')
        plt.show()


if __name__ == '__main__':
    unittest.main()
