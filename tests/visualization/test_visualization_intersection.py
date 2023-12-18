import os
import unittest

import matplotlib as mpl

# mpl.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import pytest

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.traffic_sign import (
    TrafficSign,
    TrafficSignElement,
    TrafficSignIDChina,
    TrafficSignIDGermany,
    TrafficSignIDRussia,
    TrafficSignIDSpain,
    TrafficSignIDUsa,
    TrafficSignIDZamunda,
)
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer


class TestVisualizationV2(unittest.TestCase):
    def setUp(self):
        self.full_path = os.path.dirname(os.path.abspath(__file__))
        self.filename_urban = os.path.join(
            self.full_path, "../test_scenarios/xml/2020a/ZAM_TestReadingIntersectionTrafficSign-1_1_T-1.xml"
        )
        self.filename_complex_tl = os.path.join(
            self.full_path, "../test_scenarios/xml/2020a/ZAM_TestReadingComplexTl-1_1_T-1.xml"
        )
        self.filename_lanelet = os.path.join(
            self.full_path, "../test_scenarios/xml/2020a/ZAM_TestReadingLanelets-1_1_T-1.xml"
        )
        self.filename_test_all = os.path.join(
            self.full_path, "../test_scenarios/xml/2020a/ZAM_TestReadingAll-1_1_T-1.xml"
        )
        self.rnd = MPRenderer()

    def test_intersection_plot(self):
        """Uses all options for plotting objects related to intersections or
        traffic sign/lights."""
        scenario, pp = CommonRoadFileReader(self.filename_urban).open()
        plt.close("all")
        mpl.rcParams["lines.scale_dashes"] = False
        draw_params = MPDrawParams()
        draw_params.time_begin = 20
        draw_params.lanelet_network.intersection.draw_intersections = True
        draw_params.lanelet_network.traffic_sign.draw_traffic_signs = True
        draw_params.lanelet_network.intersection.show_label = True
        draw_params.lanelet_network.lanelet.show_label = True
        draw_params.lanelet_network.lanelet.draw_line_markings = True
        scenario.lanelet_network.draw(self.rnd, draw_params)
        self.rnd.render(show=True)

    def test_traffic_signs(self):
        """Uses all options for plotting objects related to intersections or
        traffic sign/lights."""
        scenario, pp = CommonRoadFileReader(self.filename_urban).open()
        plt.close("all")
        draw_params = MPDrawParams()
        draw_params.time_begin = 20
        draw_params.lanelet_network.intersection.draw_intersections = True
        draw_params.traffic_sign.draw_traffic_signs = True
        draw_params.traffic_sign.show_label = False
        draw_params.traffic_sign.show_traffic_signs = "all"
        draw_params.traffic_sign.scale_factor = 1.0
        draw_params.lanelet_network.lanelet.draw_line_markings = False
        draw_params.lanelet_network.lanelet.show_label = False

        scenario.lanelet_network.draw(self.rnd, draw_params=draw_params)
        ts = TrafficSign(
            traffic_sign_id=100000,
            traffic_sign_elements=[
                TrafficSignElement(TrafficSignIDGermany.MAX_WIDTH, additional_values=[str(3)]),
                TrafficSignElement(TrafficSignIDGermany.MAX_SPEED_ZONE_START, additional_values=[str(30 / 3.6)]),
                TrafficSignElement(TrafficSignIDGermany.ADDITION_VALID_IN_X_KILOMETERS, additional_values=[str(3)]),
                TrafficSignElement(TrafficSignIDGermany.ADDITION_VALID_FOR_X_METERS, additional_values=[str(3)]),
                TrafficSignElement(TrafficSignIDGermany.ADDITION_TIME_PERIOD_PERMITTED, additional_values=[str(3)]),
                # TrafficSignElement(TrafficSignIDGermany.MAX_LENGTH,
                #                    additional_values=[str(3)]),
                #     TrafficSignElement(TrafficSignIDUsa.MAX_SPEED,
                #                        additional_values=[str(50 / 2.23694)]),
                #     TrafficSignElement(TrafficSignIDGermany.MIN_SPEED,
                #                        additional_values=[str(30 / 3.6)]),
                #     TrafficSignElement(TrafficSignIDGermany.MAX_SPEED,
                #                        additional_values=[str(80 / 3.6)]),
                #     TrafficSignElement(TrafficSignIDGermany.MAX_SPEED,
                #                        additional_values=[str(130 / 3.6)]),
                #     TrafficSignElement(TrafficSignIDGermany.NO_OVERTAKING_START,
                #                        additional_values=['test']),
                #     TrafficSignElement(TrafficSignIDGermany.MAX_HEIGHT,
                #                        additional_values=['80']),
                TrafficSignElement(TrafficSignIDGermany.MAX_WEIGHT, additional_values=["80"]),
            ],
            position=np.array([159.0, -88.0]),
            virtual=False,
            first_occurrence=set(),
        )
        draw_params.traffic_sign.speed_limit_unit = "auto"
        draw_params.traffic_sign.scale_factor = 1.0
        ts.draw(self.rnd, draw_params=draw_params)

        self.rnd.render(show=True)

    def test_all_signs(self):
        """Check if all traffic signs can be plotted."""
        kwargs = {"traffic_sign_id": 1, "first_occurrence": 1, "position": np.array([0.0, 0.0])}
        plt.close("all")
        plt.figure(figsize=[10, 10])
        self.rnd = MPRenderer(ax=plt.gca())
        with pytest.warns(None) as record:
            for value in TrafficSignIDGermany:
                kwargs["position"] = kwargs["position"] + np.array([0.0, 5.0])
                TrafficSign(traffic_sign_elements=[TrafficSignElement(value, ["foo"])], **kwargs).draw(self.rnd)

            for value in TrafficSignIDUsa:
                kwargs["position"] = kwargs["position"] + np.array([0.0, 5.0])
                TrafficSign(traffic_sign_elements=[TrafficSignElement(value, ["foo"])], **kwargs).draw(self.rnd)

            for value in TrafficSignIDRussia:
                kwargs["position"] = kwargs["position"] + np.array([0.0, 5.0])
                TrafficSign(traffic_sign_elements=[TrafficSignElement(value, ["foo"])], **kwargs).draw(self.rnd)

            for value in TrafficSignIDSpain:
                kwargs["position"] = kwargs["position"] + np.array([0.0, 5.0])
                TrafficSign(traffic_sign_elements=[TrafficSignElement(value, ["foo"])], **kwargs).draw(self.rnd)

            for value in TrafficSignIDChina:
                kwargs["position"] = kwargs["position"] + np.array([0.0, 5.0])
                TrafficSign(traffic_sign_elements=[TrafficSignElement(value, ["foo"])], **kwargs).draw(self.rnd)

            for value in TrafficSignIDZamunda:
                kwargs["position"] = kwargs["position"] + np.array([0.0, 5.0])
                TrafficSign(traffic_sign_elements=[TrafficSignElement(value, ["foo"])], **kwargs).draw(self.rnd)

        # uncomment to check plots
        self.rnd.render(show=True)

        # check for warnings
        for r in record:
            print("Found warning:", r.message)
        assert len(record) == 0, record

    def test_signal_states(self):
        """Uses all options for plotting objects related to intersections or
        traffic sign/lights."""
        scenario, pp = CommonRoadFileReader(self.filename_test_all).open()
        plt.close("all")
        draw_params = MPDrawParams()
        for t in range(2):
            self.rnd.clear()
            draw_params["time_begin"] = t
            self.rnd.draw_list(scenario.obstacles, draw_params=draw_params)
            self.rnd.render(show=True)

    def test_complex_intersection_tl(self):
        scenario, pp = CommonRoadFileReader(self.filename_complex_tl).open()
        plt.close("all")
        mpl.rcParams["lines.scale_dashes"] = False
        draw_params = MPDrawParams()
        draw_params.time_begin = 30
        draw_params.lanelet_network.intersection.draw_intersections = True
        draw_params.traffic_sign.draw_traffic_signs = True
        draw_params.lanelet_network.intersection.show_label = True
        draw_params.lanelet_network.lanelet.draw_line_markings = True
        draw_params.lanelet_network.lanelet.show_label = True
        scenario.lanelet_network.draw(self.rnd, draw_params=draw_params)
        self.rnd.render(show=True)


if __name__ == "__main__":
    unittest.main()
