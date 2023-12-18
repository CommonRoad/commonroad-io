# import matplotlib
# matplotlib.use('Qt5Agg')
# matplotlib.use('TkAgg')
import math
import os
import tempfile
import time
import unittest
import warnings

import matplotlib.pyplot as plt
import numpy as np
import pytest

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Circle, Polygon, Rectangle, ShapeGroup
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import Occupancy, SetBasedPrediction
from commonroad.scenario.obstacle import (
    EnvironmentObstacle,
    ObstacleType,
    PhantomObstacle,
    StaticObstacle,
)
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState
from commonroad.scenario.traffic_sign import (
    TRAFFIC_SIGN_WITH_ADDITIONAL_VALUE,
    TrafficSign,
    TrafficSignElement,
    TrafficSignIDGermany,
    TrafficSignIDSpain,
    TrafficSignIDUsa,
    TrafficSignIDZamunda,
)
from commonroad.visualization.draw_params import (
    DynamicObstacleParams,
    MPDrawParams,
    VehicleShapeParams,
)
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.traffic_sign import (
    draw_traffic_light_signs,
    text_prop_dict,
)


class TestVisualizationV2(unittest.TestCase):
    def setUp(self) -> None:
        self.rnd = MPRenderer()
        self.cwd_path = os.path.dirname(os.path.abspath(__file__))
        full_path = os.path.dirname(os.path.abspath(__file__))
        self.out_path = self.cwd_path + "/../.pytest_cache"
        self.ngsim_scen_1 = full_path + "/../test_scenarios/USA_Peach-4_8_T-1.xml"
        self.ngsim_scen_2 = full_path + "/../test_scenarios/USA_US101-4_1_T-1.xml"
        self.scenario_3d_points = full_path + "/../test_scenarios/test_3d_points.xml"
        if not os.path.isdir(self.out_path):
            os.makedirs(self.out_path)
        else:
            for dirpath, dirnames, filenames in os.walk(self.out_path):
                for file in filenames:
                    if file.endswith(".mp4") or file.endswith(".gif") or file.endswith(".png"):
                        os.remove(os.path.join(dirpath, file))

    def test_primitive(self):
        params = MPDrawParams()

        rect = Rectangle(1, 2, np.array([1.5, 2.0]), math.pi * 0.25)
        poly = Polygon(np.array([[0.2, 0.2], [0.2, 0.4], [0.3, 0.6], [0.4, 0.4], [0.4, 0.2], [0.2, 0.2]]))
        circ = Circle(2, np.array([3, 3]))

        rect.draw(self.rnd, params)
        poly.draw(self.rnd, params)
        circ.draw(self.rnd, params)
        self.rnd.render()

    def test_scenario(self):
        # test draw_object for all possible object types
        scenario, planning_problem_set = CommonRoadFileReader(self.ngsim_scen_1).open()
        scenario: Scenario = scenario

        rnd = MPRenderer()
        with pytest.warns(None):
            scenario.lanelet_network.draw(
                rnd,
            )
            rnd.render()
            # visualization
            circ = Circle(2.0, np.array([10.0, 0.0]))
            obs = StaticObstacle(
                1000, ObstacleType.CAR, circ, initial_state=InitialState(position=np.array([0, 0]), orientation=0.4)
            )
            scenario.add_objects(obs)
            for obs in scenario.static_obstacles:
                obs.draw(
                    rnd,
                )
            rnd.render()

            draw_params = MPDrawParams()
            draw_params.dynamic_obstacle.occupancy.draw_occupancies = True

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

        # plt.close('all')

    def test_scenario_3d(self):
        scenario, _ = CommonRoadFileReader(self.scenario_3d_points).open()
        rnd = MPRenderer()
        draw_params = MPDrawParams()
        draw_params.lanelet_network.traffic_sign.draw_traffic_signs = True
        draw_params.lanelet_network.traffic_light.draw_traffic_lights = True
        draw_params.lanelet_network.lanelet.show_label = True
        scenario.lanelet_network.draw(rnd, draw_params)
        rnd.render()

    def test_focus_obstacle(self):
        # test draw_object for all possible object types
        scenario, planning_problem_set = CommonRoadFileReader(self.ngsim_scen_1).open()
        scenario: Scenario = scenario
        x0 = -40
        rnd = MPRenderer(plot_limits=[x0, 40, -40, 40], focus_obstacle=scenario.obstacle_by_id(1239))
        rnd.draw_params.dynamic_obstacle.occupancy.draw_occupancies = True
        with pytest.warns(None):
            scenario.lanelet_network.draw(
                rnd,
            )

            rnd.draw_list(scenario.dynamic_obstacles)
            assert rnd.plot_limits_focused[0] != 40

            rnd.focus_obstacle_id = None
            rnd.clear(keep_static_artists=True)
            rnd.draw_list(scenario.dynamic_obstacles)
            assert rnd.plot_limits_focused[0] == x0

        plt.close("all")

    def test_plotting_all_traffic_signs(self):
        traffic_sign_types = [TrafficSignIDZamunda, TrafficSignIDGermany, TrafficSignIDUsa, TrafficSignIDSpain]
        rnd = MPRenderer()
        with warnings.catch_warnings(record=True) as w:
            for ts_type in traffic_sign_types:
                for v in ts_type:
                    # print(v)
                    ts = TrafficSign(
                        100,
                        [TrafficSignElement(v, ["test1", "test2"])],
                        first_occurrence={0},
                        position=np.array([0.0, 0.0]),
                        virtual=False,
                    )
                    draw_traffic_light_signs(
                        ts,
                        traffic_sign_params=rnd.draw_params.traffic_sign,
                        traffic_light_params=rnd.draw_params.traffic_light,
                        rnd=rnd,
                    )

            self.assertEqual(
                len(w), 0, msg="The following warnings were raised:\n" + "\n".join(str(w_tmp.message) for w_tmp in w)
            )

    def test_planning(self):
        # test draw_object for all possible object types
        full_path = os.path.dirname(os.path.abspath(__file__))
        # print(full_path)
        # filename = full_path +
        # '/../../../../../scenarios/cooperative/C-USA_Lanker-2_4_T-1.xml'
        filename = full_path + "/../test_scenarios/test_reading_all.xml"
        scenario, planning_problem_set = CommonRoadFileReader(filename).open()
        planning_problem_set: PlanningProblemSet = planning_problem_set

        with pytest.warns(None) as record_warnings:
            planning_problem_set.draw(
                self.rnd,
            )
            self.rnd.render()
            self.rnd.clear()
            problem = list(planning_problem_set.planning_problem_dict.values())[0]
            problem.draw(
                self.rnd,
            )
            problem.goal.draw(self.rnd)
            problem.initial_state.draw(self.rnd)
            self.rnd.render()

        self.assertEqual(0, len(record_warnings))
        plt.close("all")

    def test_trajectory_unique_colors(self):
        scenario, planning_problem_set = CommonRoadFileReader(self.ngsim_scen_2).open()
        traj = list(map(lambda x: x.prediction.trajectory, scenario.dynamic_obstacles))
        params = MPDrawParams()
        params.trajectory.unique_colors = True
        scenario.lanelet_network.draw(self.rnd)
        self.rnd.draw_list(scenario.dynamic_obstacles)
        self.rnd.draw_trajectories(traj, params)
        self.rnd.render(show=True)

    def test_visual_appearance(self):
        tt = 0
        nrun = 20
        plt.close("all")
        # plt.ioff()
        # set_non_blocking()
        scenario, planning_problem_set = CommonRoadFileReader(self.ngsim_scen_2).open()
        scenario: Scenario = scenario
        # plt.autoscale(False)
        plt.style.use("classic")
        inch_in_cm = 2.54
        figsize = [20, 15]
        plt.figure(figsize=(figsize[0] / inch_in_cm, figsize[1] / inch_in_cm))
        plt.gca().set(title="occupancies should be be plotted with opacity, " "plot limits: [-50,60,-50,50]")
        plt.gca().autoscale_view(False, False, False)
        self.rnd = MPRenderer(plot_limits=[-50, 60, -50, 50])
        for i in range(0, nrun):
            t1 = time.time()
            draw_params = MPDrawParams()
            draw_params.time_begin = 15
            draw_params.time_end = 25
            draw_params.dynamic_obstacle.occupancy.draw_occupancies = True
            draw_params.dynamic_obstacle.occupancy.shape.facecolor = "g"
            draw_params.planning_problem_set.draw_ids = [list(planning_problem_set.planning_problem_dict.keys())[0]]
            scenario.draw(self.rnd, draw_params=draw_params)
            # plt.tight_layout()
            planning_problem_set.draw(self.rnd, draw_params=draw_params)
            self.rnd.render(show=False, filename="/tmp/{}.png".format(i))
            self.rnd.clear()
            tt += time.time() - t1  # plt.close()

        print("time: {}".format(tt / nrun))

    def test_traffic_sign_plotting_properties(self):
        vis_dict = text_prop_dict()
        for t_id in TRAFFIC_SIGN_WITH_ADDITIONAL_VALUE:
            self.assertTrue(
                TrafficSignIDGermany[t_id].value in vis_dict,
                f"Create plot settings for traffic sign {t_id} with ID {TrafficSignIDGermany[t_id].value}",
            )

    def test_stylesheet(self):
        # Write default params to file
        params = MPDrawParams()
        params.time_begin = 1
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as fp:
            params.save(fp.name)
            params_read = MPDrawParams.load(fp.name)

        self.assertEqual(params_read, params)
        # Use for drawing
        scenario, planning_problem_set = CommonRoadFileReader(self.ngsim_scen_2).open()
        scenario.draw(self.rnd, draw_params=params)

    def test_setting_params_child_attribute(self):
        """Check if setting a child attribute of a parent attribute works."""
        params = MPDrawParams()
        params.time_begin = 1
        params.time_end = 2
        # Note, faceolor is not a valid attribute of the parent attribute
        params.facecolor = "red"
        self.assertEqual(params.dynamic_obstacle.time_begin, 1)
        self.assertEqual(params.dynamic_obstacle.time_end, 2)
        self.assertEqual(params.dynamic_obstacle.occupancy.shape.facecolor, "red")

    def test_setting_params_child_ctor(self):
        """Check if setting a child attribute of a parent ctor works."""
        params = MPDrawParams(time_begin=1, time_end=2)
        self.assertEqual(params.time_begin, 1)
        self.assertEqual(params.dynamic_obstacle.time_begin, 1)
        self.assertEqual(params.dynamic_obstacle.time_end, 2)
        self.assertEqual(params.time_end, 2)

    def test_video(self):
        scenario, _ = CommonRoadFileReader(self.ngsim_scen_1).open()  #
        t0 = time.time()
        self.rnd.focus_obstacle_id = 520
        self.rnd.plot_limits = [-40, 40, -40, 40]
        for _ in range(1):
            draw_params = MPDrawParams()
            draw_params.time_end = 10
            draw_params.dynamic_obstacle.show_label = False
            draw_params.dynamic_obstacle.draw_icon = True
            draw_params.dynamic_obstacle.draw_shape = True
            self.rnd.create_video(
                [scenario], os.path.join(self.out_path, str(scenario.scenario_id) + ".mp4"), draw_params=draw_params
            )

        print(time.time() - t0)

    def test_phantom_obstacle(self):
        occupancy = [Occupancy(0, Rectangle(10, 10)), Occupancy(1, Rectangle(10, 10, np.array([10.0, 10.0])))]
        pred = SetBasedPrediction(0, occupancy)
        phantom_obs = PhantomObstacle(0, pred)
        phantom_obs.draw(
            self.rnd,
        )
        self.rnd.render(show=True)

        self.rnd.clear()

        scn = Scenario(0.1)
        scn.add_objects(phantom_obs)
        scn.draw(
            self.rnd,
        )
        self.rnd.render(show=True)

    def test_environment_obstacle(self):
        shape = Rectangle(20, 10)
        env_obs = EnvironmentObstacle(0, ObstacleType.BUILDING, shape)
        env_obs.draw(self.rnd)
        self.rnd.render(show=True)

        self.rnd.clear()

        scn = Scenario(0.1)
        scn.add_objects(env_obs)
        scn.draw(
            self.rnd,
        )
        self.rnd.render(show=True)

    def test_label_clearing(self):
        scenario, _ = CommonRoadFileReader(self.ngsim_scen_1).open()
        render = MPRenderer()
        for i in range(10):
            draw_params = MPDrawParams()
            draw_params.time_begin = i
            draw_params.dynamic_obstacle.show_label = True
            scenario.draw(render, draw_params)
            render.render(show=True)

    def test_params(self):
        veh = VehicleShapeParams()
        assert veh.occupancy.shape.opacity == 1.0
        dyn = DynamicObstacleParams()
        assert dyn.vehicle_shape.occupancy.shape.opacity == 1.0

    # def test_visualize_all_scenarios(self):  #     scenarios_2020a = "TODO"  #     scenarios_2018b = "TODO"  #  #
    #  factory_2020a = scenarios_2020a + "/scenario-factory"  #     hand_crafted_2020a = scenarios_2020a +  #  #
    #  "/hand-crafted"  #     ngsim_lankershim_2020a = scenarios_2020a + "/NGSIM/Lankershim"  #     ngsim_us101_2020a
    #  = scenarios_2020a + "/NGSIM/US101"  #     ngsim_peachtree_2020a = scenarios_2020a + "/NGSIM/Peachtree"  #  #
    #  bicycle_2020a = scenarios_2020a + "/THI-Bicycle"  #     cooperative_2018b = scenarios_2018b + "/cooperative"
    #     bicycle_2018b = scenarios_2018b + "/THI-Bicycle"  #     sumo_2018b = scenarios_2018b + "/SUMO"  #  #
    #     hand_crafted_2018b = scenarios_2018b + "/hand-crafted"  #     ngsim_lankershim_2018b = scenarios_2018b +  #
    #     "/NGSIM/Lankershim"  #     ngsim_us101_2018b = scenarios_2018b + "/NGSIM/US101"  #  #
    #     ngsim_peachtree_2018b = scenarios_2018b + "/NGSIM/Peachtree"  #  #     rnd = MPRenderer()  #     for  #
    #     scenario in os.listdir(hand_crafted_2018b):  #         full_path = hand_crafted_2018b + "/" + scenario  #
    #     print(full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()  #  #
    #     scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()
    #  #     for scenario in os.listdir(ngsim_lankershim_2018b):  #         full_path = ngsim_lankershim_2018b + "/"
    #  + scenario  #         print(full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(  #
    #  full_path).open()  #         scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #  #  rnd.render()
    #         rnd.clear()  #  #     for scenario in os.listdir(ngsim_us101_2018b):  #  #  full_path =
    #         ngsim_us101_2018b + "/" + scenario  #         print(full_path)  #         scenario,
    #  planning_problem_set = CommonRoadFileReader(full_path).open()  #         scenario.draw(rnd)  #  #
    #  planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()  #  #     for scenario in  #
    #  os.listdir(ngsim_peachtree_2018b):  #         full_path = ngsim_peachtree_2018b + "/" + scenario  #  #  print(
    #  full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()  #  #
    #  scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()  #
    #     for scenario in os.listdir(cooperative_2018b):  #         full_path = cooperative_2018b + "/" + scenario  #
    #     print(full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()  #  #
    #     scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()
    #  #     for scenario in os.listdir(bicycle_2018b):  #         full_path = bicycle_2018b + "/" + scenario  #  #
    #  print(full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()  #  #
    #  scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()  #
    #     for scenario in os.listdir(bicycle_2018b):  #         full_path = bicycle_2018b + "/" + scenario  #  #
    #     print(full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()  #  #
    #     scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()
    #  #     for scenario in os.listdir(bicycle_2018b):  #         full_path = bicycle_2018b + "/" + scenario  #  #
    #  print(full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()  #  #
    #  scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()  #
    #     for scenario in os.listdir(sumo_2018b):  #         full_path = sumo_2018b + "/" + scenario  #  #     print(
    #     full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()  #  #
    #     scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()
    #  #     for scenario in os.listdir(factory_2020a):  #         full_path = factory_2020a + "/" + scenario  #  #
    #  print(full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()  #  #
    #  scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()  #
    #  for scenario in os.listdir(hand_crafted_2020a):  #         full_path = hand_crafted_2020a + "/" + scenario  #
    #  print(full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()  #  #
    #  scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()  #
    #     for scenario in os.listdir(ngsim_lankershim_2020a):  #         full_path = ngsim_lankershim_2020a + "/" +
    #     scenario  #         print(full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(  #
    #     full_path).open()  #         scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #  #
    #     rnd.render()  #         rnd.clear()  #  #     for scenario in os.listdir(ngsim_us101_2020a):  #  #
    #     full_path = ngsim_us101_2020a + "/" + scenario  #         print(full_path)  #         scenario,
    #     planning_problem_set = CommonRoadFileReader(full_path).open()  #         scenario.draw(rnd)  #  #
    #     planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()  #  #     for scenario in  #
    #     os.listdir(ngsim_peachtree_2020a):  #         full_path = ngsim_peachtree_2020a + "/" + scenario  #  #
    #     print(full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()  #  #
    #     scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()
    #  #     for scenario in os.listdir(bicycle_2020a):  #         full_path = bicycle_2020a + "/" + scenario  #  #
    #  print(full_path)  #         scenario, planning_problem_set = CommonRoadFileReader(full_path).open()  #  #
    #  scenario.draw(rnd)  #         planning_problem_set.draw(rnd)  #         rnd.render()  #         rnd.clear()


if __name__ == "__main__":
    unittest.main()
