import copy
import unittest
from typing import List

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches

from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario, ScenarioID
from commonroad.scenario.state import InitialState, KSState
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization import icons
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer


def parallel_lanelets(num_lanes, lanelet_width=4.0, lanelet_length=90.0) -> List[Lanelet]:
    """
    Defines parallel lanelets along the x-axis.
    Lanelet IDs are increasing from right to left. Right boundary of lanelet 0 is at x=0
    :return: List of lanelets
    """
    lon_step = 10
    lanelets = []
    for i in range(num_lanes):
        # Lanes from right to left
        right_y = i * lanelet_width
        left_y = (i + 1) * lanelet_width
        center_y = (i + 0.5) * lanelet_width
        x_points = np.arange(start=0, stop=lanelet_length + lon_step, step=lon_step)
        ones = np.ones((x_points.shape[0]))
        right_vertices_lane = np.stack((x_points, ones * right_y), axis=1)
        left_vertices_lane = np.stack((x_points, ones * left_y), axis=1)
        center_vertices_lane = np.stack((x_points, ones * center_y), axis=1)
        if i == 0:
            adjacent_right = None
        else:
            adjacent_right = i - 1
        if i == num_lanes - 1:
            adjacent_left = None
        else:
            adjacent_left = i + 1
        lanelets.append(
            Lanelet(
                left_vertices_lane,
                center_vertices_lane,
                right_vertices_lane,
                lanelet_id=i,
                adjacent_left=adjacent_left,
                adjacent_right=adjacent_right,
                adjacent_right_same_direction=True,
                adjacent_left_same_direction=True,
            )
        )
    return lanelets


class TestIcons(unittest.TestCase):
    @staticmethod
    def _get_draw_funcs():
        return set(icons._obstacle_icon_assignment().values())

    def test_draw_scenario(self):
        """Draw a full scenario with different icons"""
        lanelets = parallel_lanelets(1)
        scn = Scenario(0.1, ScenarioID())
        scn.lanelet_network.add_lanelet(lanelets[0])
        shape = Rectangle(5, 2)
        state = KSState(time_step=0, position=np.array((0, 2)), orientation=0.0)
        for i, veh_type in enumerate(icons.supported_icons()):
            init_state = copy.deepcopy(state).convert_state_to_state(InitialState())
            init_state.position[0] = (i + 1) * 10
            traj_state = copy.deepcopy(init_state)
            traj_state.time_step = 1
            traj_state.position[0] += 5
            prediction = TrajectoryPrediction(Trajectory(1, [traj_state]), shape)
            obs = DynamicObstacle(i, veh_type, shape, init_state, prediction)
            scn.add_objects(obs)
        rnd = MPRenderer()
        draw_params = MPDrawParams()
        draw_params.time_begin = 1
        draw_params.dynamic_obstacle.draw_icon = True
        scn.draw(rnd, draw_params)
        rnd.render(show=True)

    def test_icons_for_execution(self):
        """Test if the icons return patches without raising an exception."""

        for draw_func in self._get_draw_funcs():
            patch_list = draw_func(
                pos_x=0,
                pos_y=0,
                orientation=0,
            )
            for patch in patch_list:
                assert isinstance(patch, patches.Patch)

    def test_show_icons(self):
        """Draw the icons that are generated from individual draw funcs."""
        with plt.ion():
            fig = plt.figure()
            ax = fig.gca()
            draw_funcs = self._get_draw_funcs()
            for counter, draw_func in enumerate(draw_funcs):
                patch_list = draw_func(
                    pos_x=10 * counter,
                    pos_y=0,
                    orientation=np.pi / 4,
                )
                for patch in patch_list:
                    ax.add_patch(patch)
            ax.set_xlim(-10, len(draw_funcs) * 10)
            ax.set_ylim(-20, 20)
            ax.axis("equal")
            plt.show()

    def test_get_obstacle_icon_patch(self):
        """Draw the obstacle icons in the norm square for all supported vehicle types."""
        with plt.ion():
            fig = plt.figure()
            ax = fig.gca()
            vehicle_types = icons.supported_icons()
            for counter, vehicle_type in enumerate(vehicle_types):
                # Draw the bounding box
                ax.add_patch(
                    patches.Rectangle((110 * counter - 50, -50), 100, 100, color="#0065BD")
                )

                # Add vehicle type as text.
                ax.text(110 * counter, 80, vehicle_type.name.replace("_", "\n"), ha="center")

                patch_list = icons.get_obstacle_icon_patch(
                    obstacle_type=vehicle_type,
                    pos_x=110 * counter,
                    pos_y=0,
                    orientation=0,
                    vehicle_length=100,
                    vehicle_width=100,
                )
                for patch in patch_list:
                    ax.add_patch(patch)
            ax.set_xlim(-60, (len(vehicle_types) - 1) * 110 + 10)
            ax.axis("equal")
            plt.show()

    def test_unsupported_obstacle_type(self):
        """Test if a unsupported obstacle type raises the expected exception."""
        unsupported_obstacle = None
        for obst_type in ObstacleType:
            if obst_type not in icons.supported_icons():
                unsupported_obstacle = obst_type
                break

        if unsupported_obstacle is None:
            assert True
        else:
            self.assertRaises(
                TypeError,
                icons.get_obstacle_icon_patch,
                obstacle_type=unsupported_obstacle,
                pos_x=0,
                pos_y=0,
                orientation=0,
                vehicle_length=100,
                vehicle_width=100,
            )


if __name__ == "__main__":
    unittest.main()
