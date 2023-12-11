import math
import os
import unittest

import matplotlib.pyplot as plt
import numpy as np

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.util import AngleInterval
from commonroad.geometry.shape import Rectangle, occupancy_shape_from_state, Circle, Polygon
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, StaticObstacle
from commonroad.scenario.state import KSState, InitialState
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.util import approximate_bounding_box_dyn_obstacles
from commonroad.visualization.draw_params import MPDrawParams


class TestUncertainStates(unittest.TestCase):

    def setUp(self) -> None:
        super().setUp()
        self.params = MPDrawParams()
        self.params["occupancy"]["draw_occupancies"] = True
        self.rnd = MPRenderer(self.params)

    def test_draw(self):
        full_path = os.path.dirname(os.path.abspath(__file__))
        scenario_path = full_path + "/../test_scenarios/DEU_A9-3_1_T-1.xml"
        scenario, _ = CommonRoadFileReader(scenario_path).open()
        plot_limits = approximate_bounding_box_dyn_obstacles(
                scenario.dynamic_obstacles, 0)
        f, ax = plt.subplots(1, 1, figsize=(20, 40))
        self.rnd.plot_limits = plot_limits
        self.rnd.ax = ax
        scenario.draw(self.rnd, )
        self.rnd.render(show=True)

    def test_max_rotation(self):
        rect_size = np.array([5, 3])
        shape = Rectangle(*rect_size)
        state = KSState(position=np.array([0, 0]), orientation=AngleInterval(-0.5 * math.pi, 0.5 * math.pi))
        occ = occupancy_shape_from_state(shape, state)
        self.assertAlmostEqual(occ.width, np.linalg.norm(rect_size))
        self.assertAlmostEqual(occ.length, np.linalg.norm(rect_size))

    def _test_dynamic_obstacle(self, shape):
        # Uncertain position, certain orientation
        uncertain_positions = [
            Rectangle(2 + i, 2 + i, np.array([1 + i * 4, 1 + i * 4])) for i in
            range(5)]
        uncertain_states = [
            KSState(time_step=i, position=p, orientation=0.25 * math.pi) for i, p
            in enumerate(uncertain_positions)]
        prediction = TrajectoryPrediction(Trajectory(1, uncertain_states[1:]),
                                          shape)
        dyn_obs = DynamicObstacle(0, ObstacleType.CAR, shape,
                                  uncertain_states[0].convert_state_to_state(InitialState()), prediction)
        dyn_obs.draw(self.rnd)
        self.rnd.render(show=True)

        # Uncertain position, uncertain orientation
        self.rnd.clear()
        uncertain_orientations = [AngleInterval((0.25 - i * 0.125) * math.pi,
                                                (0.25 + i * 0.125) * math.pi)
                                  for i in range(5)]
        uncertain_states = [KSState(time_step=i, position=p, orientation=o) for
                            i, (p, o) in enumerate(
                zip(uncertain_positions, uncertain_orientations))]
        prediction = TrajectoryPrediction(Trajectory(1, uncertain_states[1:]),
                                          shape)
        dyn_obs = DynamicObstacle(0, ObstacleType.CAR, shape,
                                  uncertain_states[0].convert_state_to_state(InitialState()), prediction)
        dyn_obs.draw(self.rnd, )
        self.rnd.render(show=True)

        # Certain position, uncertain orientation
        self.rnd.clear()
        uncertain_positions = [np.array([1 + i * 4, 1 + i * 4]) for i in
                               range(5)]
        uncertain_states = [KSState(time_step=i, position=p, orientation=o) for
                            i, (p, o) in enumerate(
                zip(uncertain_positions, uncertain_orientations))]
        prediction = TrajectoryPrediction(Trajectory(1, uncertain_states[1:]),
                                          shape)
        dyn_obs = DynamicObstacle(0, ObstacleType.CAR, shape,
                                  uncertain_states[0].convert_state_to_state(InitialState()), prediction)
        dyn_obs.draw(self.rnd, )
        self.rnd.render(show=True)
        self.rnd.clear()

    def test_dynamic_obstacle(self):
        self._test_dynamic_obstacle(Rectangle(5, 3))
        self._test_dynamic_obstacle(Circle(3))
        self._test_dynamic_obstacle(
            Polygon(np.array([[0.0, 0.0], [0.5, 1.0], [1.0, 0.0]])))

    def _test_static_obstacle(self, shape):
        state = InitialState(position=Circle(2), orientation=0.25 * math.pi)
        stat_obs = StaticObstacle(0, ObstacleType.CAR, shape, state)
        stat_obs.draw(self.rnd, )
        self.rnd.render(show=True)
        self.rnd.clear()

        state = InitialState(position=Polygon(np.array([[0.0, 0.0], [0.5, 1.0], [1.0, 0.0]])),
                             orientation=AngleInterval((0.25 - 0.125) * math.pi, (0.25 + 0.125) * math.pi))
        stat_obs = StaticObstacle(0, ObstacleType.CAR, shape, state)
        stat_obs.draw(self.rnd)
        self.rnd.render(show=True)
        self.rnd.clear()

        state = InitialState(position=np.array([1, 1]),
                             orientation=AngleInterval((0.25 - 0.125) * math.pi, (0.25 + 0.125) * math.pi))
        stat_obs = StaticObstacle(0, ObstacleType.CAR, shape, state)
        stat_obs.draw(self.rnd)
        self.rnd.render(show=True)
        self.rnd.clear()

    def test_static_obstacle(self):
        self._test_static_obstacle(Rectangle(5, 3))
        self._test_static_obstacle(Circle(3))
        self._test_static_obstacle(
                Polygon(np.array([[0.0, 0.0], [0.5, 1.0], [1.0, 0.0]])))


if __name__ == "__main__":
    unittest.main()
