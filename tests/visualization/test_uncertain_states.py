import math
import os
import unittest

import matplotlib.pyplot as plt
import numpy as np
import shapely

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.util import AngleInterval
from commonroad.geometry.obstacle_shapes.circle_obstacle_shape import CircleObstacleShape
from commonroad.geometry.obstacle_shapes.polygon_obstacle_shape import PolygonObstacleShape
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, StaticObstacle
from commonroad.scenario.state import InitialState, KSState
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.util import approximate_bounding_box_dyn_obstacles


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
        plot_limits = approximate_bounding_box_dyn_obstacles(scenario.dynamic_obstacles, 0)
        f, ax = plt.subplots(1, 1, figsize=(20, 40))
        self.rnd.plot_limits = plot_limits
        self.rnd.ax = ax
        scenario.draw(
            self.rnd,
        )
        self.rnd.render(show=True)

    def test_max_rotation(self):
        rect_size = np.array([5, 3])
        shape = RectObstacleShape(length=rect_size[0], width=rect_size[1])
        state = KSState(
            position=np.array([0, 0]), orientation=AngleInterval(-0.5 * math.pi, 0.5 * math.pi)
        )
        occ = shape.compute_occupancy(state)
        self.assertAlmostEqual(occ.width, np.linalg.norm(rect_size))
        self.assertAlmostEqual(occ.length, np.linalg.norm(rect_size))

    def _test_dynamic_obstacle(self, shape):
        # Uncertain position, certain orientation
        uncertain_positions = [
            RectOccupancy(
                length=2 + i,
                width=2 + i,
                rect_center=shapely.Point([1 + i * 4, 1 + i * 4]),
                orientation=0,
            )
            for i in range(5)
        ]
        uncertain_states = [
            KSState(time_step=i, position=p, orientation=0.25 * math.pi)
            for i, p in enumerate(uncertain_positions)
        ]
        prediction = TrajectoryPrediction(Trajectory(1, uncertain_states[1:]), shape)
        dyn_obs = DynamicObstacle(
            0,
            ObstacleType.CAR,
            shape,
            uncertain_states[0].convert_state_to_state(InitialState()),
            prediction,
        )
        dyn_obs.draw(self.rnd)
        self.rnd.render(show=True)

        # Uncertain position, uncertain orientation
        self.rnd.clear()
        uncertain_orientations = [
            AngleInterval((0.25 - i * 0.125) * math.pi, (0.25 + i * 0.125) * math.pi)
            for i in range(5)
        ]
        uncertain_states = [
            KSState(time_step=i, position=p, orientation=o)
            for i, (p, o) in enumerate(zip(uncertain_positions, uncertain_orientations))
        ]
        prediction = TrajectoryPrediction(Trajectory(1, uncertain_states[1:]), shape)
        dyn_obs = DynamicObstacle(
            0,
            ObstacleType.CAR,
            shape,
            uncertain_states[0].convert_state_to_state(InitialState()),
            prediction,
        )
        dyn_obs.draw(
            self.rnd,
        )
        self.rnd.render(show=True)

        # Certain position, uncertain orientation
        self.rnd.clear()
        uncertain_positions = [np.array([1 + i * 4, 1 + i * 4]) for i in range(5)]
        uncertain_states = [
            KSState(time_step=i, position=p, orientation=o)
            for i, (p, o) in enumerate(zip(uncertain_positions, uncertain_orientations))
        ]
        prediction = TrajectoryPrediction(Trajectory(1, uncertain_states[1:]), shape)
        dyn_obs = DynamicObstacle(
            0,
            ObstacleType.CAR,
            shape,
            uncertain_states[0].convert_state_to_state(InitialState()),
            prediction,
        )
        dyn_obs.draw(
            self.rnd,
        )
        self.rnd.render(show=True)
        self.rnd.clear()

    def test_dynamic_obstacle(self):
        self._test_dynamic_obstacle(RectObstacleShape(length=5, width=3))
        self._test_dynamic_obstacle(CircleObstacleShape(radius=3))
        self._test_dynamic_obstacle(
            PolygonObstacleShape(vertices=((0.0, 0.0), (0.5, 1.0), (1.0, 0.0)))
        )

    def _test_static_obstacle(self, shape):
        state = InitialState(
            position=CircleOccupancy(radius=2, circle_center=shapely.Point([0.0, 0.0])),
            orientation=0.25 * math.pi,
        )
        stat_obs = StaticObstacle(0, ObstacleType.CAR, shape, state)
        stat_obs.draw(
            self.rnd,
        )
        self.rnd.render(show=True)
        self.rnd.clear()

        state = InitialState(
            position=PolygonOccupancy(shapely.Polygon([[0.0, 0.0], [0.5, 1.0], [1.0, 0.0]])),
            orientation=AngleInterval((0.25 - 0.125) * math.pi, (0.25 + 0.125) * math.pi),
        )
        stat_obs = StaticObstacle(0, ObstacleType.CAR, shape, state)
        stat_obs.draw(self.rnd)
        self.rnd.render(show=True)
        self.rnd.clear()

        state = InitialState(
            position=np.array([1, 1]),
            orientation=AngleInterval((0.25 - 0.125) * math.pi, (0.25 + 0.125) * math.pi),
        )
        stat_obs = StaticObstacle(0, ObstacleType.CAR, shape, state)
        stat_obs.draw(self.rnd)
        self.rnd.render(show=True)
        self.rnd.clear()

    def test_static_obstacle(self):
        self._test_static_obstacle(RectObstacleShape(length=5, width=3))
        self._test_static_obstacle(CircleObstacleShape(radius=3))
        self._test_static_obstacle(PolygonObstacleShape(((0.0, 0.0), (0.5, 1.0), (1.0, 0.0))))


if __name__ == "__main__":
    unittest.main()
