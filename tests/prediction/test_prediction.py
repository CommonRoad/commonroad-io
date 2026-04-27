import unittest
from typing import Dict

import numpy as np
import shapely

from commonroad.geometry.obstacle_shapes.circle_obstacle_shape import CircleObstacleShape
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.occupancy_group import OccupancyGroup
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.prediction.prediction import (
    SetBasedPrediction,
    TimeType,
    TrajectoryPrediction,
)
from commonroad.scenario.state import KSState
from commonroad.scenario.trajectory import Trajectory


class TestTrajectoryPrediction(unittest.TestCase):
    """
    test functionality of class commonroad.prediction.prediction.TrajectoryPrediction
    """

    def setUp(self):
        """create sample trajectory"""
        # Define shape of predicted object
        self.shape = RectObstacleShape(length=2, width=1)

        # Create a trajectory
        state1 = KSState(time_step=2, position=np.array([0.0, 0.0]), orientation=0.0)
        state2 = KSState(time_step=3, position=np.array([1.0, 0.0]), orientation=np.pi / 4)
        state3 = KSState(time_step=4, position=np.array([2.0, 1.0]), orientation=np.pi / 2)
        self.trajectory = Trajectory(2, [state1, state2, state3])

        # Create reversed trajectory
        state1 = KSState(time_step=4, position=np.array([0.0, 0.0]), orientation=0.0)
        state2 = KSState(time_step=3, position=np.array([1.0, 0.0]), orientation=np.pi / 4)
        state3 = KSState(time_step=2, position=np.array([2.0, 1.0]), orientation=np.pi / 2)
        self.reverse_trajectory = Trajectory(2, [state3, state2, state1])

        # Pretend that we transition from lanelet -1 to lanelet -2 on this trajectory
        self.center_lanelet_assignment = {2: {-1}, 3: {-1, -2}, 4: {-2}}
        self.center_lanelet_assignment_different_order = {2: {-1}, 3: {-2, -1}, 4: {-2}}

        # Assume that our shape is already on lanelet -2 while the center is only on lanelet -1 at time-step 2
        self.shape_lanelet_assignment = {2: {-1, -2}, 3: {-1, -2}, 4: {-2}}
        self.shape_lanelet_assignment_different_order = {2: {-2, -1}, 3: {-1, -2}, 4: {-2}}

    def check_occupancy_set_for_trajectory(self, occ_set: Dict[TimeType, Occupancy]):
        """Check if the occupancy set is as expected for a prediction with self.trajectory and self.shape"""
        self.assertEqual(len(occ_set), 3)
        np.testing.assert_array_equal(occ_set[2].center, shapely.Point([0.0, 0.0]))
        np.testing.assert_array_equal(occ_set[3].center, shapely.Point([1.0, 0.0]))
        np.testing.assert_array_equal(occ_set[4].center, shapely.Point([2.0, 1.0]))
        self.assertAlmostEqual(occ_set[2].orientation, 0.0)
        self.assertAlmostEqual(occ_set[3].orientation, np.pi / 4)
        self.assertAlmostEqual(occ_set[4].orientation, np.pi / 2)

    def test_initialization(self):
        """test if TrajectoryPrediction initializes correctly and setter and getter for member variables work."""
        tp = TrajectoryPrediction(self.trajectory, self.shape)
        self.assertEqual(tp.shape.length, 2)
        self.assertEqual(tp.shape.width, 1)
        self.assertEqual(tp.trajectory.final_state.time_step, 4)
        np.testing.assert_array_equal(tp.trajectory.final_state.position, np.array([2.0, 1.0]))

    def test_occupancy_set(self):
        """test if occupancy set is created correctly"""
        occs = TrajectoryPrediction(self.trajectory, self.shape).occupancies
        self.check_occupancy_set_for_trajectory(occs)

    def test_occupancy_set_updates(self):
        pred = TrajectoryPrediction(self.reverse_trajectory, self.shape)
        pred.trajectory = self.trajectory
        self.check_occupancy_set_for_trajectory(pred.occupancies)

    def test_occupancy_at_time_step(self):
        """test if occupancy_at_time_step returns the correct occupancy"""
        tp = TrajectoryPrediction(self.trajectory, CircleObstacleShape(radius=1))
        occ1 = tp.occupancy_at_time_step(3)
        np.testing.assert_array_equal(occ1.center, shapely.Point([1.0, 0.0]))

    def test_eq_is_order_independent_for_center_lanelet_assignment(self):
        """test if __eq__ ignores the order of the values in center_lanelet_assignment"""
        for x, y in zip(
            list(self.center_lanelet_assignment[3]),
            list(self.center_lanelet_assignment_different_order[3]),
        ):
            self.assertNotEqual(
                x, y, msg="This test relies on these two sets having different iteration orders!"
            )
        tp = TrajectoryPrediction(
            self.trajectory,
            self.shape,
            self.center_lanelet_assignment,
            self.shape_lanelet_assignment,
        )
        tp_copy = TrajectoryPrediction(
            self.trajectory,
            self.shape,
            self.center_lanelet_assignment_different_order,
            self.shape_lanelet_assignment,
        )
        # Should be equal as they represent exactly the same trajectory
        self.assertEqual(tp, tp_copy)

    def test_eq_is_order_independent_for_shape_lanelet_assignment(self):
        """test if __eq__ ignores the order of the values in center_lanelet_assignment"""
        for x, y in zip(
            list(self.shape_lanelet_assignment[2]),
            list(self.shape_lanelet_assignment_different_order[2]),
        ):
            self.assertNotEqual(
                x, y, msg="This test relies on these two sets having different iteration orders!"
            )
        tp = TrajectoryPrediction(
            self.trajectory,
            self.shape,
            self.center_lanelet_assignment,
            self.shape_lanelet_assignment_different_order,
        )
        tp_copy = TrajectoryPrediction(
            self.trajectory,
            self.shape,
            self.center_lanelet_assignment,
            self.shape_lanelet_assignment_different_order,
        )
        # Should be equal as they represent exactly the same trajectory
        self.assertEqual(tp, tp_copy)

    def test_translate_rotate(self):
        """test if TrajectoryPrediction can be translated and rotated properly"""
        tp = TrajectoryPrediction(self.trajectory, self.shape)
        tp.translate_rotate(np.array([3.0, 2.2]), -np.pi)
        self.assertIsInstance(tp.occupancies[2], RectOccupancy)
        self.assertAlmostEqual(tp.occupancies[2].orientation, -np.pi)
        self.assertAlmostEqual(tp.occupancies[3].orientation, -3.0 / 4.0 * np.pi)
        self.assertAlmostEqual(tp.occupancies[4].orientation, -np.pi / 2.0)
        self.assertAlmostEqual(tp.occupancies[2].center.x, -3.0)
        self.assertAlmostEqual(tp.occupancies[2].center.y, -2.2)
        self.assertAlmostEqual(tp.occupancies[3].center.x, -4.0)
        self.assertAlmostEqual(tp.occupancies[3].center.y, -2.2)
        self.assertAlmostEqual(tp.occupancies[4].center.x, -5.0)
        self.assertAlmostEqual(tp.occupancies[4].center.y, -3.2)

    def test_hash(self):
        tp1 = TrajectoryPrediction(self.trajectory, self.shape)
        tp2 = TrajectoryPrediction(self.trajectory, self.shape)
        tp3 = TrajectoryPrediction(self.reverse_trajectory, self.shape)
        self.assertEqual(tp1.__hash__(), tp2.__hash__())
        self.assertNotEqual(tp1.__hash__(), tp3.__hash__())

    def test_equality(self):
        tp1 = TrajectoryPrediction(self.trajectory, self.shape)
        tp2 = TrajectoryPrediction(self.trajectory, self.shape)
        tp3 = TrajectoryPrediction(self.reverse_trajectory, self.shape)
        self.assertTrue(tp1.__eq__(tp2))
        self.assertFalse(tp1.__eq__(tp3))


class TestSetBasedPrediction(unittest.TestCase):
    """
    test functionality of class commonroad.prediciton.prediction.SetBasedPrediction
    """

    def setUp(self):
        """create sample occupancies"""
        self.s1 = RectOccupancy(
            length=3,
            width=10,
            rect_center=shapely.Point([0.0, 0.0]),
            orientation=0.0,
        )
        self.s2 = CircleOccupancy(radius=4, circle_center=shapely.Point([2.0, 1.0]))
        self.s3 = OccupancyGroup(occupancies=(self.s1, self.s2))

        self.t1 = 2
        self.t2 = 3
        self.t3 = 3

        self.occs = {self.t1: self.s1, self.t2: self.s3}
        self.occs2 = {self.t1: self.s1, self.t3: self.s2}

    def test_initialization(self):
        """test if SetBasedPrediction initializes correctly and setter and getter work"""
        sp = SetBasedPrediction(2, self.occs)
        self.assertEqual(sp.initial_time_step, 2)
        self.assertIsInstance(sp.occupancies[2], RectOccupancy)
        self.assertIsInstance(sp.occupancies[3], OccupancyGroup)
        self.assertIsInstance(sp.occupancy_at_time_step(2), RectOccupancy)
        self.assertIsInstance(sp.occupancy_at_time_step(3), OccupancyGroup)

    def test_translate_rotate(self):
        """test if SetBasedPrediction can be translated-rotated correctly"""
        sp = SetBasedPrediction(2, self.occs)
        sp.translate_rotate(np.array([-1.0, 1.0]), np.pi / 2)
        self.assertIsInstance(sp.occupancies[2], RectOccupancy)
        self.assertAlmostEqual(sp.occupancies[2].center.x, -1.0)
        self.assertAlmostEqual(sp.occupancies[2].center.y, -1.0)
        self.assertAlmostEqual(sp.occupancies[2].orientation, np.pi / 2)
        self.assertIsInstance(sp.occupancies[3], OccupancyGroup)
        self.assertIsInstance(sp.occupancies[3].occupancies[1], CircleOccupancy)
        self.assertAlmostEqual(sp.occupancies[3].occupancies[1].center.x, -2.0)
        self.assertAlmostEqual(sp.occupancies[3].occupancies[1].center.y, 1.0)

    def test_hash(self):
        sp1 = SetBasedPrediction(2, self.occs)
        sp2 = SetBasedPrediction(2, self.occs)
        sp3 = SetBasedPrediction(2, self.occs2)
        self.assertEqual(sp1.__hash__(), sp2.__hash__())
        self.assertNotEqual(sp1.__hash__(), sp3.__hash__())

    def test_equality(self):
        sp1 = SetBasedPrediction(2, self.occs)
        sp2 = SetBasedPrediction(2, self.occs)
        sp3 = SetBasedPrediction(2, self.occs2)
        self.assertTrue(sp1.__eq__(sp2))
        self.assertFalse(sp1.__eq__(sp3))


if __name__ == "__main__":
    unittest.main()
