import unittest
from typing import List

import numpy as np

from commonroad.common.util import Interval
from commonroad.geometry.shape import Circle, Rectangle, ShapeGroup
from commonroad.prediction.prediction import (
    Occupancy,
    SetBasedPrediction,
    TrajectoryPrediction,
)
from commonroad.scenario.state import KSState
from commonroad.scenario.trajectory import Trajectory


class TestOccupancy(unittest.TestCase):
    """
    test functionality of class commonroad.prediciton.prediction.Occupancy
    """

    def setUp(self):
        """create different shapes"""
        self.s1 = Rectangle(3, 10)
        self.s2 = Circle(4, np.array([2.0, 1.0]))
        self.s3 = ShapeGroup([self.s1, self.s2])

    def test_initialization(self):
        """test if occupancy initializes correctly and setter and getter for member variables work"""
        occ1 = Occupancy(3, self.s3)
        occ2 = Occupancy(Interval(0, 4), self.s2)

        self.assertEqual(occ1.time_step, 3)
        self.assertEqual(occ1.shape.shapes[0].length, 3)
        self.assertEqual(occ1.shape.shapes[0].width, 10)
        self.assertEqual(occ1.shape.shapes[1].radius, 4)
        np.testing.assert_array_equal(occ1.shape.shapes[1].center, np.array([2.0, 1.0]))

        self.assertEqual(occ2.shape.radius, 4)
        np.testing.assert_array_equal(occ2.shape.center, np.array([2.0, 1.0]))
        self.assertEqual(occ2.time_step.start, 0)
        self.assertEqual(occ2.time_step.end, 4)

    def test_translate_rotate(self):
        """test if occupancy can be translated and rotated properly"""
        occ1 = Occupancy(2, self.s1)
        occ1.translate_rotate(np.array([5.5, -1.0]), np.pi / 2)
        self.assertAlmostEqual(occ1.shape.center[0], 1.0)
        self.assertAlmostEqual(occ1.shape.center[1], 5.5)
        self.assertAlmostEqual(occ1.shape.orientation, np.pi / 2)
        self.assertEqual(occ1.time_step, 2)

    def test_hash(self):
        occ1 = Occupancy(2, self.s1)
        occ2 = Occupancy(2, self.s1)
        occ3 = Occupancy(Interval(0, 4), self.s2)
        self.assertEqual(occ1.__hash__(), occ2.__hash__())
        self.assertNotEqual(occ1.__hash__(), occ3.__hash__())

    def test_euality(self):
        occ1 = Occupancy(2, self.s1)
        occ2 = Occupancy(2, self.s1)
        occ3 = Occupancy(Interval(0, 4), self.s2)
        self.assertTrue(occ1.__eq__(occ2))
        self.assertFalse(occ1.__eq__(occ3))


class TestTrajectoryPrediction(unittest.TestCase):
    """
    test functionality of class commonroad.prediction.prediction.TrajectoryPrediction
    """

    def setUp(self):
        """create sample trajectory"""
        # Define shape of predicted object
        self.shape = Rectangle(2, 1)

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

    def check_occupancy_set_for_trajectory(self, occ_set: List[Occupancy]):
        """Check if the occupancy set is as expected for a prediction with self.trajectory and self.shape"""
        self.assertEqual(len(occ_set), 3)
        np.testing.assert_array_equal(occ_set[0].shape.center, np.array([0.0, 0.0]))
        np.testing.assert_array_equal(occ_set[1].shape.center, np.array([1.0, 0.0]))
        np.testing.assert_array_equal(occ_set[2].shape.center, np.array([2.0, 1.0]))
        self.assertAlmostEqual(occ_set[0].shape.orientation, 0.0)
        self.assertAlmostEqual(occ_set[1].shape.orientation, np.pi / 4)
        self.assertAlmostEqual(occ_set[2].shape.orientation, np.pi / 2)
        self.assertEqual(occ_set[0].time_step, 2)
        self.assertEqual(occ_set[1].time_step, 3)
        self.assertEqual(occ_set[2].time_step, 4)

    def test_initialization(self):
        """test if TrajectoryPrediction initializes correctly and setter and getter for member variables work."""
        tp = TrajectoryPrediction(self.trajectory, self.shape)
        self.assertEqual(tp.shape.length, 2)
        self.assertEqual(tp.shape.width, 1)
        self.assertEqual(tp.trajectory.final_state.time_step, 4)
        np.testing.assert_array_equal(tp.trajectory.final_state.position, np.array([2.0, 1.0]))

    def test_occupancy_set(self):
        """test if occupancy set is created correctly"""
        occ_set = TrajectoryPrediction(self.trajectory, self.shape).occupancy_set
        self.check_occupancy_set_for_trajectory(occ_set)

    def test_occupancy_set_updates(self):
        pred = TrajectoryPrediction(self.reverse_trajectory, self.shape)
        pred.trajectory = self.trajectory
        self.check_occupancy_set_for_trajectory(pred.occupancy_set)

    def test_occupancy_at_time_step(self):
        """test if occupancy_at_time_step returns the correct occupancy"""
        tp = TrajectoryPrediction(self.trajectory, Circle(1))
        occ1 = tp.occupancy_at_time_step(3)
        self.assertEqual(occ1.time_step, 3)
        np.testing.assert_array_equal(occ1.shape.center, np.array([1.0, 0.0]))

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
        self.assertIsInstance(tp.occupancy_set[0].shape, Rectangle)
        self.assertAlmostEqual(tp.occupancy_set[0].shape.orientation, -np.pi)
        self.assertAlmostEqual(tp.occupancy_set[1].shape.orientation, -3.0 / 4.0 * np.pi)
        self.assertAlmostEqual(tp.occupancy_set[2].shape.orientation, -np.pi / 2.0)
        self.assertAlmostEqual(tp.occupancy_set[0].shape.center[0], -3.0)
        self.assertAlmostEqual(tp.occupancy_set[0].shape.center[1], -2.2)
        self.assertAlmostEqual(tp.occupancy_set[1].shape.center[0], -4.0)
        self.assertAlmostEqual(tp.occupancy_set[1].shape.center[1], -2.2)
        self.assertAlmostEqual(tp.occupancy_set[2].shape.center[0], -5.0)
        self.assertAlmostEqual(tp.occupancy_set[2].shape.center[1], -3.2)

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
        s1 = Rectangle(3, 10)
        s2 = Circle(4, np.array([2.0, 1.0]))
        s3 = ShapeGroup([s1, s2])
        self.occ1 = Occupancy(2, s1)
        self.occ2 = Occupancy(3, s3)
        self.occ3 = Occupancy(3, s2)

    def test_initialization(self):
        """test if SetBasedPrediction initializes correctly and setter and getter work"""
        sp = SetBasedPrediction(2, [self.occ1, self.occ2])
        self.assertEqual(sp.initial_time_step, 2)
        self.assertIsInstance(sp.occupancy_set[0].shape, Rectangle)
        self.assertIsInstance(sp.occupancy_set[1].shape, ShapeGroup)
        self.assertIsInstance(sp.occupancy_at_time_step(2).shape, Rectangle)
        self.assertIsInstance(sp.occupancy_at_time_step(3).shape, ShapeGroup)

    def test_translate_rotate(self):
        """test if SetBasedPrediction can be translated-rotated correctly"""
        sp = SetBasedPrediction(2, [self.occ1, self.occ2])
        sp.translate_rotate(np.array([-1.0, 1.0]), np.pi / 2)
        self.assertIsInstance(sp.occupancy_set[0].shape, Rectangle)
        self.assertAlmostEqual(sp.occupancy_set[0].shape.center[0], -1.0)
        self.assertAlmostEqual(sp.occupancy_set[0].shape.center[1], -1.0)
        self.assertAlmostEqual(sp.occupancy_set[0].shape.orientation, np.pi / 2)
        self.assertIsInstance(sp.occupancy_set[1].shape, ShapeGroup)
        self.assertIsInstance(sp.occupancy_set[1].shape.shapes[1], Circle)
        self.assertAlmostEqual(sp.occupancy_set[1].shape.shapes[1].center[0], -2.0)
        self.assertAlmostEqual(sp.occupancy_set[1].shape.shapes[1].center[1], 1.0)

    def test_hash(self):
        sp1 = SetBasedPrediction(2, [self.occ1, self.occ2])
        sp2 = SetBasedPrediction(2, [self.occ1, self.occ2])
        sp3 = SetBasedPrediction(2, [self.occ1, self.occ3])
        self.assertEqual(sp1.__hash__(), sp2.__hash__())
        self.assertNotEqual(sp1.__hash__(), sp3.__hash__())

    def test_equality(self):
        sp1 = SetBasedPrediction(2, [self.occ1, self.occ2])
        sp2 = SetBasedPrediction(2, [self.occ1, self.occ2])
        sp3 = SetBasedPrediction(2, [self.occ1, self.occ3])
        self.assertTrue(sp1.__eq__(sp2))
        self.assertFalse(sp1.__eq__(sp3))


if __name__ == "__main__":
    unittest.main()
