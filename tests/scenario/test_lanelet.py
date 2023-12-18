import copy
import unittest
import unittest.mock as mock

import numpy as np

from commonroad.common.common_lanelet import StopLine
from commonroad.geometry.shape import Polygon, Rectangle
from commonroad.prediction.prediction import Trajectory, TrajectoryPrediction
from commonroad.scenario.area import Area
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LineMarking
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import InitialState, STState
from commonroad.scenario.traffic_sign import (
    TrafficSign,
    TrafficSignElement,
    TrafficSignIDGermany,
)


class TestLanelet(unittest.TestCase):
    def test_initialize_lanelet(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, 0.5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array(
            [[0, 0.5], [1, 0.5], [2, 0.5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, 0.5], [8, 0.5]]
        )
        lanelet_id = 5
        predecessor = [1, 2]
        successor = [6, 7]
        adjacent_left = 3
        adjacent_right = 4
        adjacent_right_same_dir = True
        adjacent_left_same_dir = False
        line_marking_right = LineMarking.SOLID
        line_marking_left = LineMarking.DASHED
        stop_line = StopLine(start=np.array([0, 0]), end=np.array([0, 1]), line_marking=LineMarking.SOLID)
        traffic_sign_max_speed = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED, ["15"])
        traffic_sign = TrafficSign(1, [traffic_sign_max_speed], {5}, np.array([0.0, 0.0]))
        area = Area(1)
        lanelet = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            successor,
            adjacent_left,
            adjacent_left_same_dir,
            adjacent_right,
            adjacent_right_same_dir,
            line_marking_left,
            line_marking_right,
            stop_line,
            None,
            None,
            None,
            {traffic_sign.traffic_sign_id},
            None,
            {area.area_id},
        )

        s1 = np.sqrt(1.25)
        s2 = np.sqrt(2.0)
        desired_dist = [
            0.0,
            1.0,
            2.0,
            2.0 + s1,
            2.0 + 2 * s1,
            3.0 + 2 * s1,
            4.0 + 2 * s1,
            4.0 + 2 * s1 + s2,
            5.0 + 2 * s1 + s2,
        ]
        for i, (min_dist, dist) in enumerate(zip(lanelet.inner_distance, lanelet.distance)):
            self.assertAlmostEqual(dist, desired_dist[i])
            self.assertLessEqual(min_dist, dist)
            print(min_dist, dist)

        self.assertEqual(lanelet.lanelet_id, lanelet_id)
        np.testing.assert_array_almost_equal(lanelet.right_vertices, right_vertices)
        np.testing.assert_array_almost_equal(lanelet.left_vertices, left_vertices)
        np.testing.assert_array_almost_equal(lanelet.center_vertices, center_vertices)
        np.testing.assert_array_almost_equal(lanelet.predecessor, predecessor)
        np.testing.assert_array_almost_equal(lanelet.successor, successor)
        self.assertEqual(lanelet.adj_left, adjacent_left)
        self.assertEqual(lanelet.adj_right, adjacent_right)
        self.assertEqual(lanelet.adj_left_same_direction, adjacent_left_same_dir)
        self.assertEqual(lanelet.adj_right_same_direction, adjacent_right_same_dir)
        self.assertEqual(lanelet.line_marking_left_vertices, line_marking_left)
        self.assertEqual(lanelet.line_marking_right_vertices, line_marking_right)
        self.assertSetEqual(lanelet.traffic_signs, {traffic_sign.traffic_sign_id})
        self.assertEqual(lanelet.stop_line, stop_line)
        self.assertEqual(lanelet.adjacent_areas, {area.area_id})

    def test_translate_rotate(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, 0.5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array(
            [[0, 0.5], [1, 0.5], [2, 0.5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, 0.5], [8, 0.5]]
        )
        stop_line = StopLine(start=np.array([0, 0]), end=np.array([0, 1]), line_marking=LineMarking.SOLID)

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1, stop_line=stop_line)

        lanelet.translate_rotate(np.array([2, -4]), np.pi / 2)

        desired_lanelet_center = np.array(
            [[3.5, 2], [3.5, 3], [3.5, 4], [3, 5], [2.5, 6], [2.5, 7], [2.5, 8], [3.5, 9], [3.5, 10]]
        )
        desired_stop_line_start = np.array([4, 2])
        desired_stop_line_end = np.array([3, 2])

        np.testing.assert_array_almost_equal(lanelet.center_vertices, desired_lanelet_center)
        np.testing.assert_array_almost_equal(lanelet.stop_line.start, desired_stop_line_start)
        np.testing.assert_array_almost_equal(lanelet.stop_line.end, desired_stop_line_end)

    def test_convert_to_2d(self):
        right_vertices = np.array([[0, 0, 5], [1, 0, 4], [2, 0, 3], [3, 0.5, 2]])
        left_vertices = np.array([[0, 1, 5], [1, 1, 4], [2, 1, 3], [3, 1.5, 2]])
        center_vertices = np.array([[0, 0.5, 5], [1, 0.5, 4], [2, 0.5, 3], [3, 1, 2]])
        stop_line_mock = mock.MagicMock(spec=StopLine)
        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1, stop_line=stop_line_mock)

        lanelet.convert_to_2d()

        np.testing.assert_allclose(lanelet.right_vertices, np.array([[0, 0], [1, 0], [2, 0], [3, 0.5]]))
        np.testing.assert_allclose(lanelet.left_vertices, np.array([[0, 1], [1, 1], [2, 1], [3, 1.5]]))
        np.testing.assert_allclose(lanelet.center_vertices, np.array([[0, 0.5], [1, 0.5], [2, 0.5], [3, 1]]))
        stop_line_mock.convert_to_2d.assert_called_once()

    def test_translate_invalid(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, 0.5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array(
            [[0, 0.5], [1, 0.5], [2, 0.5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, 0.5], [8, 0.5]]
        )

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1)

        with self.assertRaises(AssertionError):
            lanelet.translate_rotate([2, -4], 320)
        with self.assertRaises(AssertionError):
            lanelet.translate_rotate([3, 5, -7], np.pi / 2)
        with self.assertRaises(AssertionError):
            lanelet.translate_rotate([3], np.pi / 2)
        with self.assertRaises(AssertionError):
            lanelet.translate_rotate(0.0, np.pi / 2)

    def test_interpolate_position(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, 0.5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array(
            [[0, 0.5], [1, 0.5], [2, 0.5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, 0.5], [8, 0.5]]
        )

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1)

        interpolated_position = lanelet.interpolate_position(5.736067977)
        self.assertTrue(len(interpolated_position) == 4)
        desired_position = [[5.5, 1.5], [5.5, 1], [5.5, 2]]

        for i, elt in enumerate(desired_position):
            self.assertAlmostEqual(elt[0], interpolated_position[i][0])
            self.assertAlmostEqual(elt[1], interpolated_position[i][1])
        self.assertEqual(interpolated_position[3], 5)

    def test_interpolate_position_invalid(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, 0.5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array(
            [[0, 0.5], [1, 0.5], [2, 0.5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, 0.5], [8, 0.5]]
        )

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1)

        # values bigger than the distance should not be accepted
        with self.assertRaises(AssertionError):
            lanelet.interpolate_position(10.0)
        # negative values should not be accepted
        with self.assertRaises(AssertionError):
            lanelet.interpolate_position(-1.0)

    def test_get_obstacles(self):
        right_vertices = np.array([[0, 0], [10, 0], [20, 0], [30, 0]])
        left_vertices = np.array([[0, 4], [10, 4], [20, 4], [30, 4]])
        center_vertices = np.array([[0, 2], [10, 2], [20, 2], [30, 2]])

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1)

        rect = Rectangle(5, 3)
        state_list = [
            STState(position=np.array([0.0, 2]), orientation=0.0, time_step=1),
            STState(position=np.array([10.0, 5]), orientation=0.0, time_step=2),
            STState(position=np.array([20.0, 6]), orientation=0.0, time_step=3),
            STState(position=np.array([20.0, 6]), orientation=3.14 / 2.0, time_step=4),
        ]
        trajectory = Trajectory(1, state_list)
        prediction = TrajectoryPrediction(trajectory, rect)

        dynamic_obs = DynamicObstacle(
            obstacle_id=30,
            obstacle_type=ObstacleType.PARKED_VEHICLE,
            prediction=prediction,
            initial_state=InitialState(**{"position": np.array([0, 2]), "orientation": 0, "time_step": 0}),
            obstacle_shape=rect,
        )

        self.assertTrue(lanelet.get_obstacles([dynamic_obs]))
        self.assertTrue(lanelet.get_obstacles([dynamic_obs], time_step=2))
        self.assertFalse(lanelet.get_obstacles([dynamic_obs], time_step=3))
        self.assertTrue(lanelet.get_obstacles([dynamic_obs], time_step=4))

    def test_convert_to_polygon(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, 0.5]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5]])
        center_vertices = np.array([[0, 0.5], [1, 0.5], [2, 0.5], [3, 1]])

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1)

        polygon = lanelet.convert_to_polygon()
        self.assertTrue(isinstance(polygon, Polygon))
        vertices = np.append(right_vertices, np.flip(left_vertices, axis=0), axis=0)
        vertices = np.concatenate((vertices, np.array([[0, 0]])), axis=0)
        vertices = vertices[::-1]
        np.testing.assert_array_almost_equal(polygon.vertices, vertices)

    def test_merge_lanelets(self):
        right_vertices1 = np.array([[0, 0], [1, 0], [2, 0], [3, 0.5]])
        left_vertices1 = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5]])
        center_vertices1 = np.array([[0, 0.5], [1, 0.5], [2, 0.5], [3, 1]])

        lanelet1 = Lanelet(left_vertices1, center_vertices1, right_vertices1, 1, successor=[2], predecessor=[5, 7])

        right_vertices2 = np.array([[3, 0.5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices2 = np.array([[3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices2 = np.array([[3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, 0.5], [8, 0.5]])

        lanelet2 = Lanelet(left_vertices2, center_vertices2, right_vertices2, 2, predecessor=[1], successor=[10, 11])

        lanelet1.add_static_obstacle_to_lanelet(100)
        lanelet2.add_static_obstacle_to_lanelet(101)

        lanelet1.add_dynamic_obstacle_to_lanelet(102, 0)
        lanelet2.add_dynamic_obstacle_to_lanelet(103, 0)
        lanelet1.add_dynamic_obstacle_to_lanelet(102, 1)
        lanelet2.add_dynamic_obstacle_to_lanelet(103, 1)
        lanelet1.add_dynamic_obstacle_to_lanelet(102, 2)
        lanelet2.add_dynamic_obstacle_to_lanelet(103, 2)
        lanelet2.add_dynamic_obstacle_to_lanelet(103, 3)

        merged_lanelet = Lanelet.merge_lanelets(lanelet1, lanelet2)
        self.assertListEqual(merged_lanelet.predecessor, [5, 7])
        self.assertListEqual(merged_lanelet.successor, [10, 11])
        np.testing.assert_array_almost_equal(
            merged_lanelet.right_vertices, np.append(right_vertices1, right_vertices2[1:], axis=0)
        )
        np.testing.assert_array_almost_equal(
            merged_lanelet.left_vertices, np.append(left_vertices1, left_vertices2[1:], axis=0)
        )
        np.testing.assert_array_almost_equal(
            merged_lanelet.center_vertices, np.append(center_vertices1, center_vertices2[1:], axis=0)
        )

        # merging of obstacle assignment
        self.assertSetEqual(merged_lanelet.static_obstacles_on_lanelet, {100, 101})
        self.assertEqual(
            merged_lanelet.dynamic_obstacles_on_lanelet, {0: {102, 103}, 1: {102, 103}, 2: {102, 103}, 3: {103}}
        )

        # merging works also in reverse order
        merged_lanelet = Lanelet.merge_lanelets(lanelet2, lanelet1)
        self.assertListEqual(merged_lanelet.predecessor, [5, 7])
        self.assertListEqual(merged_lanelet.successor, [10, 11])
        np.testing.assert_array_almost_equal(
            merged_lanelet.right_vertices, np.append(right_vertices1, right_vertices2[1:], axis=0)
        )
        np.testing.assert_array_almost_equal(
            merged_lanelet.left_vertices, np.append(left_vertices1, left_vertices2[1:], axis=0)
        )
        np.testing.assert_array_almost_equal(
            merged_lanelet.center_vertices, np.append(center_vertices1, center_vertices2[1:], axis=0)
        )

        # lanelets that have no successor-predecessor connection cannot be merged
        lanelet3 = Lanelet(left_vertices1, center_vertices1, right_vertices1, 3)
        with self.assertRaises(AssertionError):
            Lanelet.merge_lanelets(lanelet3, lanelet2)

    def test_all_lanelets_by_merging_successors_from_lanelet(self):
        v_right1 = np.array([[0, 0], [1, 0]])
        v_left1 = np.array([[0, 1], [1, 1]])
        v_center1 = np.array([[0, 0.5], [1, 0.5]])
        v_right2 = np.array([[1, 0], [2, 0]])
        v_left2 = np.array([[1, 1], [2, 1]])
        v_center2 = np.array([[1, 0.5], [2, 0.5]])
        v_right3 = np.array([[2, 0], [3, 0]])
        v_left3 = np.array([[2, 1], [3, 1]])
        v_center3 = np.array([[2, 0.5], [3, 0.5]])
        lanelet1 = Lanelet(v_left1, v_center1, v_right1, lanelet_id=1, successor=[2], predecessor=[3])
        lanelet2 = Lanelet(v_left2, v_center2, v_right2, lanelet_id=2, successor=[3], predecessor=[1])
        lanelet3 = Lanelet(v_left3, v_center3, v_right3, lanelet_id=3, successor=[1], predecessor=[2])

        lanelet_network = LaneletNetwork.create_from_lanelet_list([lanelet1, lanelet2, lanelet3])

        merged_lanelets, output_ids = Lanelet.all_lanelets_by_merging_successors_from_lanelet(
            lanelet1, lanelet_network, max_length=100.0
        )

        expected = [[1, 2, 3]]
        self.assertListEqual(output_ids[0], expected[0])
        out_vertices_right = np.array([v_right1[0], v_right2[0], v_right3[0], v_right3[1]])
        out_vertices_left = np.array([v_left1[0], v_left2[0], v_left3[0], v_left3[1]])
        out_vertices_center = np.array([v_center1[0], v_center2[0], v_center3[0], v_center3[1]])
        np.testing.assert_array_almost_equal(merged_lanelets[0].left_vertices, out_vertices_left)
        np.testing.assert_array_almost_equal(merged_lanelets[0].right_vertices, out_vertices_right)
        np.testing.assert_array_almost_equal(merged_lanelets[0].center_vertices, out_vertices_center)

        # test length restriction
        merged_lanelets, output_ids = Lanelet.all_lanelets_by_merging_successors_from_lanelet(
            lanelet1, lanelet_network, max_length=1
        )

        expected = [[1, 2]]
        self.assertListEqual(output_ids[0], expected[0])
        out_vertices_right = np.array([v_right1[0], v_right2[0], v_right3[0]])
        out_vertices_left = np.array([v_left1[0], v_left2[0], v_left3[0]])
        out_vertices_center = np.array([v_center1[0], v_center2[0], v_center3[0]])
        np.testing.assert_array_almost_equal(merged_lanelets[0].left_vertices, out_vertices_left)
        np.testing.assert_array_almost_equal(merged_lanelets[0].right_vertices, out_vertices_right)
        np.testing.assert_array_almost_equal(merged_lanelets[0].center_vertices, out_vertices_center)

    def test_all_lanelets_by_merging_predecessors_from_lanelet(self):
        v_right1 = np.array([[0, 0], [1, 0]])
        v_left1 = np.array([[0, 1], [1, 1]])
        v_center1 = np.array([[0, 0.5], [1, 0.5]])
        v_right2 = np.array([[1, 0], [2, 0]])
        v_left2 = np.array([[1, 1], [2, 1]])
        v_center2 = np.array([[1, 0.5], [2, 0.5]])
        v_right3 = np.array([[2, 0], [3, 0]])
        v_left3 = np.array([[2, 1], [3, 1]])
        v_center3 = np.array([[2, 0.5], [3, 0.5]])
        lanelet1 = Lanelet(v_left1, v_center1, v_right1, lanelet_id=1, successor=[2], predecessor=[])
        lanelet2 = Lanelet(v_left2, v_center2, v_right2, lanelet_id=2, successor=[3], predecessor=[1])
        lanelet3 = Lanelet(v_left3, v_center3, v_right3, lanelet_id=3, successor=[], predecessor=[2])

        lanelet_network = LaneletNetwork.create_from_lanelet_list([lanelet1, lanelet2, lanelet3])

        merged_lanelets, output_ids = Lanelet.all_lanelets_by_merging_predecessors_from_lanelet(
            lanelet3, lanelet_network, max_length=100.0
        )
        expected = [[3, 2, 1]]
        self.assertListEqual(output_ids[0], expected[0])
        out_vertices_right = np.array([v_right1[0], v_right2[0], v_right3[0], v_right3[1]])
        out_vertices_left = np.array([v_left1[0], v_left2[0], v_left3[0], v_left3[1]])
        out_vertices_center = np.array([v_center1[0], v_center2[0], v_center3[0], v_center3[1]])
        np.testing.assert_array_almost_equal(merged_lanelets[0].left_vertices, out_vertices_left)
        np.testing.assert_array_almost_equal(merged_lanelets[0].right_vertices, out_vertices_right)
        np.testing.assert_array_almost_equal(merged_lanelets[0].center_vertices, out_vertices_center)

        # test length restriction
        merged_lanelets, output_ids = Lanelet.all_lanelets_by_merging_predecessors_from_lanelet(
            lanelet3, lanelet_network, max_length=1
        )

        expected = [[3, 2]]
        self.assertListEqual(output_ids[0], expected[0])
        out_vertices_right = np.array([v_right2[0], v_right3[0], v_right3[1]])
        out_vertices_left = np.array([v_left2[0], v_left3[0], v_left3[1]])
        out_vertices_center = np.array([v_center2[0], v_center3[0], v_center3[1]])
        np.testing.assert_array_almost_equal(merged_lanelets[0].left_vertices, out_vertices_left)
        np.testing.assert_array_almost_equal(merged_lanelets[0].right_vertices, out_vertices_right)
        np.testing.assert_array_almost_equal(merged_lanelets[0].center_vertices, out_vertices_center)

        # test when lanelet has no predecessors
        lanelet4 = Lanelet(v_left2, v_center2, v_right2, lanelet_id=4, successor=[], predecessor=[])
        merged_lanelets, output_ids = Lanelet.all_lanelets_by_merging_predecessors_from_lanelet(
            lanelet4, lanelet_network
        )
        self.assertEqual(merged_lanelets, [lanelet4])
        self.assertListEqual(output_ids, [[lanelet4.lanelet_id]])

        # test with two created paths (1-4-3 and 1-2-3)
        lanelet4.predecessor = [1]
        lanelet4.successor = [3]
        lanelet1.successor = [2, 4]
        lanelet3.predecessor = [2, 4]

        lanelet_network.add_lanelet(lanelet4)

        merged_lanelets, output_ids = Lanelet.all_lanelets_by_merging_predecessors_from_lanelet(
            lanelet3, lanelet_network
        )

        self.assertListEqual(output_ids, [[3, 2, 1], [3, 4, 1]])

        out_vertices_right = np.array([v_right1[0], v_right2[0], v_right3[0], v_right3[1]])
        out_vertices_left = np.array([v_left1[0], v_left2[0], v_left3[0], v_left3[1]])
        out_vertices_center = np.array([v_center1[0], v_center2[0], v_center3[0], v_center3[1]])

        np.testing.assert_array_almost_equal(merged_lanelets[0].left_vertices, out_vertices_left)
        np.testing.assert_array_almost_equal(merged_lanelets[0].right_vertices, out_vertices_right)
        np.testing.assert_array_almost_equal(merged_lanelets[0].center_vertices, out_vertices_center)

        np.testing.assert_array_almost_equal(merged_lanelets[1].left_vertices, out_vertices_left)
        np.testing.assert_array_almost_equal(merged_lanelets[1].right_vertices, out_vertices_right)
        np.testing.assert_array_almost_equal(merged_lanelets[1].center_vertices, out_vertices_center)

    def test_lanelet_successors_in_range(self):
        v_right1 = np.array([[0, 0], [1, 0]])
        v_left1 = np.array([[0, 1], [1, 1]])
        v_center1 = np.array([[0, 0.5], [1, 0.5]])
        v_right2 = np.array([[1, 0], [2, 0]])
        v_left2 = np.array([[1, 1], [2, 1]])
        v_center2 = np.array([[1, 0.5], [2, 0.5]])
        v_right3 = np.array([[2, 0], [3, 0]])
        v_left3 = np.array([[2, 1], [3, 1]])
        v_center3 = np.array([[2, 0.5], [3, 0.5]])
        v_right4 = np.array([[3, 0], [4, 0]])
        v_left4 = np.array([[3, 1], [4, 1]])
        v_center4 = np.array([[3, 0.5], [4, 0.5]])

        lanelet1 = Lanelet(v_left1, v_center1, v_right1, lanelet_id=1, successor=[2], predecessor=[3])
        lanelet2 = Lanelet(v_left2, v_center2, v_right2, lanelet_id=2, successor=[3], predecessor=[1])
        lanelet3 = Lanelet(v_left3, v_center3, v_right3, lanelet_id=3, successor=[1, 4], predecessor=[2])
        lanelet4 = Lanelet(v_left4, v_center4, v_right4, lanelet_id=4, successor=[], predecessor=[3])
        ln = LaneletNetwork.create_from_lanelet_list([lanelet1, lanelet2, lanelet3, lanelet4])

        paths = lanelet1.find_lanelet_successors_in_range(ln)
        self.assertIn([2, 3], paths)
        self.assertIn([2, 3, 4], paths)
        self.assertTrue(len(paths) == 2)

        paths2 = lanelet1.find_lanelet_successors_in_range(ln, max_length=1.0)
        self.assertIn([2], paths2)
        self.assertTrue(len(paths2) == 1)

    def test_lanelet_predecessors_in_range(self):
        v_right1 = np.array([[0, 0], [1, 0]])
        v_left1 = np.array([[0, 1], [1, 1]])
        v_center1 = np.array([[0, 0.5], [1, 0.5]])
        v_right2 = np.array([[1, 0], [2, 0]])
        v_left2 = np.array([[1, 1], [2, 1]])
        v_center2 = np.array([[1, 0.5], [2, 0.5]])
        v_right3 = np.array([[2, 0], [3, 0]])
        v_left3 = np.array([[2, 1], [3, 1]])
        v_center3 = np.array([[2, 0.5], [3, 0.5]])
        v_right4 = np.array([[3, 0], [4, 0]])
        v_left4 = np.array([[3, 1], [4, 1]])
        v_center4 = np.array([[3, 0.5], [4, 0.5]])

        lanelet1 = Lanelet(v_left1, v_center1, v_right1, lanelet_id=1, successor=[2], predecessor=[])
        lanelet2 = Lanelet(v_left2, v_center2, v_right2, lanelet_id=2, successor=[3], predecessor=[1])
        lanelet3 = Lanelet(v_left3, v_center3, v_right3, lanelet_id=3, successor=[4], predecessor=[2])
        lanelet4 = Lanelet(v_left4, v_center4, v_right4, lanelet_id=4, successor=[], predecessor=[3, 2])
        ln = LaneletNetwork.create_from_lanelet_list([lanelet1, lanelet2, lanelet3, lanelet4])

        paths = lanelet4.find_lanelet_predecessors_in_range(ln)
        self.assertIn([2, 1], paths)
        self.assertIn([3, 2, 1], paths)
        self.assertTrue(len(paths) == 2)

        paths2 = lanelet4.find_lanelet_predecessors_in_range(ln, max_length=1)
        self.assertIn([3], paths2)
        self.assertIn([2], paths2)
        self.assertTrue(len(paths2) == 2)
        print(paths2)

        paths3 = lanelet4.find_lanelet_predecessors_in_range(ln, max_length=1.5)
        self.assertIn([3, 2], paths3)
        self.assertIn([2, 1], paths3)
        self.assertTrue(len(paths3) == 2)

    def test_add_predecessor(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, 0.5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array(
            [[0, 0.5], [1, 0.5], [2, 0.5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, 0.5], [8, 0.5]]
        )
        lanelet_id = 5
        predecessor = [1, 2]
        successor = [6, 7]
        adjacent_left = 3
        adjacent_right = 4
        adjacent_right_same_dir = True
        adjacent_left_same_dir = False

        lanelet = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            successor,
            adjacent_left,
            adjacent_left_same_dir,
            adjacent_right,
            adjacent_right_same_dir,
        )
        lanelet.add_predecessor(10)
        lanelet.add_predecessor(12)
        lanelet.add_predecessor(1)
        self.assertListEqual(lanelet.predecessor, [1, 2, 10, 12])

    def test_remove_predecessor(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, 0.5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array(
            [[0, 0.5], [1, 0.5], [2, 0.5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, 0.5], [8, 0.5]]
        )
        lanelet_id = 5
        predecessor = [1, 2]
        successor = [6, 7]
        adjacent_left = 3
        adjacent_right = 4
        adjacent_right_same_dir = True
        adjacent_left_same_dir = False

        lanelet = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            successor,
            adjacent_left,
            adjacent_left_same_dir,
            adjacent_right,
            adjacent_right_same_dir,
        )
        lanelet.remove_predecessor(1)
        lanelet.remove_predecessor(20)
        self.assertListEqual(lanelet.predecessor, [2])

    def test_add_successor(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, 0.5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array(
            [[0, 0.5], [1, 0.5], [2, 0.5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, 0.5], [8, 0.5]]
        )
        lanelet_id = 5
        predecessor = [1, 2]
        successor = [6, 7]
        adjacent_left = 3
        adjacent_right = 4
        adjacent_right_same_dir = True
        adjacent_left_same_dir = False

        lanelet = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            successor,
            adjacent_left,
            adjacent_left_same_dir,
            adjacent_right,
            adjacent_right_same_dir,
        )
        lanelet.add_successor(10)
        lanelet.add_successor(12)
        lanelet.add_successor(6)
        self.assertListEqual(lanelet.successor, [6, 7, 10, 12])

    def test_remove_successor(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, 0.5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array(
            [[0, 0.5], [1, 0.5], [2, 0.5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, 0.5], [8, 0.5]]
        )
        lanelet_id = 5
        predecessor = [1, 2]
        successor = [6, 7]
        adjacent_left = 3
        adjacent_right = 4
        adjacent_right_same_dir = True
        adjacent_left_same_dir = False

        lanelet = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            successor,
            adjacent_left,
            adjacent_left_same_dir,
            adjacent_right,
            adjacent_right_same_dir,
        )
        lanelet.remove_successor(7)
        lanelet.remove_predecessor(20)
        self.assertListEqual(lanelet.successor, [6])

    def test_equality(self):
        left_vertices = np.array([[0, 1], [1, 1], [8, 1]])
        center_vertices = np.array([[0, 0.5], [1, 0.5], [8, 0.5]])
        right_vertices = np.array([[0, 0], [1, 0], [8, 0]])
        lanelet_id = 7

        lanelet_1 = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id)
        lanelet_2 = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id)
        self.assertTrue(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(np.array([[0, 1.00000000001], [1, 1], [8, 1]]), center_vertices, right_vertices, lanelet_id)
        self.assertTrue(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(np.array([[0, 1.0001], [1, 1], [8, 1]]), center_vertices, right_vertices, lanelet_id)
        self.assertFalse(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(
            left_vertices, np.array([[0, 0.5], [1, 0.49999999999], [8, 0.5]]), right_vertices, lanelet_id
        )
        self.assertTrue(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(left_vertices, np.array([[0, 0.5], [1, 0.6], [8, 0.5]]), right_vertices, lanelet_id)
        self.assertFalse(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(left_vertices, center_vertices, np.array([[0, 0], [1, 0], [7.999999999997, 0]]), lanelet_id)
        self.assertTrue(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(left_vertices, center_vertices, np.array([[0, 0], [1, 0], [7.7, 0]]), lanelet_id)
        self.assertFalse(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(left_vertices, center_vertices, right_vertices, 8)
        self.assertFalse(lanelet_1 == lanelet_2)

        predecessor = [3, 4]
        successor = [6, 7, 8]
        adjacent_left = 9
        adjacent_left_same_dir = True
        adjacent_right = 11
        adjacent_right_same_dir = False

        lanelet_1 = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            successor,
            adjacent_left,
            adjacent_left_same_dir,
            adjacent_right,
            adjacent_right_same_dir,
        )
        lanelet_2 = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            successor,
            adjacent_left,
            adjacent_left_same_dir,
            adjacent_right,
            adjacent_right_same_dir,
        )
        self.assertTrue(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            [3],
            successor,
            adjacent_left,
            adjacent_left_same_dir,
            adjacent_right,
            adjacent_right_same_dir,
        )
        self.assertFalse(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            [4, 3],
            successor,
            adjacent_left,
            adjacent_left_same_dir,
            adjacent_right,
            adjacent_right_same_dir,
        )
        self.assertTrue(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            [2],
            adjacent_left,
            adjacent_left_same_dir,
            adjacent_right,
            adjacent_right_same_dir,
        )
        self.assertFalse(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            [7, 8, 6],
            adjacent_left,
            adjacent_left_same_dir,
            adjacent_right,
            adjacent_right_same_dir,
        )
        self.assertTrue(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            successor,
            1,
            adjacent_left_same_dir,
            adjacent_right,
            adjacent_right_same_dir,
        )
        self.assertFalse(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            successor,
            adjacent_left,
            False,
            adjacent_right,
            adjacent_right_same_dir,
        )
        self.assertFalse(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            successor,
            adjacent_left,
            adjacent_left_same_dir,
            2,
            adjacent_right_same_dir,
        )
        self.assertFalse(lanelet_1 == lanelet_2)

        lanelet_2 = Lanelet(
            left_vertices,
            center_vertices,
            right_vertices,
            lanelet_id,
            predecessor,
            successor,
            adjacent_left,
            adjacent_left_same_dir,
            adjacent_right,
            True,
        )
        self.assertFalse(lanelet_1 == lanelet_2)

    def test_hash(self):
        left_vertices = np.array([[0, 1], [1, 1], [8, 1]])
        center_vertices = np.array([[0, 0.5], [1, 0.5], [8, 0.5]])
        right_vertices = np.array([[0, 0], [1, 0], [8, 0]])
        lanelet_id = 7

        lanelet_1 = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id)
        lanelet_2 = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id)
        self.assertEqual(hash(lanelet_1), hash(lanelet_2))

        right_vertices = np.array([[0, 0], [1, 0.0000000001], [8, 0]])
        lanelet_2 = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id)
        self.assertNotEqual(hash(lanelet_1), hash(lanelet_2))

    def test_orientation_by_position(self):
        left_vertices = np.array([[0, 1], [1, 1], [8, 1]])
        center_vertices = np.array([[0, 0.5], [1, 0.5], [8, 0.5]])
        right_vertices = np.array([[0, 0], [1, 0], [8, 0]])
        lanelet_id = 1

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id)

        self.assertAlmostEqual(lanelet.orientation_by_position(np.array([5.0, 0.5])), 0.0)

        for rotation in np.deg2rad([45, -45, 90, -90]):
            lanelet_2 = copy.deepcopy(lanelet)
            lanelet_2.translate_rotate(translation=np.array([0.0, 0.0]), angle=rotation)

            self.assertAlmostEqual(lanelet_2.orientation_by_position(lanelet_2.center_vertices[2]), rotation)

    def test_orientation_by_position_invalid(self):
        left_vertices = np.array([[0, 1], [1, 1], [8, 1]])
        center_vertices = np.array([[0, 0.5], [1, 0.5], [8, 0.5]])
        right_vertices = np.array([[0, 0], [1, 0], [8, 0]])
        lanelet_id = 1

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id)

        # values bigger than the distance should not be accepted
        with self.assertRaises(AssertionError):
            lanelet.orientation_by_position([-1.0, 3.0])
        # negative values should not be accepted
        with self.assertRaises(AssertionError):
            lanelet.interpolate_position([9.0, -3.0])


class TestStopLine(unittest.TestCase):
    def test_convert_to_2d(self):
        stop_line = StopLine(np.array([0, 0, 0]), np.array([0, 1, 2]), LineMarking.SOLID, {1, 2}, {3, 4})

        stop_line.convert_to_2d()

        self.assertEqual(stop_line.start.shape, (2,))
        np.testing.assert_array_almost_equal(stop_line.start, np.array([0, 0]))
        self.assertEqual(stop_line.end.shape, (2,))
        np.testing.assert_array_almost_equal(stop_line.end, np.array([0, 1]))

    def test_equality(self):
        stop_line_1 = StopLine(np.array([0, 0]), np.array([0, 1]), LineMarking.SOLID, {1, 2}, {3, 4})
        stop_line_2 = StopLine(np.array([0, 0]), np.array([0, 1]), LineMarking.SOLID, {1, 2}, {3, 4})
        self.assertTrue(stop_line_1 == stop_line_2)

        stop_line_2 = StopLine(np.array([0, 0.00000000001]), np.array([0, 1]), LineMarking.SOLID, {1, 2}, {3, 4})
        self.assertTrue(stop_line_1 == stop_line_2)

        stop_line_2 = StopLine(np.array([0, 0.005]), np.array([0, 1]), LineMarking.SOLID, {1, 2}, {3, 4})
        self.assertFalse(stop_line_1 == stop_line_2)

        stop_line_2 = StopLine(np.array([0, 0]), np.array([0, 0.99999999999]), LineMarking.SOLID, {1, 2}, {3, 4})
        self.assertTrue(stop_line_1 == stop_line_2)

        stop_line_2 = StopLine(np.array([0, 0]), np.array([0, 1.05]), LineMarking.SOLID, {1, 2}, {3, 4})
        self.assertFalse(stop_line_1 == stop_line_2)

        stop_line_2 = StopLine(np.array([0, 0]), np.array([0, 1]), LineMarking.DASHED, {1, 2}, {3, 4})
        self.assertFalse(stop_line_1 == stop_line_2)

        stop_line_2 = StopLine(np.array([0, 0]), np.array([0, 1]), LineMarking.SOLID, {1, 2, 5}, {3, 4})
        self.assertFalse(stop_line_1 == stop_line_2)

        stop_line_2 = StopLine(np.array([0, 0]), np.array([0, 1]), LineMarking.SOLID, {1, 2}, {3})
        self.assertFalse(stop_line_1 == stop_line_2)

        stop_line_2 = StopLine(np.array([0, 0]), np.array([0, 1]), LineMarking.SOLID)
        self.assertFalse(stop_line_1 == stop_line_2)

    def test_hash(self):
        stop_line_1 = StopLine(np.array([1, 0]), np.array([0, 1]), LineMarking.SOLID, {1, 2}, {3, 4})
        stop_line_2 = StopLine(np.array([1, 0]), np.array([0, 1]), LineMarking.SOLID, {1, 2}, {3, 4})
        self.assertEqual(hash(stop_line_1), hash(stop_line_2))

        stop_line_2 = StopLine(np.array([1, 0]), np.array([-0.001, 1]), LineMarking.SOLID, {1, 2}, {3, 4})
        self.assertNotEqual(hash(stop_line_1), hash(stop_line_2))


if __name__ == "__main__":
    unittest.main()
