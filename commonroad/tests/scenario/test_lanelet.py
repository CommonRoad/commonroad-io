import unittest
import numpy as np
from commonroad.scenario.lanelet import Lanelet, LineMarking, LaneletNetwork
from commonroad.geometry.shape import Polygon, Rectangle
from commonroad.prediction.prediction import Trajectory, TrajectoryPrediction
from commonroad.scenario.obstacle import State, DynamicObstacle, StaticObstacle, ObstacleType


class TestLanelet(unittest.TestCase):
    def test_initialize_lanelet(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5], [8, .5]])
        lanelet_id = 5
        predecessor = [1, 2]
        successor = [6, 7]
        adjacent_left = 3
        adjacent_right = 4
        adjacent_right_same_dir = True
        adjacent_left_same_dir = False
        speed_limit = 15
        line_marking_right = LineMarking.SOLID
        line_marking_left = LineMarking.DASHED

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id, predecessor, successor,
                          adjacent_left, adjacent_left_same_dir, adjacent_right, adjacent_right_same_dir, speed_limit,
                          line_marking_left, line_marking_right)

        s1 = np.sqrt(1.25)
        s2 = np.sqrt(2.0)
        desired_dist = [0.0, 1.0, 2.0, 2.0 + s1, 2.0 + 2*s1, 3.0 + 2*s1, 4.0 + 2*s1, 4.0 + 2*s1 + s2, 5.0 + 2*s1 + s2]
        for i, dist in enumerate(lanelet.distance):
            self.assertAlmostEqual(dist, desired_dist[i])

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

    def test_translate_rotate(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5], [8, .5]])

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1)

        lanelet.translate_rotate(np.array([2, -4]), np.pi/2)

        desired_center = np.array([[3.5, 2], [3.5, 3], [3.5, 4], [3, 5], [2.5, 6], [2.5, 7], [2.5,  8], [3.5, 9],
                                   [3.5, 10]])

        np.testing.assert_array_almost_equal(lanelet.center_vertices, desired_center)

    def test_translate_invalid(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5], [8, .5]])

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
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5], [8, .5]])

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1)

        interpolated_position = lanelet.interpolate_position(5.736067977)
        desired_position = [[5.5, 1.5], [5.5, 1], [5.5, 2]]

        for i, elt in enumerate(desired_position):
            self.assertAlmostEqual(elt[0], interpolated_position[i][0])
            self.assertAlmostEqual(elt[1], interpolated_position[i][1])
        self.assertEqual(interpolated_position[3], 5)

    def test_interpolate_position_invalid(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5], [8, .5]])

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1)

        # values bigger than the distance should not be accepted
        with self.assertRaises(AssertionError):
            lanelet.interpolate_position(10.0)
        # negative values should not be accepted
        with self.assertRaises(AssertionError):
            lanelet.interpolate_position(-1.0)

    def test_get_obstacles(self):
        print("TEST")
        right_vertices = np.array([[0, 0], [10, 0], [20, 0], [30, 0]])
        left_vertices = np.array([[0, 4], [10, 4], [20, 4], [30, 4]])
        center_vertices = np.array([[0, 2], [10, 2], [20, 2], [30, 2]])

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1)

        rect = Rectangle(5, 3)
        state_list = [State(position=np.array([0.0, 2]), orientation=0.0, time_step=1),
                      State(position=np.array([10.0, 5]), orientation=0.0, time_step=2),
                      State(position=np.array([20.0, 6]), orientation=0.0, time_step=3),
                      State(position=np.array([20.0, 6]), orientation=3.14/2., time_step=4)]
        trajectory = Trajectory(1, state_list)
        prediction = TrajectoryPrediction(trajectory, rect)

        # without inital_state
        dynamic_obs = DynamicObstacle(obstacle_id=30, obstacle_type=ObstacleType.PARKED_VEHICLE,
                                      prediction=prediction,
                                      initial_state=State(**{'position': np.array([0, 2]), 'orientation': 0,
                                                             'time_step': 0}),
                                      obstacle_shape=rect)

        self.assertTrue(lanelet.get_obstacles([dynamic_obs]))
        self.assertTrue(lanelet.get_obstacles([dynamic_obs], time_step=2))
        self.assertFalse(lanelet.get_obstacles([dynamic_obs], time_step=3))
        self.assertTrue(lanelet.get_obstacles([dynamic_obs], time_step=4))

    def test_convert_to_polygon(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5]])
        center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1]])

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1)

        polygon = lanelet.convert_to_polygon()
        self.assertTrue(isinstance(polygon, Polygon))
        vertices = np.append(right_vertices, np.flip(left_vertices, axis=0), axis=0)
        vertices = np.concatenate((vertices, np.array([[0, 0]])), axis=0)
        vertices = vertices[::-1]
        np.testing.assert_array_almost_equal(polygon.vertices, vertices)

    def test_merge_lanelets(self):
        right_vertices1 = np.array([[0, 0], [1, 0], [2, 0], [3, .5]])
        left_vertices1 = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5]])
        center_vertices1 = np.array([[0, .5], [1, .5], [2, .5], [3, 1]])

        lanelet1 = Lanelet(left_vertices1, center_vertices1, right_vertices1, 1,
                           successor=[2], predecessor=[5, 7])

        right_vertices2 = np.array([[3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices2 = np.array([[3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices2 = np.array([[3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5], [8, .5]])

        lanelet2 = Lanelet(left_vertices2, center_vertices2, right_vertices2, 2,
                           predecessor=[1], successor=[10, 11])

        merged_lanelet = Lanelet.merge_lanelets(lanelet1, lanelet2)
        self.assertListEqual(merged_lanelet.predecessor, [5, 7])
        self.assertListEqual(merged_lanelet.successor, [10, 11])
        np.testing.assert_array_almost_equal(merged_lanelet.right_vertices,
                                             np.append(right_vertices1, right_vertices2[1:], axis=0))
        np.testing.assert_array_almost_equal(merged_lanelet.left_vertices,
                                             np.append(left_vertices1, left_vertices2[1:], axis=0))
        np.testing.assert_array_almost_equal(merged_lanelet.center_vertices,
                                             np.append(center_vertices1, center_vertices2[1:], axis=0))

        # merging works also in reverse order
        merged_lanelet = Lanelet.merge_lanelets(lanelet2, lanelet1)
        self.assertListEqual(merged_lanelet.predecessor, [5, 7])
        self.assertListEqual(merged_lanelet.successor, [10, 11])
        np.testing.assert_array_almost_equal(merged_lanelet.right_vertices,
                                             np.append(right_vertices1, right_vertices2[1:], axis=0))
        np.testing.assert_array_almost_equal(merged_lanelet.left_vertices,
                                             np.append(left_vertices1, left_vertices2[1:], axis=0))
        np.testing.assert_array_almost_equal(merged_lanelet.center_vertices,
                                             np.append(center_vertices1, center_vertices2[1:], axis=0))

        # lanelets that have no successor-predecessor connection cannot be merged
        lanelet3 = Lanelet(left_vertices1, center_vertices1, right_vertices1, 3)
        with self.assertRaises(AssertionError):
            Lanelet.merge_lanelets(lanelet3, lanelet2)

    def test_all_lanelets_by_merging_successors_from_lanelet(self):
        v_right1 = np.array([[0, 0], [1, 0]])
        v_left1 = np.array([[0, 1], [1, 1]])
        v_center1 = np.array([[0, .5], [1, .5]])
        v_right2 = np.array([[1, 0], [2, 0]])
        v_left2 = np.array([[1, 1], [2, 1]])
        v_center2 = np.array([[1, .5], [2, .5]])
        v_right3 = np.array([[2, 0], [3, 0]])
        v_left3 = np.array([[2, 1], [3, 1]])
        v_center3 = np.array([[2, .5], [3, .5]])
        lanelet1 = Lanelet(v_left1, v_center1, v_right1, lanelet_id=1, successor=[2], predecessor=[3])
        lanelet2 = Lanelet(v_left2, v_center2, v_right2, lanelet_id=2, successor=[3], predecessor=[1])
        lanelet3 = Lanelet(v_left3, v_center3, v_right3, lanelet_id=3, successor=[1], predecessor=[2])

        lanelet_network = LaneletNetwork.create_from_lanelet_list([lanelet1,
                                                                   lanelet2,
                                                                   lanelet3])

        merged_lanelets, output_ids = Lanelet.all_lanelets_by_merging_successors_from_lanelet(lanelet1,
                                                                                              lanelet_network,
                                                                                              max_length=100.0)

        expected = [[1, 2, 3]]
        self.assertListEqual(output_ids[0], expected[0])
        out_vertices_right = np.array([v_right1[0], v_right2[0], v_right3[0], v_right3[1]])
        out_vertices_left = np.array([v_left1[0], v_left2[0], v_left3[0], v_left3[1]])
        out_vertices_center = np.array([v_center1[0], v_center2[0], v_center3[0], v_center3[1]])
        np.testing.assert_array_almost_equal(merged_lanelets[0].left_vertices, out_vertices_left)
        np.testing.assert_array_almost_equal(merged_lanelets[0].right_vertices, out_vertices_right)
        np.testing.assert_array_almost_equal(merged_lanelets[0].center_vertices, out_vertices_center)

        ### test length restriction
        merged_lanelets, output_ids = Lanelet.all_lanelets_by_merging_successors_from_lanelet(lanelet1,
                                                                                              lanelet_network,
                                                                                              max_length=1.0)

        expected = [[1, 2]]
        self.assertListEqual(output_ids[0], expected[0])
        out_vertices_right = np.array([v_right1[0], v_right2[0], v_right3[0]])
        out_vertices_left = np.array([v_left1[0], v_left2[0], v_left3[0]])
        out_vertices_center = np.array([v_center1[0], v_center2[0], v_center3[0]])
        np.testing.assert_array_almost_equal(merged_lanelets[0].left_vertices, out_vertices_left)
        np.testing.assert_array_almost_equal(merged_lanelets[0].right_vertices, out_vertices_right)
        np.testing.assert_array_almost_equal(merged_lanelets[0].center_vertices, out_vertices_center)


if __name__ == '__main__':
    unittest.main()
