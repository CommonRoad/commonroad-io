import unittest
from commonroad.scenario.lanelet import Lanelet, LineMarking
from commonroad.common.lanelet_checker import *

__author__ = "Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Car@TUM"]
__version__ = "2020.3"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


class TestLaneletChecker(unittest.TestCase):

    def setUp(self):
        lanelet_1_right_vertices = np.array([[0, 0], [1, 0], [2, 0]])
        lanelet_1_left_vertices = np.array([[0, 1], [1, 1], [2, 1]])
        lanelet_1_center_vertices = np.array([[0, 0.5], [1, 0.5], [2, 0.5]])
        lanelet_1_id = 1
        lanelet_1_predecessor = []
        lanelet_1_successor = [6]
        lanelet_1_adjacent_left = 3
        lanelet_1_adjacent_right = 2
        lanelet_1_adjacent_right_same_dir = True
        lanelet_1_adjacent_left_same_dir = True
        lanelet_1_line_marking_right = LineMarking.SOLID
        lanelet_1_line_marking_left = LineMarking.DASHED

        lanelet_2_right_vertices = np.array([[0, -1], [1, -1], [2, -1]])
        lanelet_2_left_vertices = np.array([[0, 0], [1, 0], [2, 0]])
        lanelet_2_center_vertices = np.array([[0, -0.5], [1, -0.5], [2, -0.5]])
        lanelet_2_id = 2
        lanelet_2_predecessor = None
        lanelet_2_successor = [7]
        lanelet_2_adjacent_left = 1
        lanelet_2_adjacent_right = None
        lanelet_2_adjacent_right_same_dir = None
        lanelet_2_adjacent_left_same_dir = True
        lanelet_2_line_marking_right = LineMarking.SOLID
        lanelet_2_line_marking_left = LineMarking.DASHED

        lanelet_3_right_vertices = np.array([[0, 1], [1, 1], [2, 1]])
        lanelet_3_left_vertices = np.array([[0, 2], [1, 2], [2, 2]])
        lanelet_3_center_vertices = np.array([[0, 1.5], [1, 1.5], [2, 1.5]])
        lanelet_3_id = 3
        lanelet_3_predecessor = None
        lanelet_3_successor = None
        lanelet_3_adjacent_left = None
        lanelet_3_adjacent_right = 1
        lanelet_3_adjacent_right_same_dir = True
        lanelet_3_adjacent_left_same_dir = None
        lanelet_3_line_marking_right = LineMarking.SOLID
        lanelet_3_line_marking_left = LineMarking.DASHED

        lanelet_4_right_vertices = np.array([[0, 2], [1, 2], [2, 2]])
        lanelet_4_left_vertices = np.array([[0, 3], [1, 3], [2, 3]])
        lanelet_4_center_vertices = np.array([[0, 2.5], [1, 2.5], [2, 2.5]])
        lanelet_4_id = 4
        lanelet_4_predecessor = [22]
        lanelet_4_successor = None
        lanelet_4_adjacent_left = None
        lanelet_4_adjacent_right = 1
        lanelet_4_adjacent_right_same_dir = True
        lanelet_4_adjacent_left_same_dir = None
        lanelet_4_line_marking_right = LineMarking.SOLID
        lanelet_4_line_marking_left = LineMarking.DASHED

        lanelet_5_right_vertices = np.array([[0, 3], [1, 3], [2, 3]])
        lanelet_5_left_vertices = np.array([[0, 4], [1, 4], [2, 4]])
        lanelet_5_center_vertices = np.array([[0, 3.5], [1, 3.5], [2, 3.5]])
        lanelet_5_id = 5
        lanelet_5_predecessor = None
        lanelet_5_successor = [23]
        lanelet_5_adjacent_left = None
        lanelet_5_adjacent_right = 1
        lanelet_5_adjacent_right_same_dir = True
        lanelet_5_adjacent_left_same_dir = None
        lanelet_5_line_marking_right = LineMarking.SOLID
        lanelet_5_line_marking_left = LineMarking.DASHED

        lanelet_6_right_vertices = np.array([[2, 0], [3, 0], [4, 0]])
        lanelet_6_left_vertices = np.array([[2, 1], [3, 1], [4, 1]])
        lanelet_6_center_vertices = np.array([[2, 0.5], [3, 0.5], [4, 0.5]])
        lanelet_6_id = 6
        lanelet_6_predecessor = [1]
        lanelet_6_successor = []
        lanelet_6_adjacent_left = 3
        lanelet_6_adjacent_right = 2
        lanelet_6_adjacent_right_same_dir = True
        lanelet_6_adjacent_left_same_dir = True
        lanelet_6_line_marking_right = LineMarking.SOLID
        lanelet_6_line_marking_left = LineMarking.DASHED

        lanelet_7_right_vertices = np.array([[2.5, -1], [3.5, -1], [4.5, -1]])
        lanelet_7_left_vertices = np.array([[2.5, 0], [3.5, 0], [4.5, 0]])
        lanelet_7_center_vertices = np.array([[2.5, -0.5], [3.5, -0.5], [4.5, -0.5]])
        lanelet_7_id = 7
        lanelet_7_predecessor = [2]
        lanelet_7_successor = None
        lanelet_7_adjacent_left = 1
        lanelet_7_adjacent_right = None
        lanelet_7_adjacent_right_same_dir = None
        lanelet_7_adjacent_left_same_dir = True
        lanelet_7_line_marking_right = LineMarking.SOLID
        lanelet_7_line_marking_left = LineMarking.DASHED

        lanelet_8_right_vertices = np.array([[2, 1], [3, 1], [4, 1]])
        lanelet_8_left_vertices = np.array([[2, 2], [3, 2], [4, 2]])
        lanelet_8_center_vertices = np.array([[2, 1.5], [3, 1.5], [4, 1.5]])
        lanelet_8_id = 8
        lanelet_8_predecessor = None
        lanelet_8_successor = None
        lanelet_8_adjacent_left = None
        lanelet_8_adjacent_right = 1
        lanelet_8_adjacent_right_same_dir = True
        lanelet_8_adjacent_left_same_dir = None
        lanelet_8_line_marking_right = LineMarking.SOLID
        lanelet_8_line_marking_left = LineMarking.DASHED

        lanelet_1 = Lanelet(lanelet_1_left_vertices, lanelet_1_center_vertices, lanelet_1_right_vertices,
                            lanelet_1_id, lanelet_1_predecessor, lanelet_1_successor, lanelet_1_adjacent_left,
                            lanelet_1_adjacent_left_same_dir, lanelet_1_adjacent_right,
                            lanelet_1_adjacent_right_same_dir, lanelet_1_line_marking_left,
                            lanelet_1_line_marking_right)

        lanelet_2 = Lanelet(lanelet_2_left_vertices, lanelet_2_center_vertices, lanelet_2_right_vertices,
                            lanelet_2_id, lanelet_2_predecessor, lanelet_2_successor, lanelet_2_adjacent_left,
                            lanelet_2_adjacent_left_same_dir, lanelet_2_adjacent_right,
                            lanelet_2_adjacent_right_same_dir, lanelet_2_line_marking_left,
                            lanelet_2_line_marking_right)

        lanelet_3 = Lanelet(lanelet_3_left_vertices, lanelet_3_center_vertices, lanelet_3_right_vertices,
                            lanelet_3_id, lanelet_3_predecessor, lanelet_3_successor, lanelet_3_adjacent_left,
                            lanelet_3_adjacent_left_same_dir, lanelet_3_adjacent_right,
                            lanelet_3_adjacent_right_same_dir, lanelet_3_line_marking_left,
                            lanelet_3_line_marking_right)

        lanelet_4 = Lanelet(lanelet_4_left_vertices, lanelet_4_center_vertices, lanelet_4_right_vertices,
                            lanelet_4_id, lanelet_4_predecessor, lanelet_4_successor, lanelet_4_adjacent_left,
                            lanelet_4_adjacent_left_same_dir, lanelet_4_adjacent_right,
                            lanelet_4_adjacent_right_same_dir, lanelet_4_line_marking_left,
                            lanelet_4_line_marking_right)

        lanelet_5 = Lanelet(lanelet_5_left_vertices, lanelet_5_center_vertices, lanelet_5_right_vertices,
                            lanelet_5_id, lanelet_5_predecessor, lanelet_5_successor, lanelet_5_adjacent_left,
                            lanelet_5_adjacent_left_same_dir, lanelet_5_adjacent_right,
                            lanelet_5_adjacent_right_same_dir, lanelet_5_line_marking_left,
                            lanelet_5_line_marking_right)

        lanelet_6 = Lanelet(lanelet_6_left_vertices, lanelet_6_center_vertices, lanelet_6_right_vertices,
                            lanelet_6_id, lanelet_6_predecessor, lanelet_6_successor, lanelet_6_adjacent_left,
                            lanelet_6_adjacent_left_same_dir, lanelet_6_adjacent_right,
                            lanelet_6_adjacent_right_same_dir, lanelet_6_line_marking_left,
                            lanelet_6_line_marking_right)

        lanelet_7 = Lanelet(lanelet_7_left_vertices, lanelet_7_center_vertices, lanelet_7_right_vertices,
                            lanelet_7_id, lanelet_7_predecessor, lanelet_7_successor, lanelet_7_adjacent_left,
                            lanelet_7_adjacent_left_same_dir, lanelet_7_adjacent_right,
                            lanelet_7_adjacent_right_same_dir, lanelet_7_line_marking_left,
                            lanelet_7_line_marking_right)

        lanelet_8 = Lanelet(lanelet_8_left_vertices, lanelet_8_center_vertices, lanelet_8_right_vertices,
                            lanelet_8_id, lanelet_8_predecessor, lanelet_8_successor, lanelet_8_adjacent_left,
                            lanelet_8_adjacent_left_same_dir, lanelet_8_adjacent_right,
                            lanelet_8_adjacent_right_same_dir, lanelet_8_line_marking_left,
                            lanelet_8_line_marking_right)

        lanelet_9 = Lanelet(left_vertices=np.array([[0, 5], [1, 5], [2, 5]]),
                            center_vertices=np.array([[0, 4.5], [1, 4.5], [2, 4.5]]),
                            right_vertices=np.array([[0, 4], [1, 4], [2, 4]]), lanelet_id=9,
                            successor=[], line_marking_left_vertices=LineMarking.SOLID,
                            line_marking_right_vertices=LineMarking.DASHED)

        lanelet_10 = Lanelet(left_vertices=np.array([[2, 5], [3, 5], [4, 5]]),
                             center_vertices=np.array([[2, 4.5], [3, 4.5], [4, 4.5]]),
                             right_vertices=np.array([[2, 4], [3, 4], [4, 4]]), lanelet_id=10,
                             predecessor=[9], line_marking_left_vertices=LineMarking.SOLID,
                             line_marking_right_vertices=LineMarking.DASHED)

        lanelet_11 = Lanelet(left_vertices=np.array([[0, 6], [1, 6], [2, 6]]),
                             center_vertices=np.array([[0, 5.5], [1, 5.5], [2, 5.5]]),
                             right_vertices=np.array([[0, 5], [1, 5], [2, 5]]), lanelet_id=11,
                             successor=[12], line_marking_left_vertices=LineMarking.SOLID,
                             line_marking_right_vertices=LineMarking.DASHED)

        lanelet_12 = Lanelet(left_vertices=np.array([[2, 6], [3, 6], [4, 6]]),
                             center_vertices=np.array([[2, 5.5], [3, 5.5], [4, 5.5]]),
                             right_vertices=np.array([[2, 5], [3, 5], [4, 5]]), lanelet_id=12,
                             predecessor=[], line_marking_left_vertices=LineMarking.SOLID,
                             line_marking_right_vertices=LineMarking.DASHED)

        lanelet_13 = Lanelet(left_vertices=np.array([[2, 6], [3, 6], [4, 6]]),
                             center_vertices=np.array([[2, 5.5], [3, 5.5], [4, 5.5]]),
                             right_vertices=np.array([[2, 5], [3, 5], [4, 5]]), lanelet_id=13,
                             predecessor=[13], successor=[13], line_marking_left_vertices=LineMarking.SOLID,
                             line_marking_right_vertices=LineMarking.DASHED)

        self.lanelet_network_1 = LaneletNetwork()
        self.lanelet_network_1.add_lanelet(lanelet_1)
        self.lanelet_network_1.add_lanelet(lanelet_6)

        self.lanelet_network_2 = LaneletNetwork()
        self.lanelet_network_2.add_lanelet(lanelet_4)

        self.lanelet_network_3 = LaneletNetwork()
        self.lanelet_network_3.add_lanelet(lanelet_5)

        self.lanelet_network_4 = LaneletNetwork()
        self.lanelet_network_4.add_lanelet(lanelet_2)
        self.lanelet_network_4.add_lanelet(lanelet_7)

        self.lanelet_network_5 = LaneletNetwork()
        self.lanelet_network_5.add_lanelet(lanelet_3)
        self.lanelet_network_5.add_lanelet(lanelet_8)

        self.lanelet_network_6 = LaneletNetwork()
        self.lanelet_network_6.add_lanelet(lanelet_9)
        self.lanelet_network_6.add_lanelet(lanelet_10)

        self.lanelet_network_7 = LaneletNetwork()
        self.lanelet_network_7.add_lanelet(lanelet_11)
        self.lanelet_network_7.add_lanelet(lanelet_12)

        self.lanelet_network_8 = LaneletNetwork()
        self.lanelet_network_8.add_lanelet(lanelet_13)

    def test_check_successor_predecessor_relationships(self):
        self.assertDictEqual(check_successor_predecessor_relationships(self.lanelet_network_1), {1: set(), 6: set()})
        self.assertDictEqual(check_successor_predecessor_relationships(self.lanelet_network_2),
                             {4: {LaneletCheckerErrorCode.ERROR_1, LaneletCheckerErrorCode.ERROR_3}})
        self.assertDictEqual(check_successor_predecessor_relationships(self.lanelet_network_3),
                             {5: {LaneletCheckerErrorCode.ERROR_2, LaneletCheckerErrorCode.ERROR_4}})
        self.assertDictEqual(check_successor_predecessor_relationships(self.lanelet_network_4),
                             {2: {LaneletCheckerErrorCode.ERROR_6},
                              7: {LaneletCheckerErrorCode.ERROR_5}})
        self.assertDictEqual(check_successor_predecessor_relationships(self.lanelet_network_5),
                             {3: {LaneletCheckerErrorCode.ERROR_7},
                              8: set()})
        self.assertDictEqual(check_successor_predecessor_relationships(self.lanelet_network_6),
                             {9: {LaneletCheckerErrorCode.ERROR_4},
                              10: {LaneletCheckerErrorCode.ERROR_3}})
        self.assertDictEqual(check_successor_predecessor_relationships(self.lanelet_network_7),
                             {11: {LaneletCheckerErrorCode.ERROR_4},
                              12: {LaneletCheckerErrorCode.ERROR_3}})
        self.assertDictEqual(check_successor_predecessor_relationships(self.lanelet_network_8),
                             {13: {LaneletCheckerErrorCode.ERROR_5, LaneletCheckerErrorCode.ERROR_6,
                                   LaneletCheckerErrorCode.ERROR_8, LaneletCheckerErrorCode.ERROR_9}})


if __name__ == '__main__':
    unittest.main()
