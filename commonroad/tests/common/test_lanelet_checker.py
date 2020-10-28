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
        lanelet_1 = Lanelet(left_vertices=np.array([[0, 1], [1, 1], [2, 1]]),
                            center_vertices=np.array([[0, 0.5], [1, 0.5], [2, 0.5]]),
                            right_vertices=np.array([[0, 0], [1, 0], [2, 0]]), lanelet_id=1,
                            successor=[6], line_marking_left_vertices=LineMarking.SOLID,
                            line_marking_right_vertices=LineMarking.DASHED)

        lanelet_2 = Lanelet(left_vertices=np.array([[0, 0], [1, 0], [2, 0]]),
                            center_vertices=np.array([[0, -0.5], [1, -0.5], [2, -0.5]]),
                            right_vertices=np.array([[0, -1], [1, -1], [2, -1]]), lanelet_id=2,
                            successor=[7], line_marking_left_vertices=LineMarking.SOLID,
                            line_marking_right_vertices=LineMarking.DASHED)

        lanelet_3 = Lanelet(left_vertices=np.array([[0, 2], [1, 2], [2, 2]]),
                            center_vertices=np.array([[0, 1.5], [1, 1.5], [2, 1.5]]),
                            right_vertices=np.array([[0, 1], [1, 1], [2, 1]]), lanelet_id=3,
                            line_marking_left_vertices=LineMarking.SOLID,
                            line_marking_right_vertices=LineMarking.DASHED)

        lanelet_4 = Lanelet(left_vertices=np.array([[0, 3], [1, 3], [2, 3]]),
                            center_vertices=np.array([[0, 2.5], [1, 2.5], [2, 2.5]]),
                            right_vertices=np.array([[0, 2], [1, 2], [2, 2]]), lanelet_id=4,
                            predecessor=[22], line_marking_left_vertices=LineMarking.SOLID,
                            line_marking_right_vertices=LineMarking.DASHED)

        lanelet_5 = Lanelet(left_vertices=np.array([[0, 4], [1, 4], [2, 4]]),
                            center_vertices=np.array([[0, 3.5], [1, 3.5], [2, 3.5]]),
                            right_vertices=np.array([[0, 3], [1, 3], [2, 3]]), lanelet_id=5,
                            successor=[23], line_marking_left_vertices=LineMarking.SOLID,
                            line_marking_right_vertices=LineMarking.DASHED)

        lanelet_6 = Lanelet(left_vertices=np.array([[2, 1], [3, 1], [4, 1]]),
                            center_vertices=np.array([[2.5, -0.5], [3.5, -0.5], [4.5, -0.5]]),
                            right_vertices=np.array([[2, 0], [3, 0], [4, 0]]), lanelet_id=6,
                            predecessor=[1], line_marking_left_vertices=LineMarking.SOLID,
                            line_marking_right_vertices=LineMarking.DASHED)

        lanelet_7 = Lanelet(left_vertices=np.array([[2.5, 0], [3.5, 0], [4.5, 0]]),
                            center_vertices=np.array([[2.5, -0.5], [3.5, -0.5], [4.5, -0.5]]),
                            right_vertices=np.array([[2.5, -1], [3.5, -1], [4.5, -1]]), lanelet_id=7,
                            predecessor=[2], line_marking_left_vertices=LineMarking.SOLID,
                            line_marking_right_vertices=LineMarking.DASHED)

        lanelet_8 = Lanelet(left_vertices=np.array([[2, 2], [3, 2], [4, 2]]),
                            center_vertices=np.array([[2, 1.5], [3, 1.5], [4, 1.5]]),
                            right_vertices=np.array([[2, 1], [3, 1], [4, 1]]), lanelet_id=8,
                            line_marking_left_vertices=LineMarking.SOLID,
                            line_marking_right_vertices=LineMarking.DASHED)

        lanelet_9 = Lanelet(left_vertices=np.array([[0, 5], [1, 5], [2, 5]]),
                            center_vertices=np.array([[0, 4.5], [1, 4.5], [2, 4.5]]),
                            right_vertices=np.array([[0, 4], [1, 4], [2, 4]]), lanelet_id=9,
                            line_marking_left_vertices=LineMarking.SOLID,
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
                             line_marking_left_vertices=LineMarking.SOLID,
                             line_marking_right_vertices=LineMarking.DASHED)

        lanelet_13 = Lanelet(left_vertices=np.array([[2, 6], [3, 6], [4, 6]]),
                             center_vertices=np.array([[2, 5.5], [3, 5.5], [4, 5.5]]),
                             right_vertices=np.array([[2, 5], [3, 5], [4, 5]]), lanelet_id=13,
                             predecessor=[13], successor=[13], line_marking_left_vertices=LineMarking.SOLID,
                             line_marking_right_vertices=LineMarking.DASHED)

        # everything correct
        self.lanelet_network_1 = LaneletNetwork()
        self.lanelet_network_1.add_lanelet(lanelet_1)
        self.lanelet_network_1.add_lanelet(lanelet_6)

        # predecessor ID does not exist
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
