import unittest
from commonroad.scenario.intersection import Intersection, IntersectionIncomingElement


class TestIntersection(unittest.TestCase):
    def setUp(self) -> None:
        self._incoming_id_1 = 2
        lanelets_1 = {10, 11}
        successors_right_1 = {12, 13}
        successors_straight_1 = {14, 15}
        successors_left_1 = {16, 17}
        left_of_1 = 18
        self._incoming_1 = IntersectionIncomingElement(self._incoming_id_1, lanelets_1, successors_right_1,
                                                       successors_straight_1, successors_left_1, left_of_1)

        self._incoming_id_2 = 3
        lanelets_2 = {20, 21}
        successors_right_2 = {22, 23}
        successors_straight_2 = {24, 25}
        successors_left_2 = {26, 27}
        left_of_2 = 28
        self._incoming_2 = IntersectionIncomingElement(self._incoming_id_2, lanelets_2, successors_right_2,
                                                       successors_straight_2, successors_left_2, left_of_2)

        incoming_id_3 = 4
        lanelets_3 = {40, 41}
        successors_right_3 = {42, 43}
        successors_straight_3 = {44, 45}
        successors_left_3 = {46, 47}
        left_of_3 = 48
        self._incoming_3 = IntersectionIncomingElement(incoming_id_3, lanelets_3, successors_right_3,
                                                       successors_straight_3, successors_left_3, left_of_3)

        self._intersection_id_1 = 1
        self._crossings = {30, 31}
        self._intersection_1 = Intersection(1, [self._incoming_1, self._incoming_2], self._crossings)

    def test_initialization_intersection(self):
        self.assertEqual(self._intersection_1.intersection_id, self._intersection_id_1)
        self.assertEqual(self._intersection_1.incomings[0].incoming_id, self._incoming_id_1)
        self.assertEqual(self._intersection_1.incomings[1].incoming_id, self._incoming_id_2)
        self.assertSetEqual(self._intersection_1.crossings, self._crossings)

        with self.assertWarns(Warning):
            self._intersection_1.intersection_id = 5
        with self.assertWarns(Warning):
            self._intersection_1.crossings = {123}
        with self.assertWarns(Warning):
            self._intersection_1.incomings = [self._incoming_3]

        intersection_id_2 = '5'
        self.assertRaises(AssertionError, Intersection, intersection_id_2, [])

        intersection_3 = Intersection(6, [self._incoming_1, self._incoming_2])
        self.assertSetEqual(intersection_3.crossings, set())

    def test_map_incoming_lanelets(self):
        exp_result = {10: self._incoming_1.incoming_id,
                      11: self._incoming_1.incoming_id,
                      20: self._incoming_2.incoming_id,
                      21: self._incoming_2.incoming_id}

        self.assertListEqual(list(exp_result.keys()), list(self._intersection_1.map_incoming_lanelets.keys()))
        self.assertListEqual(list(exp_result.values()), [val.incoming_id for val
                                                         in self._intersection_1.map_incoming_lanelets.values()])

    def test_equality(self):
        incoming_1 = IntersectionIncomingElement(2, {10, 11}, {12, 13}, {14, 15}, {16, 17}, 18)
        incoming_2 = IntersectionIncomingElement(3, {20, 21}, {22, 23}, {24, 25}, {26, 27}, 28)
        intersection_1 = Intersection(1, [incoming_1, incoming_2], {30, 31})
        intersection_2 = Intersection(1, [incoming_1, incoming_2], {30, 31})
        self.assertTrue(intersection_1 == intersection_2)

        intersection_2 = Intersection(5, [incoming_1, incoming_2], {30, 31})
        self.assertFalse(intersection_1 == intersection_2)

        intersection_2 = Intersection(1, [incoming_1], {30, 31})
        self.assertFalse(intersection_1 == intersection_2)

        intersection_2 = Intersection(1, [incoming_1, incoming_2], {30, 31, 34})
        self.assertFalse(intersection_1 == intersection_2)

    def test_hash(self):
        incoming = IntersectionIncomingElement(3, {20, 21}, {22, 23}, {24, 25}, {26, 27}, 28)
        intersection_1 = Intersection(1, [incoming], {30, 31})
        intersection_2 = Intersection(1, [incoming], {30, 31})
        self.assertEqual(hash(intersection_1), hash(intersection_2))

        incoming = IntersectionIncomingElement(3, {20, 21}, {22, 23}, {24, 25}, {26, 28}, 28)
        intersection_2 = Intersection(1, [incoming], {30, 31})
        self.assertNotEqual(hash(intersection_1), hash(intersection_2))


class TestIntersectionIncomingElement(unittest.TestCase):
    def test_initialization_IntersectionIncomingElement(self):
        incoming_id_1 = 2
        lanelets_1 = {10, 11}
        successors_right_1 = {12, 13}
        successors_straight_1 = {14, 15}
        successors_left_1 = {16, 17}
        left_of_1 = 18
        incoming_1 = IntersectionIncomingElement(incoming_id_1, lanelets_1, successors_right_1, successors_straight_1,
                                                 successors_left_1, left_of_1)

        self.assertEqual(incoming_1.incoming_id, incoming_id_1)
        self.assertSetEqual(incoming_1.incoming_lanelets, lanelets_1)
        self.assertSetEqual(incoming_1.successors_right, successors_right_1)
        self.assertSetEqual(incoming_1.successors_straight, successors_straight_1)
        self.assertSetEqual(incoming_1.successors_left, successors_left_1)
        self.assertEqual(incoming_1.left_of, left_of_1)
        with self.assertWarns(Warning):
            incoming_1.incoming_id = 5
        with self.assertWarns(Warning):
            incoming_1.incoming_lanelets = {123}

        incoming_id_2 = '3'
        self.assertRaises(AssertionError, IntersectionIncomingElement, incoming_id_2)

    def test_equality(self):
        incoming_1 = IntersectionIncomingElement(2, {10, 11}, {12, 13}, {14, 15}, {16, 17}, 18)
        incoming_2 = IntersectionIncomingElement(2, {10, 11}, {12, 13}, {14, 15}, {16, 17}, 18)
        self.assertTrue(incoming_1 == incoming_2)

        incoming_2 = IntersectionIncomingElement(1, {10, 11}, {12, 13}, {14, 15}, {16, 17}, 18)
        self.assertFalse(incoming_1 == incoming_2)

        incoming_2 = IntersectionIncomingElement(2, {11, 11}, {12, 13}, {14, 15}, {16, 17}, 18)
        self.assertFalse(incoming_1 == incoming_2)

        incoming_2 = IntersectionIncomingElement(2, {10, 11}, {12, 17}, {14, 15}, {16, 17}, 18)
        self.assertFalse(incoming_1 == incoming_2)

        incoming_2 = IntersectionIncomingElement(2, {10, 11}, {12, 13}, {5, 15}, {16, 17}, 18)
        self.assertFalse(incoming_1 == incoming_2)

        incoming_2 = IntersectionIncomingElement(2, {10, 11}, {12, 13}, {14, 15}, {7, 17}, 18)
        self.assertFalse(incoming_1 == incoming_2)

        incoming_2 = IntersectionIncomingElement(2, {10, 11}, {12, 13}, {14, 15}, {16, 17}, 5)
        self.assertFalse(incoming_1 == incoming_2)

        incoming_1 = IntersectionIncomingElement(2, {10, 11}, None, {14, 15}, {16, 17}, 18)
        incoming_2 = IntersectionIncomingElement(2, {10, 11}, None, {14, 15}, {16, 17}, 18)
        self.assertTrue(incoming_1 == incoming_2)

        incoming_2 = IntersectionIncomingElement(2, {10, 11}, {1, 2, 3, 4}, {14, 15}, {16, 17}, 18)
        self.assertFalse(incoming_1 == incoming_2)

    def test_hash(self):
        incoming_1 = IntersectionIncomingElement(2, {10, 11}, {12, 13}, {14, 15}, {16, 17}, 18)
        incoming_2 = IntersectionIncomingElement(2, {10, 11}, {12, 13}, {14, 15}, {16, 17}, 18)
        self.assertEqual(hash(incoming_1), hash(incoming_2))

        incoming_2 = IntersectionIncomingElement(2, {10, 11}, {12, 17}, {14, 15}, {16, 17}, 18)
        self.assertNotEqual(hash(incoming_1), hash(incoming_2))


if __name__ == '__main__':
    unittest.main()
