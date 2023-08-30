import unittest
from commonroad.scenario.intersection import Intersection, IncomingGroup, OutgoingGroup, CrossingGroup


class TestIntersection(unittest.TestCase):
    def setUp(self) -> None:
        self._incoming_id_1 = 2
        lanelets_1 = {10, 11}
        outgoing_right_1 = {12, 13}
        outgoing_straight_1 = {14, 15}
        outgoing_left_1 = {16, 17}
        self._crossing_group1 = CrossingGroup(1231, {1}, self._incoming_id_1, 19)
        self._outgoing_group1 = OutgoingGroup(19, {12, 13, 14, 15, 16, 17})
        self._incoming_1 = IncomingGroup(self._incoming_id_1, lanelets_1, 19, outgoing_right_1,
                                         outgoing_straight_1, outgoing_left_1)

        self._incoming_id_2 = 3
        lanelets_2 = {20, 21}
        outgoing_right_2 = {22, 23}
        outgoing_straight_2 = {24, 25}
        outgoing_left_2 = {26, 27}
        self._crossing_group2 = CrossingGroup(1232, {2}, self._incoming_id_2, 29)
        self._outgoing_group2 = OutgoingGroup(29, {22, 23, 24, 25, 26, 27})
        self._incoming_2 = IncomingGroup(self._incoming_id_2, lanelets_2, 29, outgoing_right_2,
                                         outgoing_straight_2, outgoing_left_2)

        incoming_id_3 = 4
        lanelets_3 = {40, 41}
        outgoing_right_3 = {42, 43}
        outgoing_straight_3 = {44, 45}
        outgoing_left_3 = {46, 47}
        self._crossing_group3 = CrossingGroup(1233, {3}, incoming_id_3, 49)
        self._outgoing_group3 = OutgoingGroup(49, {42, 43, 44, 45, 46, 47})
        self._incoming_3 = IncomingGroup(incoming_id_3, lanelets_3, 49, outgoing_right_3,
                                         outgoing_straight_3, outgoing_left_3)

        self._intersection_id_1 = 1

        self._intersection_1 = Intersection(1, [self._incoming_1, self._incoming_2],
                                            [self._outgoing_group1, self._outgoing_group2],
                                            [self._crossing_group1, self._crossing_group2, self._crossing_group3])

    def test_initialization_intersection(self):
        self.assertEqual(self._intersection_1.intersection_id, self._intersection_id_1)
        self.assertEqual(self._intersection_1.incomings[0].incoming_id, self._incoming_id_1)
        self.assertEqual(self._intersection_1.incomings[1].incoming_id, self._incoming_id_2)

        intersection_id_2 = '5'
        self.assertRaises(AssertionError, Intersection, intersection_id_2, [])

    def test_map_incoming_lanelets(self):
        exp_result = {10: self._incoming_1.incoming_id,
                      11: self._incoming_1.incoming_id,
                      20: self._incoming_2.incoming_id,
                      21: self._incoming_2.incoming_id}

        self.assertListEqual(list(exp_result.keys()), list(self._intersection_1.map_incoming_lanelets.keys()))
        self.assertListEqual(list(exp_result.values()), [val.incoming_id for val
                                                         in self._intersection_1.map_incoming_lanelets.values()])

    def test_equality(self):
        incoming_1 = IncomingGroup(2, {10, 11}, 19, {12, 13}, {14, 15}, {16, 17})
        incoming_2 = IncomingGroup(3, {20, 21}, 29, {22, 23}, {24, 25}, {26, 27})
        outgoing_1 = OutgoingGroup(19, {12, 13, 14, 15, 16, 17})
        outgoing_2 = OutgoingGroup(29, {22, 23, 24, 25, 26, 27})
        crossing_1 = CrossingGroup(1234, {625}, 2, 19)
        crossing_2 = CrossingGroup(1235, {812}, 3, 29)
        intersection_1 = Intersection(1, [incoming_1, incoming_2], [outgoing_1, outgoing_2], [crossing_1, crossing_2])
        intersection_2 = Intersection(1, [incoming_1, incoming_2], [outgoing_1, outgoing_2], [crossing_1, crossing_2])
        self.assertTrue(intersection_1 == intersection_2)

        intersection_2 = Intersection(5, [incoming_1, incoming_2], [outgoing_1, outgoing_2])
        self.assertFalse(intersection_1 == intersection_2)

        intersection_2 = Intersection(1, [incoming_1], [outgoing_1, outgoing_2])
        self.assertFalse(intersection_1 == intersection_2)

    def test_hash(self):
        incoming = IncomingGroup(3, {20, 21}, 29, {22, 23}, {24, 25}, {26, 27})
        outgoing = OutgoingGroup(29, {22, 23, 24, 25, 26, 27})
        intersection_1 = Intersection(1, [incoming], [outgoing])
        intersection_2 = Intersection(1, [incoming], [outgoing])
        self.assertEqual(hash(intersection_1), hash(intersection_2))

        incoming = IncomingGroup(3, {20, 21}, 29, {22, 23}, {24, 25}, {26, 28})
        intersection_2 = Intersection(1, [incoming], [outgoing])
        self.assertNotEqual(hash(intersection_1), hash(intersection_2))

    def test_find_incoming_by_id(self):

        #  Testing when the function returns an IncomingGroup
        incoming_1 = self._intersection_1.find_incoming_by_id(self._incoming_1.incoming_id)
        self.assertEqual(incoming_1.incoming_id, self._incoming_1.incoming_id)

        incoming_2 = self._intersection_1.find_incoming_by_id(self._incoming_2.incoming_id)
        self.assertEqual(incoming_2.incoming_id, self._incoming_2.incoming_id)

        #  Testing when the function returns None
        incoming_3 = self._intersection_1.find_incoming_by_id(10)
        self.assertIsNone(incoming_3)


class TestOutgoingGroup(unittest.TestCase):
    def test_initialization_OutgoingGroupElement(self):
        outgoing_id = 1
        outgoing_lanelets = {1, 2, 3}
        outgoing = OutgoingGroup(outgoing_id, outgoing_lanelets)
        self.assertEqual(outgoing.outgoing_id, outgoing_id)
        self.assertEqual(outgoing.outgoing_lanelets, outgoing_lanelets)

        outgoing = OutgoingGroup(outgoing_id, None)
        self.assertEqual(outgoing.outgoing_id, outgoing_id)
        self.assertEqual(outgoing.outgoing_lanelets, set())

    def test_equality(self):
        outgoing_group_1 = OutgoingGroup(1, {1, 2, 3})

        outgoing_group_2 = OutgoingGroup(1, {1, 2, 3})
        self.assertTrue(outgoing_group_1 == outgoing_group_2)

        outgoing_group_2 = OutgoingGroup(2, {1, 2, 3})
        self.assertFalse(outgoing_group_1 == outgoing_group_2)

        outgoing_group_2 = OutgoingGroup(2, {1, 2})
        self.assertFalse(outgoing_group_1 == outgoing_group_2)

        outgoing_group_2 = OutgoingGroup(2)
        self.assertFalse(outgoing_group_1 == outgoing_group_2)

    def test_hash(self):
        outgoing_group_1 = OutgoingGroup(1, {1, 2, 3})
        outgoing_group_2 = OutgoingGroup(1, {1, 2, 3})
        self.assertEqual(hash(outgoing_group_1), hash(outgoing_group_2))

        outgoing_group_2 = OutgoingGroup(1, {1, 2})
        self.assertNotEqual(hash(outgoing_group_1), hash(outgoing_group_2))


class TestIntersectionIncomingElement(unittest.TestCase):
    def test_initialization_IntersectionIncomingElement(self):
        incoming_id_1 = 2
        lanelets_1 = {10, 11}
        outgoing_right_1 = {12, 13}
        outgoing_straight_1 = {14, 15}
        outgoing_left_1 = {16, 17}
        incoming_1 = IncomingGroup(incoming_id_1, lanelets_1, 19, outgoing_right_1, outgoing_straight_1,
                                   outgoing_left_1)

        self.assertEqual(incoming_1.incoming_id, incoming_id_1)
        self.assertSetEqual(incoming_1.incoming_lanelets, lanelets_1)
        self.assertSetEqual(incoming_1.outgoing_right, outgoing_right_1)
        self.assertSetEqual(incoming_1.outgoing_straight, outgoing_straight_1)
        self.assertSetEqual(incoming_1.outgoing_left, outgoing_left_1)

        incoming_id_2 = '3'
        self.assertRaises(AssertionError, IncomingGroup, incoming_id_2)

    def test_equality(self):
        incoming_1 = IncomingGroup(2, {10, 11}, 19, {12, 13}, {14, 15}, {16, 17})
        incoming_2 = IncomingGroup(2, {10, 11}, 19, {12, 13}, {14, 15}, {16, 17})
        self.assertTrue(incoming_1 == incoming_2)

        incoming_2 = IncomingGroup(1, {10, 11}, 19, {12, 13}, {14, 15}, {16, 17})
        self.assertFalse(incoming_1 == incoming_2)

        incoming_2 = IncomingGroup(2, {11, 11}, 19, {12, 13}, {14, 15}, {16, 17})
        self.assertFalse(incoming_1 == incoming_2)

        incoming_2 = IncomingGroup(2, {10, 11}, 19, {12, 17}, {14, 15}, {16, 17})
        self.assertFalse(incoming_1 == incoming_2)

        incoming_2 = IncomingGroup(2, {10, 11}, 19, {12, 13}, {5, 15}, {16, 17})
        self.assertFalse(incoming_1 == incoming_2)

        incoming_2 = IncomingGroup(2, {10, 11}, 19, {12, 13}, {14, 15}, {7, 17})
        self.assertFalse(incoming_1 == incoming_2)

        incoming_1 = IncomingGroup(2, {10, 11}, 19, None, {14, 15}, {16, 17})
        incoming_2 = IncomingGroup(2, {10, 11}, 19, None, {14, 15}, {16, 17})
        self.assertTrue(incoming_1 == incoming_2)

        incoming_2 = IncomingGroup(2, {10, 11}, 19, {1, 2, 3, 4}, {14, 15}, {16, 17})
        self.assertFalse(incoming_1 == incoming_2)

        incoming_2 = IncomingGroup(2, {10, 11}, 19, {12, 13}, {14, 15}, {16, 17})
        self.assertFalse(incoming_1 == incoming_2)

    def test_hash(self):
        incoming_1 = IncomingGroup(2, {10, 11}, 19, {12, 13}, {14, 15}, {16, 17})
        incoming_2 = IncomingGroup(2, {10, 11}, 19, {12, 13}, {14, 15}, {16, 17})
        self.assertEqual(hash(incoming_1), hash(incoming_2))

        incoming_2 = IncomingGroup(2, {10, 11}, 19, {12, 17}, {14, 15}, {16, 17})
        self.assertNotEqual(hash(incoming_1), hash(incoming_2))


if __name__ == '__main__':
    unittest.main()
