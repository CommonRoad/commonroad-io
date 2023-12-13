import unittest

from commonroad.common.common_lanelet import LineMarking
from commonroad.visualization.util import line_marking_to_linestyle


class TestVisUtils(unittest.TestCase):
    def test_line_marking_to_linestyle(self):
        self.assertEqual(
            line_marking_to_linestyle(LineMarking.DASHED),
            (
                "--",
                (10, 10),
                0.25,
            ),
        )
        self.assertEqual(line_marking_to_linestyle(LineMarking.SOLID), ("-", (None, None), 0.25))
        self.assertEqual(line_marking_to_linestyle(LineMarking.BROAD_DASHED), ("--", (10, 10), 0.5))
        self.assertEqual(line_marking_to_linestyle(LineMarking.BROAD_SOLID), ("-", (None, None), 0.5))
        self.assertEqual(line_marking_to_linestyle(LineMarking.CURB), ("-", (None, None), 1))
        self.assertEqual(line_marking_to_linestyle(LineMarking.LOWERED_CURB), ("-", (None, None), 0.75))
        self.assertEqual(line_marking_to_linestyle(LineMarking.DASHED_DASHED), ("-", (10, 10), 1.0))
        self.assertEqual(line_marking_to_linestyle(LineMarking.SOLID_SOLID), ("-", (None, None), 1.0))
