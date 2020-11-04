import unittest
from commonroad.visualization.param_server import ParamServer


class TestParamServer(unittest.TestCase):

    def setUp(self) -> None:
        self.param_server = ParamServer()

    def test_callstack(self):
        res = self.param_server.by_callstack(
                ('scenario', 'lanelet_network', 'lanelet'),
                'center_bound_color')
        self.assertEqual('#dddddd', res)
