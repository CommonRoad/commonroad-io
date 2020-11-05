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

        res = self.param_server.by_callstack(tuple(), ('time_begin'))
        self.assertEqual(0, res)

        res = self.param_server.by_callstack(tuple(),
                                             ('lanelet_network', 'time_begin'))
        self.assertEqual(0, res)

    def test_set_item(self):
        self.param_server['test1', 'test2', 'test3'] = 1
        self.assertEqual(1, self.param_server['test1', 'test2', 'test3'])
        self.param_server['test1', 'test21'] = 2
        with self.assertRaises(KeyError):
            self.param_server['test1', 'test2', 'test3', 'test4'] = 3
        with self.assertRaises(KeyError):
            self.param_server['test1', 'test21', 'test3', 'test4'] = 4
        self.assertEqual(1, self.param_server['test1', 'test2', 'test3'])
