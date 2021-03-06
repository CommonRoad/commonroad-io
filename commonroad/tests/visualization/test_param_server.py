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

    def test_hierarchy(self):
        path = ('history', 'occupancy', 'shape', 'rectangle', 'edgecolor')
        default_color = self.param_server[('dynamic_obstacle',) + path]

        self.param_server[path] = '#ffffff'

        fetched = self.param_server[('dynamic_obstacle',) + path]

        self.assertEqual(default_color, fetched)

    def test_chaining(self):
        param_a = ParamServer(params={"param_a": "a", "param_c": "a"})
        param_b = ParamServer(params={"param_b": "b", "param_a": "b"}, default=param_a)

        self.assertEqual(param_b["param_b"], "b")
        self.assertEqual(param_b["param_a"], "b")
        self.assertEqual(param_b["param_c"], "a")
        self.assertEqual(param_b["time_begin"], 0)


if __name__ == '__main__':
    unittest.main()
