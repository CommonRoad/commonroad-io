import unittest
import numpy as np
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.common.util import Interval


class TestTrajectory(unittest.TestCase):
    def test_constructor(self):

        self.assertRaises(AssertionError, Trajectory, 0, list())

        states = list()
        states.append(State(**{'position': np.array([1.35, -2.4]), 'orientation': 0.87, 'velocity': 0}))
        states.append(State(**{'position': np.array([2.35, -2.4]), 'orientation': 0.97, 'velocity': 1}))
        states.append(State(**{'position': np.array([3.35, -2.4]), 'orientation': 1.07, 'velocity': 2}))
        states.append(State(**{'position': np.array([4.35, -2.4]), 'orientation': 1.17, 'velocity': 3}))
        states.append(State(**{'position': np.array([5.35, -2.4]), 'orientation': 1.27, 'velocity': 4}))
        states.append(State(**{'position': np.array([6.35, -2.4]), 'orientation': 1.37, 'velocity': 5}))
        states.append(State(**{'position': np.array([7.35, -2.4]), 'orientation': 1.47, 'velocity': 6}))
        states.append(State(**{'position': np.array([8.35, -2.4]), 'orientation': 1.57, 'velocity': 7}))
        states.append(State(**{'position': np.array([9.35, -2.4]), 'orientation': 1.67, 'velocity': 8}))

        trajectory = Trajectory(0, states)
        self.assertEqual(len(trajectory.state_list), len(states))
        self.assertEqual(trajectory.state_list, states)

        self.assertRaises(AssertionError, Trajectory, 0.67, states)

        states = list()
        states.append(State(**{'position': np.array([1.35, -2.4]), 'orientation': 0.87, 'time_step': 0}))
        states.append(State(**{'position': np.array([2.35, -2.4]), 'orientation': 0.97, 'time_step': 1}))
        states.append(State(**{'position': np.array([3.35, -2.4]), 'orientation': 1.07, 'time_step': 2}))
        states.append(State(**{'position': np.array([4.35, -2.4]), 'orientation': 1.17, 'time_step': 3}))
        states.append(State(**{'position': np.array([5.35, -2.4]), 'orientation': 1.27, 'time_step': 4}))
        states.append(State(**{'position': np.array([6.35, -2.4]), 'orientation': 1.37, 'time_step': 5}))
        states.append(State(**{'position': np.array([7.35, -2.4]), 'orientation': 1.47, 'time_step': 6}))
        states.append(State(**{'position': np.array([8.35, -2.4]), 'orientation': 1.57, 'time_step': 7}))
        states.append(State(**{'position': np.array([9.35, -2.4]), 'orientation': 1.67, 'time_step': 8}))

        trajectory = Trajectory(0, states)
        self.assertEqual(len(trajectory.state_list), len(states))
        self.assertEqual(trajectory.state_list, states)

        states[-1].time_step = Interval(8, 10)
        self.assertRaises(AssertionError, Trajectory, 0, states)

        states.append(Interval(0, 1))
        self.assertRaises(AssertionError, Trajectory, 0, states)

        states = list()
        states.append(State(**{'position': np.array([1.35, -2.4]), 'orientation': 0.87, 'time_step': 0}))
        states.append(State(**{'position': np.array([2.35, -2.4]), 'velocity': 4.97, 'time_step': 1}))
        states.append(State(**{'position': np.array([3.35, -2.4]), 'orientation': 1.07, 'time_step': 2}))

        self.assertRaises(AssertionError, Trajectory, 0, states)

        states = list()
        states.append(State(**{'position': np.array([1.35, -2.4]), 'orientation': 0.87, 'time_step': 6}))
        states.append(State(**{'position': np.array([2.35, -2.4]), 'orientation': 0.97, 'time_step': 7}))
        states.append(State(**{'position': np.array([3.35, -2.4]), 'orientation': 1.07, 'time_step': 8}))
        states.append(State(**{'position': np.array([4.35, -2.4]), 'orientation': 1.17, 'time_step': 9}))
        states.append(State(**{'position': np.array([5.35, -2.4]), 'orientation': 1.27, 'time_step': 10}))
        self.assertRaises(AssertionError, Trajectory, 0, states)

    def test_translate_rotate(self):
        states = list()
        states.append(State(**{'position': np.array([1.35, -2.4]), 'orientation': 0.87, 'time_step': 0}))
        states.append(State(**{'position': np.array([2.35, -2.4]), 'orientation': 0.97, 'time_step': 1}))
        states.append(State(**{'position': np.array([3.35, -2.4]), 'orientation': 1.07, 'time_step': 2}))
        states.append(State(**{'position': np.array([4.35, -2.4]), 'orientation': 1.17, 'time_step': 3}))
        states.append(State(**{'position': np.array([5.35, -2.4]), 'orientation': 1.27, 'time_step': 4}))

        translation = np.array([2.0, 0.0])
        angle = 0.0
        states_new = list()
        states_new.append(State(**{'position': np.array([3.35, -2.4]), 'orientation': 0.87, 'time_step': 0}))
        states_new.append(State(**{'position': np.array([4.35, -2.4]), 'orientation': 0.97, 'time_step': 1}))
        states_new.append(State(**{'position': np.array([5.35, -2.4]), 'orientation': 1.07, 'time_step': 2}))
        states_new.append(State(**{'position': np.array([6.35, -2.4]), 'orientation': 1.17, 'time_step': 3}))
        states_new.append(State(**{'position': np.array([7.35, -2.4]), 'orientation': 1.27, 'time_step': 4}))

        trajectory = Trajectory(0, states)

        trajectory.translate_rotate(translation, angle)

        for i, state in enumerate(trajectory.state_list):
            self.assertAlmostEqual(state.position[0], states_new[i].position[0])
            self.assertAlmostEqual(state.position[1], states_new[i].position[1])

    def test_state_at_time_step(self):
        states = list()
        states.append(State(**{'position': np.array([1.35, -2.4]), 'orientation': 0.87, 'time_step': 0}))
        states.append(State(**{'position': np.array([2.35, -2.4]), 'orientation': 0.97, 'time_step': 1}))
        states.append(State(**{'position': np.array([3.35, -2.4]), 'orientation': 1.07, 'time_step': 2}))
        states.append(State(**{'position': np.array([4.35, -2.4]), 'orientation': 1.17, 'time_step': 3}))
        states.append(State(**{'position': np.array([5.35, -2.4]), 'orientation': 1.27, 'time_step': 4}))
        trajectory = Trajectory(0, states)
        self.assertEqual(trajectory.state_at_time_step(2), states[2])

        states = list()
        states.append(State(**{'position': np.array([1.35, -2.4]), 'orientation': 0.87, 'time_step': 6}))
        states.append(State(**{'position': np.array([2.35, -2.4]), 'orientation': 0.97, 'time_step': 7}))
        states.append(State(**{'position': np.array([3.35, -2.4]), 'orientation': 1.07, 'time_step': 8}))
        states.append(State(**{'position': np.array([4.35, -2.4]), 'orientation': 1.17, 'time_step': 9}))
        states.append(State(**{'position': np.array([5.35, -2.4]), 'orientation': 1.27, 'time_step': 10}))
        trajectory = Trajectory(6, states)
        self.assertEqual(trajectory.state_at_time_step(2), None)
        self.assertEqual(trajectory.state_at_time_step(9), states[3])
        self.assertEqual(trajectory.state_at_time_step(10), states[4])


if __name__ == '__main__':
    unittest.main()