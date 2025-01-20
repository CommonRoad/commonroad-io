import unittest

import numpy as np

from commonroad.common.util import Interval
from commonroad.scenario.state import InitialState, KSState
from commonroad.scenario.trajectory import Trajectory


class TestTrajectory(unittest.TestCase):
    def test_constructor(self):
        self.assertRaises(AssertionError, Trajectory, 0, list())

        states = list()
        states.append(KSState(position=np.array([1.35, -2.4]), orientation=0.87, time_step=0))
        states.append(KSState(position=np.array([2.35, -2.4]), orientation=0.97, time_step=1))
        states.append(KSState(position=np.array([3.35, -2.4]), orientation=1.07, time_step=2))
        states.append(KSState(position=np.array([4.35, -2.4]), orientation=1.17, time_step=3))
        states.append(KSState(position=np.array([5.35, -2.4]), orientation=1.27, time_step=4))
        states.append(KSState(position=np.array([6.35, -2.4]), orientation=1.37, time_step=5))
        states.append(KSState(position=np.array([7.35, -2.4]), orientation=1.47, time_step=6))
        states.append(KSState(position=np.array([8.35, -2.4]), orientation=1.57, time_step=7))
        states.append(KSState(position=np.array([9.35, -2.4]), orientation=1.67, time_step=8))

        trajectory = Trajectory(0, states)
        self.assertEqual(len(trajectory.state_list), len(states))
        self.assertEqual(trajectory.state_list, states)

        self.assertRaises(AssertionError, Trajectory, 0.67, states)

        states = list()
        states.append(KSState(position=np.array([1.35, -2.4]), orientation=0.87, time_step=0))
        states.append(KSState(position=np.array([2.35, -2.4]), orientation=0.97, time_step=1))
        states.append(KSState(position=np.array([3.35, -2.4]), orientation=1.07, time_step=2))
        states.append(KSState(position=np.array([4.35, -2.4]), orientation=1.17, time_step=3))
        states.append(KSState(position=np.array([5.35, -2.4]), orientation=1.27, time_step=4))
        states.append(KSState(position=np.array([6.35, -2.4]), orientation=1.37, time_step=5))
        states.append(KSState(position=np.array([7.35, -2.4]), orientation=1.47, time_step=6))
        states.append(KSState(position=np.array([8.35, -2.4]), orientation=1.57, time_step=7))
        states.append(KSState(position=np.array([9.35, -2.4]), orientation=1.67, time_step=8))

        trajectory = Trajectory(0, states)
        self.assertEqual(len(trajectory.state_list), len(states))
        self.assertEqual(trajectory.state_list, states)

        states[-1].time_step = Interval(8, 10)
        self.assertRaises(AssertionError, Trajectory, 0, states)

        states.append(Interval(0, 1))
        self.assertRaises(AssertionError, Trajectory, 0, states)

        states = list()
        states.append(KSState(position=np.array([1.35, -2.4]), orientation=0.87, time_step=0))
        states.append(KSState(position=np.array([2.35, -2.4]), velocity=4.97, time_step=1))
        states.append(KSState(position=np.array([3.35, -2.4]), orientation=1.07, time_step=2))

        self.assertRaises(AssertionError, Trajectory, 0, states)

    def test_translate_rotate(self):
        states = list()
        states.append(KSState(position=np.array([1.35, -2.4]), orientation=0.87, time_step=0))
        states.append(KSState(position=np.array([2.35, -2.4]), orientation=0.97, time_step=1))
        states.append(KSState(position=np.array([3.35, -2.4]), orientation=1.07, time_step=2))
        states.append(KSState(position=np.array([4.35, -2.4]), orientation=1.17, time_step=3))
        states.append(KSState(position=np.array([5.35, -2.4]), orientation=1.27, time_step=4))

        translation = np.array([2.0, 0.0])
        angle = 0.0
        states_new = list()
        states_new.append(KSState(position=np.array([3.35, -2.4]), orientation=0.87, time_step=0))
        states_new.append(KSState(position=np.array([4.35, -2.4]), orientation=0.97, time_step=1))
        states_new.append(KSState(position=np.array([5.35, -2.4]), orientation=1.07, time_step=2))
        states_new.append(KSState(position=np.array([6.35, -2.4]), orientation=1.17, time_step=3))
        states_new.append(KSState(position=np.array([7.35, -2.4]), orientation=1.27, time_step=4))

        trajectory = Trajectory(0, states)

        trajectory.translate_rotate(translation, angle)

        for i, state in enumerate(trajectory.state_list):
            self.assertAlmostEqual(state.position[0], states_new[i].position[0])
            self.assertAlmostEqual(state.position[1], states_new[i].position[1])

    def test_state_at_time_step(self):
        states = list()
        states.append(KSState(position=np.array([1.35, -2.4]), orientation=0.87, time_step=0))
        states.append(KSState(position=np.array([2.35, -2.4]), orientation=0.97, time_step=1))
        states.append(KSState(position=np.array([3.35, -2.4]), orientation=1.07, time_step=2))
        states.append(KSState(position=np.array([4.35, -2.4]), orientation=1.17, time_step=3))
        states.append(KSState(position=np.array([5.35, -2.4]), orientation=1.27, time_step=4))
        trajectory = Trajectory(0, states)
        self.assertEqual(trajectory.state_at_time_step(2), states[2])

        states = list()
        states.append(KSState(position=np.array([1.35, -2.4]), orientation=0.87, time_step=6))
        states.append(KSState(position=np.array([2.35, -2.4]), orientation=0.97, time_step=7))
        states.append(KSState(position=np.array([3.35, -2.4]), orientation=1.07, time_step=8))
        states.append(KSState(position=np.array([4.35, -2.4]), orientation=1.17, time_step=9))
        states.append(KSState(position=np.array([5.35, -2.4]), orientation=1.27, time_step=10))
        trajectory = Trajectory(6, states)
        self.assertEqual(trajectory.state_at_time_step(2), None)
        self.assertEqual(trajectory.state_at_time_step(9), states[3])
        self.assertEqual(trajectory.state_at_time_step(10), states[4])

    def test_interpolate_states(self):
        states = list()
        states.append(KSState(time_step=5, position=np.array([0, 0]), velocity=0, orientation=10))
        states.append(KSState(time_step=5, position=np.array([5, 5]), velocity=5, orientation=15))
        states.append(
            KSState(time_step=5, position=np.array([10, 10]), velocity=10, orientation=20)
        )
        time = np.array([0, 5, 10])

        traj_new = Trajectory.resample_continuous_time_state_list(states, time, 2.5, 4)
        x_t = np.interp(np.arange(0, 2.5 * 4 + 2.5, 2.5), time, [0, 5, 10])
        for i, x in enumerate(traj_new.state_list):
            self.assertEqual(x.velocity, x_t[i])
            self.assertEqual(x.time_step, i)
            self.assertEqual(x.position[0], x_t[i])
            self.assertEqual(x.position[1], x_t[i])
            self.assertEqual(x.orientation, 10 + x_t[i])

        traj_new = Trajectory.resample_continuous_time_state_list(
            states, time, 2.5, 2, initial_time_cont=5
        )
        x_t = np.interp(np.arange(5, 2.5 * 4 + 2.5, 2.5), time, [0, 5, 10])
        for i, x in enumerate(traj_new.state_list):
            self.assertEqual(x.velocity, x_t[i])
            self.assertEqual(x.time_step, i)
            self.assertEqual(x.position[0], x_t[i])
            self.assertEqual(x.position[1], x_t[i])
            self.assertEqual(x.orientation, 10 + x_t[i])

    def test_check_state_list(self):
        self.assertRaises(AssertionError, Trajectory, 0, 1)
        self.assertRaises(AssertionError, Trajectory, 0, [])
        self.assertRaises(AssertionError, Trajectory, 0, [0, 1])
        self.assertRaises(
            AssertionError,
            Trajectory,
            0,
            [KSState(time_step="a", position=np.array([0, 0]), velocity=0, orientation=10)],
        )
        self.assertRaises(
            AssertionError,
            Trajectory,
            0,
            [
                KSState(time_step=0, position=np.array([0, 0]), velocity=0, orientation=10),
                KSState(time_step=0, position=np.array([0, 0])),
            ],
        )
        self.assertRaises(
            AssertionError,
            Trajectory,
            2,
            [
                KSState(time_step=0, position=np.array([0, 0]), velocity=0, orientation=10),
                KSState(time_step=0, position=np.array([0, 0])),
            ],
        )

    def test_append_state(self):
        states = list()
        states.append(KSState(position=np.array([1.35, -2.4]), orientation=0.87, time_step=5))
        trajectory = Trajectory(5, states)
        self.assertRaises(AssertionError, trajectory.append_state, None)
        self.assertRaises(
            AssertionError,
            trajectory.append_state,
            InitialState(
                time_step=6,
                position=np.array([1.0, -2.0]),
                orientation=0.5,
                velocity=3.3,
                acceleration=1.3,
            ),
        )
        self.assertRaises(
            AssertionError,
            trajectory.append_state,
            KSState(position=np.array([1.0, -2.0]), orientation=0.8, time_step=0),
        )

        new_state = KSState(position=np.array([2.0, -3.0]), orientation=0.9, time_step=6)
        trajectory.append_state(new_state)
        self.assertEqual(trajectory.state_at_time_step(6), new_state)


if __name__ == "__main__":
    unittest.main()
