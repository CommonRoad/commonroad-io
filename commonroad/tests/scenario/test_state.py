import unittest
import math
import numpy as np

from commonroad.scenario.trajectory import State
from commonroad.geometry.shape import Rectangle
from commonroad.common.util import Interval, AngleInterval


class TestState(unittest.TestCase):
    def test_constructor(self):

        def check_components():
            for element in all_state_elements:
                if element in state_elements.keys():
                    if isinstance(state_elements[element], np.ndarray):
                        np.testing.assert_array_equal(state.__getattribute__(element), state_elements[element])
                    else:
                        self.assertEqual(state.__getattribute__(element), state_elements[element])
                    self.assertTrue(hasattr(state, element))
                else:
                    self.assertFalse(hasattr(state, element))

        all_state_elements = [
            'position',
            'orientation',
            'velocity',
            'steering_angle',
            'yaw_rate',
            'slip_angle',
            'roll_angle',
            'roll_rate',
            'pitch_angle',
            'pitch_rate',
            'velocity_y',
            'position_z',
            'velocity_z',
            'roll_angle_front',
            'roll_rate_front',
            'velocity_y_front',
            'position_z_front',
            'velocity_z_front',
            'roll_angle_rear',
            'roll_rate_rear',
            'velocity_y_rear',
            'position_z_rear',
            'velocity_z_rear',
            'left_front_wheel_angular_speed',
            'right_front_wheel_angular_speed',
            'left_rear_wheel_angular_speed',
            'right_rear_wheel_angular_speed',
            'delta_y_f',
            'delta_y_r',
            'acceleration',
            'time_step']

        state_elements = {'position': np.array([1.0, -3.456]), 'orientation': 0.87, 'time_step': 0}
        state = State(**state_elements)
        check_components()

        state_elements = {'yaw_rate': 0.08, 'velocity': 15.6, 'time_step': 5, 'pitch_rate': 0.001, 'position_z': 1.3}
        state = State(**state_elements)
        check_components()

        state_elements = {'pitchAngle': 0.1}
        self.assertRaises(AttributeError, State, **state_elements)

    def test_translate_rotate(self):

        def translate_rotate(point):
            b = np.array(point) + np.array(translation)
            return [b[0] * math.cos(angle) - b[1] * math.sin(angle),
                    b[0] * math.sin(angle) + b[1] * math.cos(angle)]

        # test assertions
        state_elements = {'position': Interval(-3.456, 1.0), 'orientation': 0.87, 'time_step': 0}
        state = State(**state_elements)
        self.assertRaises(TypeError, state.translate_rotate, np.array([-1.0, -1.0]), 0.0)

        state_elements = {'position': np.array([1.35, -2.4]), 'orientation': 0.87, 'time_step': 0}
        state = State(**state_elements)
        self.assertRaises(AssertionError, state.translate_rotate, np.array([-.2, 1.4, 3.3]), 0.0)
        self.assertRaises(AssertionError, state.translate_rotate, np.array([-.2, 1.4]), [0.0, -.7])
        self.assertRaises(AssertionError, state.translate_rotate, np.array([0.0, 0.0]), -7.10)

        # exact position and orientation
        state_elements = {'position': np.array([1.35, -2.4]), 'orientation': 0.87, 'time_step': 0}
        state = State(**state_elements)
        translation = np.array([-1.3, 0.87])
        angle = 0.13
        transformed_state = translate_rotate(state_elements['position'])
        state_prime = state.translate_rotate(translation, angle)
        self.assertEqual(len(state_prime.position),
                         len(transformed_state))
        for ii in range(0, len(transformed_state)):
            self.assertAlmostEqual(transformed_state[ii], state_prime.position[ii])
        self.assertAlmostEqual(state_prime.orientation, state_elements['orientation'] + angle)

        # uncertain orientation
        state_elements = {'orientation': AngleInterval(-0.45, 0.334)}
        angle = 0.87
        state = State(**state_elements)
        state_prime = state.translate_rotate(translation, angle)
        self.assertAlmostEqual(state_prime.orientation.start, 0.42)
        self.assertAlmostEqual(state_prime.orientation.end, 1.204)

        # uncertain orientation large rotation
        state_elements = {'orientation': AngleInterval(-0.02, 3.01)}
        angle = 1.77 * np.pi
        state = State(**state_elements)
        state_prime = state.translate_rotate(translation, angle)
        self.assertAlmostEqual(state_prime.orientation.end - state_prime.orientation.start, 3.01+0.02)

        # uncertain position
        state_elements = {'position': Rectangle(length=4.0, width=2.0, center=np.array([1.5, 3.6]), orientation=0.13)}
        angle = 0.87
        state = State(**state_elements)
        state_prime = state.translate_rotate(translation, angle)

        new_center = translate_rotate(state_elements['position'].center)
        self.assertEqual(len(state_prime.position.center),
                         len(new_center))
        for ii in range(0, len(new_center)):
            self.assertAlmostEqual(new_center[ii], state_prime.position.center[ii])
        self.assertAlmostEqual(state_prime.position.orientation, 1.0)
        self.assertAlmostEqual(state_prime.position.length, 4.0)
        self.assertAlmostEqual(state_prime.position.width, 2.0)


if __name__ == '__main__':
    unittest.main()
