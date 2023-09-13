import unittest

from commonroad.scenario.state import *
from commonroad.geometry.shape import Rectangle
from commonroad.common.util import Interval, AngleInterval


class TestState(unittest.TestCase):

    def test_attributes(self):
        attrs = ['time_step', 'position', 'velocity', 'velocity_y']
        state = PMState()
        self.assertEqual(state.attributes, attrs)

    def test_used_attributes(self):
        used_attrs = ['position', 'velocity_y']
        state = PMState(position=np.array([0., 0.]), velocity_y=10.)
        self.assertEqual(state.used_attributes, used_attrs)

    def test_is_uncertain_position(self):
        state = PMState(position=Rectangle(10., 10.))
        self.assertTrue(state.is_uncertain_position)

        state = PMState(position=np.array([5., 5.]))
        self.assertFalse(state.is_uncertain_position)

    def test_is_uncertain_orientation(self):
        state = STState(orientation=AngleInterval(0., 0.9))
        self.assertTrue(state.is_uncertain_orientation)

        state = STState(orientation=0.3)
        self.assertFalse(state.is_uncertain_orientation)

    def test_has_value(self):
        state = PMState(velocity=30.)
        self.assertTrue(state.has_value("velocity"))
        self.assertFalse(state.has_value("velocity_y"))
        self.assertFalse(state.has_value("steering_angle"))

    def test_translate_rotate(self):

        def translate_rotate(point):
            b = np.array(point) + np.array(translation)
            return [b[0] * math.cos(angle) - b[1] * math.sin(angle),
                    b[0] * math.sin(angle) + b[1] * math.cos(angle)]

        # test assertions
        state_elements = {'position': Interval(-3.456, 1.0), 'orientation': 0.87, 'time_step': 0}
        state = InitialState(**state_elements)
        self.assertRaises(TypeError, state.translate_rotate, np.array([-1.0, -1.0]), 0.0)

        state_elements = {'position': np.array([1.35, -2.4]), 'orientation': 0.87, 'time_step': 0}
        state = InitialState(**state_elements)
        self.assertRaises(AssertionError, state.translate_rotate, np.array([-.2, 1.4, 3.3]), 0.0)
        self.assertRaises(AssertionError, state.translate_rotate, np.array([-.2, 1.4]), [0.0, -.7])
        self.assertRaises(AssertionError, state.translate_rotate, np.array([0.0, 0.0]), -7.10)

        # exact position and orientation
        state_elements = {'position': np.array([1.35, -2.4]), 'orientation': 0.87, 'time_step': 0}
        state = InitialState(**state_elements)
        translation = np.array([-1.3, 0.87])
        angle = 0.13
        transformed_state = translate_rotate(state_elements['position'])
        state_prime = state.translate_rotate(translation, angle)
        self.assertEqual(len(state_prime.position), len(transformed_state))
        for ii in range(0, len(transformed_state)):
            self.assertAlmostEqual(transformed_state[ii], state_prime.position[ii])
        self.assertAlmostEqual(state_prime.orientation, state_elements['orientation'] + angle)

        # uncertain orientation
        state_elements = {'orientation': AngleInterval(-0.45, 0.334)}
        angle = 0.87
        state = InitialState(**state_elements)
        state_prime = state.translate_rotate(translation, angle)
        self.assertAlmostEqual(state_prime.orientation.start, 0.42)
        self.assertAlmostEqual(state_prime.orientation.end, 1.204)

        # uncertain orientation large rotation
        state_elements = {'orientation': AngleInterval(-0.02, 3.01)}
        angle = 1.77 * np.pi
        state = InitialState(**state_elements)
        state_prime = state.translate_rotate(translation, angle)
        self.assertAlmostEqual(state_prime.orientation.end - state_prime.orientation.start, 3.01+0.02)

        # uncertain position
        state_elements = {'position': Rectangle(length=4.0, width=2.0, center=np.array([1.5, 3.6]), orientation=0.13)}
        angle = 0.87
        state = InitialState(**state_elements)
        state_prime = state.translate_rotate(translation, angle)

        new_center = translate_rotate(state_elements['position'].center)
        self.assertEqual(len(state_prime.position.center), len(new_center))
        for ii in range(0, len(new_center)):
            self.assertAlmostEqual(new_center[ii], state_prime.position.center[ii])
        self.assertAlmostEqual(state_prime.position.orientation, 1.0)
        self.assertAlmostEqual(state_prime.position.length, 4.0)
        self.assertAlmostEqual(state_prime.position.width, 2.0)

    def test_convert_state_to_state(self):
        initial_state = InitialState(position=Rectangle(20., 5.), orientation=0.1, velocity=10.,
                                     acceleration=5., yaw_rate=4., slip_angle=6.)
        pm_state = initial_state.convert_state_to_state(PMState())
        self.assertEqual(initial_state.time_step, pm_state.time_step)
        self.assertEqual(initial_state.position, pm_state.position)
        self.assertEqual(initial_state.velocity, pm_state.velocity)
        self.assertEqual(pm_state.velocity_y, None)

    def test_fill_with_defaults(self):
        initial_state = InitialState()
        initial_state.fill_with_defaults()
        for field in initial_state.attributes:
            value = getattr(initial_state, field)
            if field == 'position':
                self.assertEqual(np.array([0., 0.]).tolist(), value.tolist())
            else:
                self.assertEqual(0., value)

    def test_equals(self):
        ks_state_1 = STState(time_step=0, position=np.array([0., 0.]), steering_angle=0.1,
                             velocity=10., orientation=AngleInterval(0., 0.2))
        ks_state_2 = STState(time_step=0, position=np.array([0., 0.]), steering_angle=0.1,
                             velocity=10., orientation=AngleInterval(0., 0.2))
        self.assertEqual(ks_state_1, ks_state_2)

        ks_state_2.position = Rectangle(10., 4.)
        self.assertNotEqual(ks_state_1, ks_state_2)

        pm_state_1 = PMState(time_step=1)
        pm_state_2 = PMState(time_step=1)
        self.assertEqual(pm_state_1, pm_state_2)

        pm_state_2.velocity = Interval(5., 20.)
        self.assertNotEqual(pm_state_1, pm_state_2)

    def test_hash(self):
        ks_state_1 = STState(time_step=0, position=np.array([0., 0.]), steering_angle=0.1,
                             velocity=10., orientation=AngleInterval(0., 0.2))
        ks_state_2 = STState(time_step=0, position=np.array([0., 0.]), steering_angle=0.1,
                             velocity=10., orientation=AngleInterval(0., 0.2))
        self.assertEqual(hash(ks_state_1), hash(ks_state_2))

        states_1 = set()
        states_2 = set()
        for i in range(10):
            pm_state = PMState(time_step=i, position=np.array([i, i]), velocity=i * 0.01, velocity_y=i * 0.02)
            states_1.add(pm_state)
            states_2.add(copy.copy(pm_state))
        self.assertEqual(states_1, states_2)

        for state in states_2:
            state.velocity_y = 0.01
        self.assertNotEqual(states_1, states_2)

    def test_custom_state(self):
        custom_state_1 = CustomState(time_step=Interval(0, 1))
        setattr(custom_state_1, "velocity", Interval(0.0, 1))
        self.assertEqual(custom_state_1.velocity.start, 0)
        self.assertEqual(custom_state_1.velocity.end, 1)
        self.assertEqual(custom_state_1.time_step.start, 0)
        self.assertEqual(custom_state_1.time_step.end, 1)

        custom_state_2 = CustomState(time_step=Interval(0, 1), velocity=Interval(0.0, 1))
        self.assertEqual(custom_state_2.velocity.start, 0)
        self.assertEqual(custom_state_2.velocity.end, 1)
        self.assertEqual(custom_state_2.time_step.start, 0)
        self.assertEqual(custom_state_2.time_step.end, 1)

    def test_extra_state_properties(self):
        state = PMState(time_step=Interval(0, 1), velocity=0.0, velocity_y=0.0, position=np.array([0, 0]))
        self.assertEqual(state.orientation, 0.0)
        state = PMState(time_step=Interval(0, 1), velocity=0.0, velocity_y=1.0, position=np.array([0, 0]))
        self.assertAlmostEqual(state.orientation, 1.57079, 3)

        state = ExtendedPMState(time_step=Interval(0, 1), velocity=0.0, acceleration=1.0,
                                position=np.array([0, 0]), orientation=0.0)
        self.assertEqual(state.velocity_y, 0.0)


if __name__ == '__main__':
    unittest.main()
