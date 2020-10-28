import copy
from typing import List, Union
import numpy as np

import commonroad.geometry.transform
from commonroad.common.util import AngleInterval
from commonroad.common.validity import (
    is_valid_orientation,
    is_real_number_vector,
    is_real_number,
    ValidTypes,
    is_natural_number,
)
from commonroad.geometry.shape import Shape
from commonroad.common.util import make_valid_orientation

__author__ = "Stefanie Manzinger"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "2020.3"
__maintainer__ = "Stefanie Manzinger"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


class State:
    """ A state can be either exact or uncertain. Uncertain state elements can be either of type
        :class:`commonroad.common.util.Interval` or of type :class:`commonroad.geometry.shape.Shape`. A
        state is composed of several elements which are determined during runtime. The possible state elements
        are defined as slots, which comprise the necessary state elements to describe the states of all CommonRoad
        vehicle models:

        :ivar position: :math:`s_x`- and :math:`s_y`-position in a global coordinate system. Exact positions
            are given as numpy array [x, y], uncertain positions are given as :class:`commonroad.geometry.shape.Shape`
        :ivar orientation: yaw angle :math:`\Psi`. Exact values are given as real number, uncertain values are given as
            :class:`commonroad.common.util.AngleInterval`
        :ivar velocity: velocity :math:`v_x` in longitudinal direction in the vehicle-fixed coordinate system. Exact
            values are given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar steering_angle: steering angle :math:`\delta` of front wheels. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar steering_angle_speed: steering angle speed :math:`\dot{\delta}` of front wheels. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar yaw_rate: yaw rate :math:`\dot{\Psi}`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar slip_angle: slip angle :math:`\\beta`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar roll_angle: roll angle :math:`\Phi_S`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar roll_rate: roll rate :math:`\dot{\Phi}_S`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar pitch_angle: pitch angle :math:`\Theta_S`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar pitch_rate: pitch rate :math:`\dot{\Theta}_S`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar velocity_y: velocity :math:`v_y` in lateral direction in the vehicle-fixed coordinate system. Exact
            values are given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar position_z: position :math:`s_z` (height) from ground. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar velocity_z: velocity :math:`v_z` in vertical direction perpendicular to road plane. Exact values are
            given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar roll_angle_front: roll angle front :math:`\Phi_{UF}`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar roll_rate_front: roll rate front :math:`\dot{\Phi}_{UF}`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar velocity_y_front: velocity :math:`v_{y,UF}` in y-direction front. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar position_z_front: position :math:`s_{z,UF}` in z-direction front. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar velocity_z_front: velocity :math:`v_{z,UF}` in z-direction front. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar roll_angle_rear: roll angle rear :math:`\Phi_{UR}`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar roll_rate_rear: roll rate rear :math:`\dot{\Phi}_{UR}`. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar velocity_y_rear: velocity :math:`v_{y,UR}` in y-direction rear. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar position_z_rear: position :math:`s_{z,UR}` in z-direction rear. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar velocity_z_rear: velocity :math:`v_{z,UR}` in z-direction rear. Exact values are given as real number,
            uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar left_front_wheel_angular_speed: left front wheel angular speed :math:`\omega_{LF}`. Exact values
            are given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar right_front_wheel_angular_speed: right front wheel angular speed :math:`\omega_{RF}`. Exact values
            are given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar left_rear_wheel_angular_speed: left rear wheel angular speed :math:`\omega_{LR}`. Exact values
            are given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar right_rear_wheel_angular_speed: right rear wheel angular speed :math:`\omega_{RR}`. Exact values
            are given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar delta_y_f: front lateral displacement :math:`\delta_{y,f}` of sprung mass due to roll. Exact values
            are given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar delta_y_r: rear lateral displacement :math:`\delta_{y,r}` of sprung mass due to roll. Exact values
            are given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar acceleration: acceleration :math:`a_x`. We optionally include acceleration as a state variable for
            obstacles to provide additional information, e.g., for motion prediction, even though acceleration is often
            used as an input for vehicle models. Exact values are given as real number, uncertain values are given as
            :class:`commonroad.common.util.Interval`
        :ivar acceleration_y: velocity :math:`a_y`.
            We optionally include acceleration as a state variable for obstacles to provide additional information,
            e.g., for motion prediction, even though acceleration is often used as an input for vehicle models. Exact
            values are given as real number, uncertain values are given as :class:`commonroad.common.util.Interval`
        :ivar jerk: acceleration :math:`j`. We optionally include jerk as a state variable for
            obstacles to provide additional information, e.g., for motion prediction, even though jerk is often
            used as an input for vehicle models. Exact values are given as real number, uncertain values are given as
            :class:`commonroad.common.util.Interval`
        :ivar time_step: the discrete time step. Exact values are given as integers, uncertain values are given as
            :class:`commonroad.common.util.Interval`

        :Example:

        >>> import numpy as np
        >>> from commonroad.scenario.trajectory import State
        >>> from commonroad.common.util import Interval
        >>> # a state with position [2.0, 3.0] m and uncertain velocity from 5.4 to 7.0 m/s
        >>> # can be created as follows:
        >>> state = State(position=np.array([2.0, 3.0]), velocity=Interval(5.4, 7.0))
    """

    __slots__ = [
        'position',
        'orientation',
        'velocity',
        'steering_angle',
        'steering_angle_speed',
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
        'acceleration_y',
        'jerk',
        'time_step',
    ]

    def __init__(self, **kwargs):
        """ Elements of state vector are determined during runtime."""
        for (field, value) in kwargs.items():
            setattr(self, field, value)

    def translate_rotate(self, translation: np.ndarray, angle: float) -> 'State':
        """ First translates the state, and then rotates the state around the origin.

            :param translation: translation vector [x_off, y_off] in x- and y-direction
            :param angle: rotation angle in radian (counter-clockwise)
            :return: transformed state
        """
        assert is_real_number_vector(translation, 2), (
            '<State/translate_rotate>: argument translation is not '
            'a vector of real numbers of length 2.'
        )
        assert is_real_number(angle), (
            '<State/translate_rotate>: argument angle must be a scalar. '
            'angle = %s' % angle
        )
        assert is_valid_orientation(angle), (
            '<State/translate_rotate>: argument angle must be within the '
            'interval [-2pi,2pi]. angle = %s.' % angle
        )
        transformed_state = copy.copy(self)
        if hasattr(self, 'position'):
            # exact position:
            if isinstance(self.position, ValidTypes.ARRAY):
                transformed_state.position = commonroad.geometry.transform.translate_rotate(
                    np.array([self.position]), translation, angle
                )[
                    0
                ]
            # uncertain position:
            elif isinstance(self.position, Shape):
                transformed_state.position = self.position.translate_rotate(
                    translation, angle
                )
            else:
                raise TypeError(
                    '<State/translate_rotate> Expected instance of %s or %s. Got %s instead.'
                    % (ValidTypes.ARRAY, Shape, self.position.__class__)
                )
        if hasattr(self, 'orientation'):
            # orientation can be either an interval or an exact value
            if isinstance(self.orientation, ValidTypes.NUMBERS):
                transformed_state.orientation = make_valid_orientation(
                    self.orientation + angle
                )
            elif isinstance(self.orientation, AngleInterval):
                transformed_state.orientation = transformed_state.orientation + angle
            else:
                raise TypeError(
                    '<State/translate_rotate> Expected instance of %s or %s. Got %s instead.'
                    % (ValidTypes.NUMBERS, AngleInterval, self.orientation.__class__)
                )
        return transformed_state

    @property
    def attributes(self) -> List[str]:
        """ Returns all dynamically set attributes of an instance of State.

        :Example:

        >>> import numpy as np
        >>> from commonroad.scenario.trajectory import State
        >>> state = State(position=np.array([0.0, 0.0]), orientation=0.1, velocity=3.4)
        >>> print(state.attributes)
        ['position', 'orientation', 'velocity']

        :return: subset of slots which are dynamically assigned to the object.
        """
        attributes = list()
        for slot in self.__slots__:
            if hasattr(self, slot):
                attributes.append(slot)
        return attributes

    def __str__(self):
        traffic_str = '\n'
        for attr in self.attributes:
            traffic_str += attr
            traffic_str += '= {}\n'.format(self.__getattribute__(attr))
        return traffic_str


class Trajectory:
    """ Class to model the movement of an object over time. The states of the trajectory can be either exact or
    uncertain (see :class:`commonroad.scenario.trajectory.State`); however, only exact time_step are allowed. """

    def __init__(self, initial_time_step: int, state_list: List[State]):
        """
        :param initial_time_step: initial time step of the trajectory
        :param state_list: ordered sequence of states over time representing the trajectory. It is assumed that
        the time discretization between two states matches the time discretization of the scenario.
        """
        self.initial_time_step: int = initial_time_step
        self.state_list: List[State] = state_list

    @property
    def initial_time_step(self) -> int:
        """ Initial time step of the trajectory."""
        return self._initial_time_step

    @initial_time_step.setter
    def initial_time_step(self, initial_time_step):
        assert isinstance(initial_time_step, int), (
            '<Trajectory/initial_time_step>: argument initial_time_step of '
            'wrong type. Expected type: %s. Got type: %s.'
            % (int, type(initial_time_step))
        )
        self._initial_time_step = initial_time_step

    @property
    def state_list(self) -> List[State]:
        """ List of states of the trajectory over time."""
        return self._state_list

    @state_list.setter
    def state_list(self, state_list):
        assert isinstance(state_list, list), (
            '<Trajectory/state_list>: argument state_list of wrong type. '
            'Expected type: %s. Got type: %s.' % (list, type(state_list))
        )
        assert len(state_list) >= 1, (
            '<Trajectory/state_list>: argument state_list must contain at least one state.'
            ' length of state_list: %s.' % len(state_list)
        )
        assert all(isinstance(state, State) for state in state_list), (
            '<Trajectory/state_list>: element of '
            'state_list is of wrong type. Expected type: '
            '%s.' % List[State]
        )
        assert all(
            is_natural_number(state.time_step)
            for state in state_list
            if hasattr(state, 'time_step')
        ), '<Trajectory/state_list>: Element time_step of each state must be an integer.'
        assert all(
            state_list[0].attributes == state.attributes for state in state_list
        ), (
            '<Trajectory/state_list>: all states must have the same attributes. Attributes of first state: %s.'
            % state_list[0].attributes
        )
        self._state_list = state_list

    @property
    def final_state(self) -> State:
        """ Final state of the trajectory."""
        return self._state_list[-1]

    def state_at_time_step(self, time_step: int) -> Union[State, None]:
        """
        Function to get the state of a trajectory at a specific time instance.

        :param time_step: considered time step
        :return: state of the trajectory at time_step
        """
        assert isinstance(time_step, int), (
            '<Trajectory/state_at_time_step>: argument time_step of '
            'wrong type. Expected type: %s. Got type: %s.' % (int, type(time_step))
        )
        state = None
        if (
            self._initial_time_step
            <= time_step
            < self._initial_time_step + len(self._state_list)
        ):
            state = self._state_list[time_step - self._initial_time_step]
        return state

    def states_in_time_interval(self, time_begin: int, time_end: int) -> List[Union[State, None]]:
        """
        Function to get the states of a trajectory at a specific time interval.

        :param time_begin: first considered time step
        :param time_end: last considered time step
        :return: list of states
        """
        assert time_end >= time_begin
        return [self.state_at_time_step(time_step) for time_step in range(time_begin,time_end+1)]

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """ First translates each state of the trajectory, then rotates each state of the trajectory around the
        origin.

        :param translation: translation vector [x_off, y_off] in x- and y-direction
        :param angle: rotation angle in radian (counter-clockwise)
        """
        assert is_real_number_vector(translation, 2), (
            '<Trajectory/translate_rotate>: argument translation is not '
            'a vector of real numbers of length 2.'
        )
        assert is_real_number(angle), (
            '<Trajectory/translate_rotate>: argument angle must be a scalar. '
            'angle = %s' % angle
        )
        assert is_valid_orientation(angle), (
            '<Trajectory/translate_rotate>: argument angle must be within the '
            'interval [-2pi,2pi]. angle = %s' % angle
        )

        for i in range(len(self._state_list)):
            self._state_list[i] = self._state_list[i].translate_rotate(
                translation, angle
            )

    def __str__(self):
        traffic_str = '\n'
        traffic_str += 'Initial time step: {} \n'.format(self.initial_time_step)
        traffic_str += 'Number of states: {}\n'.format(len(self.state_list))
        traffic_str += 'State elements: {}'.format(self.state_list[0].attributes)
        return traffic_str
