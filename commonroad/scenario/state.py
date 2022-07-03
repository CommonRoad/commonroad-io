from __future__ import annotations
import abc
import copy
import dataclasses
import enum
from dataclasses import dataclass
from typing import Union, List, Tuple, Optional

import numpy as np

import commonroad.geometry.transform
from commonroad.common.util import Interval, AngleInterval, make_valid_orientation
from commonroad.common.validity import is_real_number_vector, is_real_number, is_valid_orientation, ValidTypes
from commonroad.geometry.shape import Shape
from commonroad.visualization.param_server import ParamServer
from commonroad.visualization.renderer import IRenderer

FloatExactOrInterval = Union[float, Interval]
AngleExactOrInterval = Union[float, AngleInterval]
ExactOrShape = Union[np.ndarray, Shape]


class StateType(enum.Enum):
    INITIAL = 1
    PM = 2
    KS = 3
    KST = 4
    ST = 5
    STD = 6
    MB = 7


@dataclass
class State(abc.ABC):
    """
    This is a class representing the base state (Base State).

    Args:
        time_step: Discrete time step :math:`t_i`
        position: Position :math:`s_x` and :math:`s_y` in a global coordinate system
        velocity: Velocity :math:`v_x` in longitudinal direction
        acceleration: Acceleration :math:`a_x` in longitudinal direction

    """
    time_step: int = 0
    position: ExactOrShape = np.array([0., 0.])
    velocity: FloatExactOrInterval = 0.
    acceleration: FloatExactOrInterval = 0.

    @property
    def attributes(self) -> List[str]:
        """
        Returns all attributes used in state space.

        :return: Attributes
        """
        fields = dataclasses.fields(type(self))

        return [field.name for field in fields]

    @property
    def is_uncertain_position(self) -> bool:
        """
        Checks whether the position is uncertain.

        :return: Uncertain or not
        """
        if hasattr(self, "position"):
            return isinstance(getattr(self, "position"), Shape)
        return False

    @property
    def is_uncertain_orientation(self):
        """
        Checks whether the orientation is uncertain.

        :return: Uncertain or not
        """
        if hasattr(self, "orientation"):
            return isinstance(getattr(self, "orientation"), AngleInterval)
        return False

    def translate_rotate(self, translation: np.ndarray, angle: float) -> State:
        """
        First translates the state and then rotates the state around the origin.

        :param translation: Translation vector [x_off, y_off] in x- and y-direction
        :param angle: Rotation angle in radian (counter-clockwise)
        :return: Transformed state
        """
        assert is_real_number_vector(translation, 2), ('<State/translate_rotate>: argument translation is not '
                                                       'a vector of real numbers of length 2.')
        assert is_real_number(angle), ('<State/translate_rotate>: argument angle must be a scalar. '
                                       'angle = %s' % angle)
        assert is_valid_orientation(angle), ('<State/translate_rotate>: argument angle must be within the '
                                             'interval [-2pi,2pi]. angle = %s.' % angle)
        transformed_state = copy.copy(self)
        if hasattr(self, 'position'):
            if isinstance(self.position, ValidTypes.ARRAY):
                transformed_state.position = commonroad.geometry.transform.translate_rotate(np.array([self.position]),
                                                                                            translation, angle)[0]
            elif isinstance(self.position, Shape):
                transformed_state.position = self.position.translate_rotate(translation, angle)
            else:
                raise TypeError('<State/translate_rotate> Expected instance of %s or %s. Got %s instead.'
                                % (ValidTypes.ARRAY, Shape, self.position.__class__))

        if hasattr(self, 'orientation'):
            if isinstance(self.orientation, ValidTypes.NUMBERS):
                transformed_state.orientation = make_valid_orientation(self.orientation + angle)
            elif isinstance(self.orientation, AngleInterval):
                transformed_state.orientation = transformed_state.orientation + angle
            else:
                raise TypeError('<State/translate_rotate> Expected instance of %s or %s. Got %s instead.'
                                % (ValidTypes.NUMBERS, AngleInterval, self.orientation.__class__))

        return transformed_state

    def convertStateToState(self, state: State):
        """
        Converts state to state from different state types.

        :param state: State for converting
        """
        from_fields = dataclasses.fields(type(self))
        to_fields = dataclasses.fields(type(state))
        for from_field in from_fields:
            for to_field in to_fields:
                if from_field.name == to_field.name:
                    setattr(state, to_field.name, getattr(self, from_field.name))

        return state

    def draw(self, renderer: IRenderer, draw_params: Union[ParamServer, dict, None] = None,
             call_stack: Optional[Tuple[str, ...]] = tuple()):
        """
        Draws state.

        :param renderer: Renderer
        :param draw_params: Params
        :param call_stack: Call stack
        """
        renderer.draw_state(self, draw_params, call_stack)


@dataclass
class InitialState(State):
    """
    This is a class representing the initial state (Initial State).

    Args:
        orientation: Yaw angle :math:`\\Psi`

    """
    orientation: AngleExactOrInterval = 0.


@dataclass
class PMState(State):
    """
    This is a class representing Point Mass Model (PM State).

    Args:
        velocity_y: Velocity :math:`v_x` in lateral direction
        acceleration_y: Acceleration :math:`a_x` in lateral direction

    """
    velocity_y: FloatExactOrInterval = 0.
    acceleration_y: FloatExactOrInterval = 0.


@dataclass
class KSState(State):
    """
    This is a class representing Kinematic Single Track Model (KS State).

    Args:
        orientation: Yaw angle :math:`\\Psi`
        steering_angle: Steering angle :math:`\\delta`

    """
    orientation: AngleExactOrInterval = 0.
    steering_angle: FloatExactOrInterval = 0.


@dataclass
class KSTState(KSState):
    """
    This is a class representing Kinematic Single-Track Model (KST State).

    Args:
        hitch_angle: Hitch angle :math:`\\alpha`

    """
    hitch_angle: AngleExactOrInterval = 0.


@dataclass
class STState(KSState):
    """
    This is a class representing Single-Track Model (ST State).

    Args:
        slip_angle: Slip angle :math:`\\beta`
        yaw_rate: Yaw rate :math:`\\dot{\\Psi}`

    """
    slip_angle: FloatExactOrInterval = 0.
    yaw_rate: FloatExactOrInterval = 0.


@dataclass
class STDState(STState):
    """
    This is a class representing Single-Track Drift Model (STD State)

    Args:
        front_wheel_angular_speed: Front wheel angular speed :math:`\\omega_{f}`
        rear_wheel_angular_speed: Rear wheel angular speed :math:`\\omega_{r}`

    """
    front_wheel_angular_speed: FloatExactOrInterval = 0.
    rear_wheel_angular_speed: FloatExactOrInterval = 0.


@dataclass
class MBState(State):
    """
    This is a class representing Multi-Body Model (MB State).

    Args:
        steering_angle: Steering angle :math:`\\delta`
        orientation: Yaw angle :math:`\\Psi`
        yaw_rate: Yaw rate :math:`\\dot{\\Psi}`
        roll_angle: Roll angle :math:`\\Phi_S`
        roll_rate: Roll rate :math:`\\dot{\\Phi}_S`
        pitch_angle: Pitch angle :math:`\\Theta_S`
        pitch_rate: Pitch rate :math:`\\dot{\\Theta}_S`
        velocity_y: Velocity :math:`v_y` in lateral direction
        position_z: Position :math:`s_z` (height) from ground
        velocity_z: Velocity :math:`v_z` in vertical direction perpendicular to road plane
        roll_angle_front: Roll angle front :math:`\\Phi_{UF}`
        roll_rate_front: Roll rate front :math:`\\dot{\\Phi}_{UF}`
        velocity_y_front: Velocity :math:`v_{y,UF}` in y-direction front
        position_z_front: Position :math:`s_{z,UF}` in z-direction front
        velocity_z_front: Velocity :math:`v_{z,UF}` in z-direction front
        roll_angle_rear: Roll angle rear :math:`\\Phi_{UR}`
        roll_rate_rear: Roll rate rear :math:`\\dot{\\Phi}_{UR}`
        velocity_y_rear: Velocity :math:`v_{y,UR}` in y-direction rear
        position_z_rear: Position :math:`s_{z,UR}` in z-direction rear
        velocity_z_rear: Velocity :math:`v_{z,UR}` in z-direction rear
        left_front_wheel_angular_speed: Left front wheel angular speed :math:`\\omega_{LF}`
        right_front_wheel_angular_speed: Right front wheel angular speed :math:`\\omega_{RF}`
        left_rear_wheel_angular_speed: Left rear wheel angular speed :math:`\\omega_{LR}`
        right_rear_wheel_angular_speed: Right rear wheel angular speed :math:`\\omega_{RR}`
        delta_y_f: Front lateral displacement :math:`\\delta_{y,f}` of sprung mass due to roll
        delta_y_r: Rear lateral displacement :math:`\\delta_{y,r}` of sprung mass due to roll

    """
    steering_angle: FloatExactOrInterval = 0.
    orientation: AngleExactOrInterval = 0.
    yaw_rate: FloatExactOrInterval = 0.
    roll_angle: FloatExactOrInterval = 0.
    roll_rate: FloatExactOrInterval = 0.
    pitch_angle: FloatExactOrInterval = 0.
    pitch_rate: FloatExactOrInterval = 0.
    velocity_y = FloatExactOrInterval = 0.
    acceleration_y: FloatExactOrInterval = 0.
    position_z: FloatExactOrInterval = 0.
    velocity_z: FloatExactOrInterval = 0.
    roll_angle_front: FloatExactOrInterval = 0.
    roll_rate_front: FloatExactOrInterval = 0.
    velocity_y_front: FloatExactOrInterval = 0.
    position_z_front: FloatExactOrInterval = 0.
    velocity_z_front: FloatExactOrInterval = 0.
    roll_angle_rear: FloatExactOrInterval = 0.
    roll_rate_rear: FloatExactOrInterval = 0.
    velocity_y_rear: FloatExactOrInterval = 0.
    position_z_rear: FloatExactOrInterval = 0.
    velocity_z_rear: FloatExactOrInterval = 0.
    left_front_wheel_angular_speed: FloatExactOrInterval = 0.
    right_front_wheel_angular_speed: FloatExactOrInterval = 0.
    left_rear_wheel_angular_speed: FloatExactOrInterval = 0.
    right_rear_wheel_angular_speed: FloatExactOrInterval = 0.
    delta_y_f: FloatExactOrInterval = 0.
    delta_y_r: FloatExactOrInterval = 0.
