from __future__ import annotations

import abc
import copy
import dataclasses
import json
import math
import warnings
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Union

import numpy as np

import commonroad.geometry.transform
from commonroad.common.util import AngleInterval, Interval, make_valid_orientation
from commonroad.common.validity import (
    ValidTypes,
    is_real_number,
    is_real_number_vector,
    is_valid_orientation,
)
from commonroad.geometry.shape import Shape
from commonroad.visualization.draw_params import (
    OptionalSpecificOrAllDrawParams,
    StateParams,
)
from commonroad.visualization.renderer import IRenderer

FloatExactOrInterval = Union[float, Interval]
AngleExactOrInterval = Union[float, AngleInterval]
ExactOrShape = Union[np.ndarray, Shape]


class MetaInformationState:
    """
    Class that keeps the meta information state of an obstacle.
    It freely allows to set attributes on a dynamic obstacle.
    """

    def __init__(
        self,
        meta_data_str: Dict[str, str] = None,
        meta_data_int: Dict[str, int] = None,
        meta_data_float: Dict[str, float] = None,
        meta_data_bool: Dict[str, bool] = None,
    ):
        """
        :param meta_data_str: dictionary of a key and a string
        :param meta_data_int: dictionary of a key and an int
        :param meta_data_float: dictionary of a key and a float
        :param meta_data_bool: dictionary of a key and a bool
        """
        self._meta_data_str = meta_data_str
        self._meta_data_int = meta_data_int
        self._meta_data_float = meta_data_float
        self._meta_data_bool = meta_data_bool

    @property
    def meta_data_str(self) -> Dict[str, str]:
        """Dictionary of a key and a string."""
        return self._meta_data_str

    @meta_data_str.setter
    def meta_data_str(self, meta_data_str: Dict[str, str]):
        assert isinstance(
            meta_data_str, Dict
        ), "<MetaInformationState/meta_data_str>: Provided meta_data_str " "is not valid! id={}".format(meta_data_str)
        self._meta_data_str = meta_data_str

    @property
    def meta_data_int(self) -> Dict[str, int]:
        """Dictionary of a key and an int."""
        return self._meta_data_int

    @meta_data_int.setter
    def meta_data_int(self, meta_data_int: Dict[str, int]):
        assert isinstance(
            meta_data_int, Dict
        ), "<MetaInformationState/meta_data_int>: Provided meta_data_int " "is not valid! id={}".format(meta_data_int)
        self._meta_data_int = meta_data_int

    @property
    def meta_data_float(self) -> Dict[str, float]:
        """Dictionary of a key and a float."""
        return self._meta_data_float

    @meta_data_float.setter
    def meta_data_float(self, meta_data_float: Dict[str, float]):
        assert isinstance(
            meta_data_float, Dict
        ), "<MetaInformationState/meta_data_float>: Provided meta_data_float " "is not valid! id={}".format(
            meta_data_float
        )
        self._meta_data_float = meta_data_float

    @property
    def meta_data_bool(self) -> Dict[str, bool]:
        """Dictionary of a key and a bool."""
        return self._meta_data_bool

    @meta_data_bool.setter
    def meta_data_bool(self, meta_data_bool: Dict[str, bool]):
        assert isinstance(
            meta_data_bool, Dict
        ), "<MetaInformationState/meta_data_bool>: Provided meta_data_bool " "is not valid! id={}".format(
            meta_data_bool
        )
        self._meta_data_bool = meta_data_bool

    def __hash__(self):
        return hash(
            (
                json.dumps(self._meta_data_str),
                json.dumps(self._meta_data_int),
                json.dumps(self._meta_data_float),
                json.dumps(self._meta_data_bool),
            )
        )


@dataclass
class State(abc.ABC):
    """
    This is a class representing the Base State.

    :param time_step: Discrete time step :math:`t_i`
    """

    time_step: Union[int, Interval] = None

    def __eq__(self, other: State):
        if not isinstance(other, State):
            return False
        if set(self.attributes) != set(other.attributes):
            return False

        dec = 10
        for attr in self.attributes:
            val_self = getattr(self, attr)
            val_other = getattr(other, attr)

            if attr == "position" and (isinstance(val_self, np.ndarray) or isinstance(val_other, np.ndarray)):
                if isinstance(val_self, np.ndarray) and isinstance(val_other, np.ndarray):
                    val_self = tuple(np.around(self.position.astype(float), dec))
                    val_other = tuple(np.around(self.position.astype(float), dec))
                else:
                    return False

            if isinstance(val_self, float):
                val_self = round(val_self, dec)
            if isinstance(val_other, float):
                val_other = round(val_other, dec)

            if val_self != val_other:
                return False

        return True

    def __hash__(self):
        values = list()

        dec = 10
        for attr in self.attributes:
            val = getattr(self, attr)

            if attr == "position" and isinstance(self.position, np.ndarray):
                val = tuple(np.around(self.position.astype(float), dec))

            if isinstance(val, float):
                val = round(val, dec)
            values.append(val)
        return hash(tuple(values))

    def __array__(self: Union[TraceState]) -> np.ndarray:
        """
        Converts the State into a 1D-numpy array by iterating over all fields of the dataclass. The order of the fields
        as defined in the dataclass is preserved.
        Note: Time step is not included, as the numpy array only contains the state vector.
        """
        values = list()
        for field in dataclasses.fields(self):
            if field.name == "time_step":
                # time step not included in state array
                continue
            elif field.name == "position":
                values.append(getattr(self, field.name)[0])  # x-position
                values.append(getattr(self, field.name)[1])  # y-position
            else:
                values.append(getattr(self, field.name))
        return np.array(values)

    @property
    def attributes(self) -> List[str]:
        """
        Returns all attributes used in state space.

        Attributes
        """
        fields = self.__dict__

        return [field_name for field_name in fields]

    @property
    def used_attributes(self) -> List[str]:
        """
        Returns all initialized attributed in state space.

        Initialized attributes
        """
        used_fields = list()
        for field_name in self.attributes:
            if getattr(self, field_name) is not None:
                used_fields.append(field_name)

        return used_fields

    @property
    def is_uncertain_position(self) -> bool:
        """
        Checks whether the position is uncertain.

        Uncertain or not
        """
        if hasattr(self, "position"):
            return isinstance(getattr(self, "position"), Shape)
        return False

    @property
    def is_uncertain_orientation(self):
        """
        Checks whether the orientation is uncertain.

        Uncertain or not
        """
        if hasattr(self, "orientation"):
            return isinstance(getattr(self, "orientation"), AngleInterval)
        return False

    def has_value(self, attr: str) -> bool:
        """
        Checks whether an attribute is given and is initialized with a value.

        :param attr: Name of attribute
        """
        return hasattr(self, attr) and getattr(self, attr) is not None

    def translate_rotate(self, translation: np.ndarray, angle: float) -> TraceState:
        """
        First translates the state and then rotates the state around the origin.

        :param translation: Translation vector [x_off, y_off] in x- and y-direction
        :param angle: Rotation angle in radian (counter-clockwise)
        :return: Transformed state
        """
        assert is_real_number_vector(translation, 2), (
            "<State/translate_rotate>: argument translation is not " "a vector of real numbers of length 2."
        )
        assert is_real_number(angle), "<State/translate_rotate>: argument angle must be a scalar. " "angle = %s" % angle
        assert is_valid_orientation(angle), (
            "<State/translate_rotate>: argument angle must be within the " "interval [-2pi,2pi]. angle = %s." % angle
        )

        transformed_state = copy.copy(self)
        if hasattr(self, "position") and getattr(self, "position") is not None:
            if isinstance(self.position, ValidTypes.ARRAY):
                transformed_state.position = commonroad.geometry.transform.translate_rotate(
                    np.array([self.position]), translation, angle
                )[0]
            elif isinstance(self.position, Shape):
                transformed_state.position = self.position.translate_rotate(translation, angle)
            else:
                raise TypeError(
                    "<State/translate_rotate> Expected instance of %s or %s. Got %s instead."
                    % (ValidTypes.ARRAY, Shape, self.position.__class__)
                )

        if hasattr(self, "orientation") and getattr(self, "orientation") is not None:
            if isinstance(self.orientation, ValidTypes.NUMBERS):
                transformed_state.orientation = make_valid_orientation(self.orientation + angle)
            elif isinstance(self.orientation, AngleInterval):
                transformed_state.orientation = transformed_state.orientation + angle
            else:
                raise TypeError(
                    "<State/translate_rotate> Expected instance of %s or %s. Got %s instead."
                    % (ValidTypes.NUMBERS, AngleInterval, self.orientation.__class__)
                )

        return transformed_state

    def convert_state_to_state(self, state: SpecificStateClasses) -> SpecificStateClasses:
        """
        Converts state to state from different state types.

        :param state: State for converting
        """
        for to_field in dataclasses.fields(type(state)):
            for from_field in self.attributes:
                if from_field == to_field.name:
                    setattr(state, to_field.name, getattr(self, from_field))
                    break
        return state

    def fill_with_defaults(self):
        """
        Fills all state fields with default values.

        """
        for field in self.attributes:
            if getattr(self, field) is not None:
                continue

            if field == "position":
                setattr(self, field, np.array([0.0, 0.0]))
            else:
                setattr(self, field, 0.0)

    def draw(self, renderer: IRenderer, draw_params: OptionalSpecificOrAllDrawParams[StateParams] = None):
        """
        Draws state.

        :param renderer: Renderer
        :param draw_params: Params
        """
        renderer.draw_state(self, draw_params)


@dataclass(eq=False)
class InitialState(State):
    """
    This is a class representing the Initial State.

    :param position: Position :math:`s_x`- and :math:`s_y` in a global coordinate system
    :param orientation: Yaw angle :math:`\\Psi`
    :param velocity: Velocity :math:`v_x` in longitudinal direction
    :param acceleration: Acceleration :math:`a_x`
    :param yaw_rate: Yaw rate :math:`\\dot{\\Psi}`
    :param slip_angle: Slip angle :math:`\\beta`
    """

    position: ExactOrShape = None
    orientation: AngleExactOrInterval = None
    velocity: FloatExactOrInterval = None
    acceleration: FloatExactOrInterval = None
    yaw_rate: FloatExactOrInterval = None
    slip_angle: FloatExactOrInterval = None


@dataclass(eq=False)
class PMState(State):
    """
    This is a class representing Point Mass State (PM State).

    :param position: Position :math:`s_x`- and :math:`s_y` in a global coordinate system
    :param velocity: Velocity :math:`v_x` in longitudinal direction
    :param velocity_y: Velocity :math:`v_x` in lateral direction
    """

    position: ExactOrShape = None
    velocity: FloatExactOrInterval = None
    velocity_y: FloatExactOrInterval = None

    @property
    def orientation(self) -> Optional[float]:
        """
        Orientation: Yaw angle :math:`\\Psi`
        Does not consider intervals.
        """
        if self.velocity is not None and self.velocity_y is not None:
            return math.atan2(self.velocity_y, self.velocity)
        else:
            return None


@dataclass(eq=False)
class ExtendedPMState(State):
    """
    This is a class extending the Point Mass State (PM State) since many existing CommonRoad scenarios use position,
    velocity, orientation, and acceleration as state which is not supported by a commonroad-io class.

    :param position: Position :math:`s_x`- and :math:`s_y` in a global coordinate system
    :param velocity: :math:`v_x` in longitudinal direction
    :param orientation: Yaw angle :math:`\\Psi`
    :param acceleration: Acceleration :math:`a_x`
    """

    position: ExactOrShape = None
    velocity: FloatExactOrInterval = None
    orientation: AngleExactOrInterval = None
    acceleration: FloatExactOrInterval = None

    @property
    def velocity_y(self) -> Optional[float]:
        """
        Velocity_y: math:`v_x` in lateral direction
        Does not consider intervals.
        """
        if self.velocity is not None and self.orientation is not None:
            return math.sin(self.orientation) * self.velocity
        else:
            return None


@dataclass(eq=False)
class KSState(State):
    """
    This is a class representing Kinematic Single Track State (KS State).

    :param position: Position :math:`s_x`- and :math:`s_y` in a global coordinate system
    :param steering_angle: Steering angle :math:`\\delta`
    :param velocity: Velocity :math:`v_x` in longitudinal direction
    :param orientation: Yaw angle :math:`\\Psi`
    """

    position: ExactOrShape = None
    steering_angle: FloatExactOrInterval = None
    velocity: FloatExactOrInterval = None
    orientation: AngleExactOrInterval = None


@dataclass(eq=False)
class KSTState(KSState):
    """
    This is a class representing Kinematic Single Track State (KST State).

    :param hitch_angle: Hitch angle :math:`\\alpha`
    """

    hitch_angle: AngleExactOrInterval = None


@dataclass(eq=False)
class STState(KSState):
    """
    This is a class representing Single Track State (ST State).

    :param slip_angle: Slip angle :math:`\\beta`
    :param yaw_rate: Yaw rate :math:`\\dot{\\Psi}`
    """

    slip_angle: FloatExactOrInterval = None
    yaw_rate: FloatExactOrInterval = None


@dataclass(eq=False)
class STDState(STState):
    """
    This is a class representing Single Track Drift State (STD State).

    :param front_wheel_angular_speed: Front wheel angular speed :math:`\\omega_{f}`
    :param rear_wheel_angular_speed: Rear wheel angular speed :math:`\\omega_{r}`
    """

    front_wheel_angular_speed: FloatExactOrInterval = None
    rear_wheel_angular_speed: FloatExactOrInterval = None


@dataclass(eq=False)
class MBState(State):
    """
    This is a class representing Multi Body State (MB State).

    :param position: Position :math:`s_x`- and :math:`s_y` in a global coordinate system
    :param steering_angle: Steering angle :math:`\\delta`
    :param velocity: Velocity :math:`v_x` in longitudinal direction
    :param orientation: Yaw angle :math:`\\Psi`
    :param yaw_rate: Yaw rate :math:`\\dot{\\Psi}`
    :param roll_angle: Roll angle :math:`\\Phi_S`
    :param roll_rate: Roll rate :math:`\\dot{\\Phi}_S`
    :param pitch_angle: Pitch angle :math:`\\Theta_S`
    :param pitch_rate: Pitch rate :math:`\\dot{\\Theta}_S`
    :param velocity_y: Velocity :math:`v_y` in lateral direction
    :param position_z: Position :math:`s_z` (height) from ground
    :param velocity_z: Velocity :math:`v_z` in vertical direction perpendicular to road plane
    :param roll_angle_front: Roll angle front :math:`\\Phi_{UF}`
    :param roll_rate_front: Roll rate front :math:`\\dot{\\Phi}_{UF}`
    :param velocity_y_front: Velocity :math:`v_{y,UF}` in y-direction front
    :param position_z_front: Position :math:`s_{z,UF}` in z-direction front
    :param velocity_z_front: Velocity :math:`v_{z,UF}` in z-direction front
    :param roll_angle_rear: Roll angle rear :math:`\\Phi_{UR}`
    :param roll_rate_rear: Roll rate rear :math:`\\dot{\\Phi}_{UR}`
    :param velocity_y_rear: Velocity :math:`v_{y,UR}` in y-direction rear
    :param position_z_rear: Position :math:`s_{z,UR}` in z-direction rear
    :param velocity_z_rear: Velocity :math:`v_{z,UR}` in z-direction rear
    :param left_front_wheel_angular_speed: Left front wheel angular speed :math:`\\omega_{LF}`
    :param right_front_wheel_angular_speed: Right front wheel angular speed :math:`\\omega_{RF}`
    :param left_rear_wheel_angular_speed: Left rear wheel angular speed :math:`\\omega_{LR}`
    :param right_rear_wheel_angular_speed: Right rear wheel angular speed :math:`\\omega_{RR}`
    :param delta_y_f: Front lateral displacement :math:`\\delta_{y,f}` of sprung mass due to roll
    :param delta_y_r: Rear lateral displacement :math:`\\delta_{y,r}` of sprung mass due to roll
    """

    position: ExactOrShape = None
    steering_angle: FloatExactOrInterval = None
    velocity: FloatExactOrInterval = None
    orientation: AngleExactOrInterval = None
    yaw_rate: FloatExactOrInterval = None
    roll_angle: FloatExactOrInterval = None
    roll_rate: FloatExactOrInterval = None
    pitch_angle: FloatExactOrInterval = None
    pitch_rate: FloatExactOrInterval = None
    velocity_y: FloatExactOrInterval = None
    position_z: FloatExactOrInterval = None
    velocity_z: FloatExactOrInterval = None
    roll_angle_front: FloatExactOrInterval = None
    roll_rate_front: FloatExactOrInterval = None
    velocity_y_front: FloatExactOrInterval = None
    position_z_front: FloatExactOrInterval = None
    velocity_z_front: FloatExactOrInterval = None
    roll_angle_rear: FloatExactOrInterval = None
    roll_rate_rear: FloatExactOrInterval = None
    velocity_y_rear: FloatExactOrInterval = None
    position_z_rear: FloatExactOrInterval = None
    velocity_z_rear: FloatExactOrInterval = None
    left_front_wheel_angular_speed: FloatExactOrInterval = None
    right_front_wheel_angular_speed: FloatExactOrInterval = None
    left_rear_wheel_angular_speed: FloatExactOrInterval = None
    right_rear_wheel_angular_speed: FloatExactOrInterval = None
    delta_y_f: FloatExactOrInterval = None
    delta_y_r: FloatExactOrInterval = None


@dataclass(eq=False)
class LongitudinalState(State):
    """
    This is a class representing the Longitudinal Motion State (Longitudinal State). The state cannot be read by
    the file reader because neither orientation nor velocity in y direction are specified.

    :param longitudinal_position: Longitudinal position :math:`s` along reference path
    :param velocity: Velocity :math:`v_x` in longitudinal direction
    :param acceleration: Acceleration :math:`a_x` in longitudinal direction
    :param jerk: Jerk :math:`j`
    """

    longitudinal_position: FloatExactOrInterval = None
    velocity: FloatExactOrInterval = None
    acceleration: FloatExactOrInterval = None
    jerk: FloatExactOrInterval = None


@dataclass(eq=False)
class LateralState(State):
    """
    This is a class representing the Lateral Motion State (Lateral State). The state cannot be read by the file reader
    because position is not specified.

    :param lateral_position: Lateral distance :math:`d` to reference path
    :param orientation: Yaw angle :math:`\\Psi`
    :param curvature: Curvature math:`\\kappa`
    :param curvature_rate: Change of curvature math:`\\dot{\\kappa}`
    """

    lateral_position: FloatExactOrInterval = None
    orientation: AngleExactOrInterval = None
    curvature: FloatExactOrInterval = None
    curvature_rate: FloatExactOrInterval = None


@dataclass(eq=False)
class InputState(State):
    """
    This is a class representing the input for most supported models.

    :param steering_angle_speed: Steering angle speed :math:`\\dot{\\delta}` of front wheels
    :param acceleration: Acceleration :math:`a_x`
    """

    steering_angle_speed: FloatExactOrInterval = None
    acceleration: FloatExactOrInterval = None


@dataclass(eq=False)
class PMInputState(State):
    """
    This is a class representing the input for PM model (PM Input).

    :param acceleration: Acceleration :math:`a_x`
    :param acceleration_y: Acceleration :math:`a_y`
    """

    acceleration: FloatExactOrInterval = None
    acceleration_y: FloatExactOrInterval = None


@dataclass(eq=False)
class LKSInputState(State):
    """
    This is a class representing the input for the linearized kinematic single track (LKS) model (LKS Input).

    :param jerk_dot: change of longitudinal jerk (i.e., jerk_dot): math:`\\dot{j}`
    :param kappa_dot_dot: change of curvature rate (i.e., kappa_dot_dot)  :math:`\\ddot{\\kappa}`
    """

    jerk_dot: FloatExactOrInterval = None
    kappa_dot_dot: FloatExactOrInterval = None


class CustomState(State):
    """
    This is a class representing the custom state. State variables can be added at runtime. The attributes position
    and orientation/velocity_y are necessary for successful file reading.
    """

    def __init__(self, **attributes):
        """
        Additional constructor for CustomState class.

        :param attributes: Variable number of attributes each consisting of name and value.
        """
        if len(attributes) > 0:  # if one wants to use the attribute adding methods manually
            super().__init__(attributes["time_step"])
        for name, value in attributes.items():
            if name == "time_step":
                continue
            self.add_attribute(name)
            self.set_value(name, value)

    def add_attribute(self, new_attr: str):
        """
        Adds a new attribute to custom state.

        :param new_attr: Attribute name
        """
        setattr(self, new_attr, None)

    def set_value(self, attr: str, value: Any):
        """
        Sets value to attribute.

        :param attr: Attribute name
        :param value: Value
        """
        assert attr in self.attributes, "{} is not an attribute of this custom state!".format(attr)

        setattr(self, attr, value)


class SignalState:
    """A signal state is a boolean value indicating the activity of the signal source at a time step.
    The possible signal state elements are defined as slots:

    :ivar bool horn: boolean indicating activity of horn
    :ivar bool indicator_left: boolean indicating activity of left indicator
    :ivar bool indicator_right: boolean indicating activity of right indicator
    :ivar bool braking_lights: boolean indicating activity of braking lights
    :ivar bool hazard_warning_lights: boolean indicating activity of hazard warning lights
    :ivar bool flashing_blue_lights: boolean indicating activity of flashing blue lights (police, ambulance)
    :ivar bool time_step: the discrete time step. Exact values are given as integers, uncertain values are given as
          :class:`commonroad.common.util.Interval`
    """

    __slots__ = [
        "horn",
        "indicator_left",
        "indicator_right",
        "braking_lights",
        "hazard_warning_lights",
        "flashing_blue_lights",
        "time_step",
    ]

    def __init__(self, **kwargs):
        """Elements of state vector are determined during runtime."""
        for field, value in kwargs.items():
            setattr(self, field, value)

    def __eq__(self, other):
        if not isinstance(other, SignalState):
            warnings.warn(f"Inequality between SignalState {repr(self)} and different type {type(other)}")
            return False

        for attr in SignalState.__slots__:
            value = None
            value_other = None

            has_attr = hasattr(self, attr)
            if has_attr:
                value = getattr(self, attr)

            has_attr_other = hasattr(other, attr)
            if has_attr_other:
                value_other = getattr(other, attr)

            if has_attr != has_attr_other or value != value_other:
                return False

        return True

    def __hash__(self):
        values = set()
        for attr in SignalState.__slots__:
            if hasattr(self, attr):
                values.add(getattr(self, attr))

        return hash(frozenset(values))


TraceState = Union[
    State,
    InitialState,
    PMState,
    KSState,
    KSTState,
    STState,
    STDState,
    MBState,
    InputState,
    PMInputState,
    LateralState,
    LongitudinalState,
    CustomState,
    ExtendedPMState,
]

SpecificStateClasses = [
    InitialState,
    PMState,
    KSState,
    KSTState,
    STState,
    STDState,
    MBState,
    InputState,
    PMInputState,
    LateralState,
    LongitudinalState,
    ExtendedPMState,
]
