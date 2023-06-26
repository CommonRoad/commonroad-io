import enum
from typing import Set

import numpy as np

from commonroad.common.validity import *
import commonroad.geometry.transform


class RoadUser(enum.Enum):
    """
    Enum describing different types of road users
    """
    VEHICLE = 'vehicle'
    CAR = 'car'
    TRUCK = 'truck'
    BUS = 'bus'
    PRIORITY_VEHICLE = 'priorityVehicle'
    MOTORCYCLE = 'motorcycle'
    BICYCLE = 'bicycle'
    PEDESTRIAN = 'pedestrian'
    TRAIN = 'train'
    TAXI = 'taxi'


class LineMarking(enum.Enum):
    """
    Enum describing different types of line markings, i.e. dashed or solid lines
    """
    DASHED = 'dashed'
    SOLID = 'solid'
    BROAD_DASHED = 'broad_dashed'
    BROAD_SOLID = 'broad_solid'
    UNKNOWN = 'unknown'
    NO_MARKING = 'no_marking'


class LaneletType(enum.Enum):
    """
    Enum describing different types of lanelets
    """
    URBAN = 'urban'
    COUNTRY = 'country'
    HIGHWAY = 'highway'
    DRIVE_WAY = 'driveWay'
    MAIN_CARRIAGE_WAY = 'mainCarriageWay'
    ACCESS_RAMP = 'accessRamp'
    EXIT_RAMP = 'exitRamp'
    SHOULDER = 'shoulder'
    BUS_LANE = 'busLane'
    BUS_STOP = 'busStop'
    BICYCLE_LANE = 'bicycleLane'
    SIDEWALK = 'sidewalk'
    CROSSWALK = 'crosswalk'
    INTERSTATE = 'interstate'
    INTERSECTION = 'intersection'
    BORDER = 'border'
    PARKING = 'parking'
    RESTRICTED = 'restricted'  # cars not allowed, e.g., special lanes for busses
    UNKNOWN = 'unknown'


class StopLine:
    """Class which describes the stop line of a lanelet"""

    def __init__(self, start: np.ndarray, end: np.ndarray, line_marking: LineMarking, traffic_sign_ref: Set[int] = None,
                 traffic_light_ref: Set[int] = None):
        self._start = start
        self._end = end
        self._line_marking = line_marking
        self._traffic_sign_ref = traffic_sign_ref
        self._traffic_light_ref = traffic_light_ref

    def __eq__(self, other):
        if not isinstance(other, StopLine):
            warnings.warn(f"Inequality between StopLine {repr(self)} and different type {type(other)}")
            return False

        prec = 10
        start_string = np.array2string(np.around(self._start.astype(float), prec), precision=prec)
        start_other_string = np.array2string(np.around(other.start.astype(float), prec), precision=prec)
        end_string = np.array2string(np.around(self._end.astype(float), prec), precision=prec)
        end_other_string = np.array2string(np.around(other.end.astype(float), prec), precision=prec)

        if start_string == start_other_string and end_string == end_other_string and self._line_marking == \
                other.line_marking and self._traffic_sign_ref == other.traffic_sign_ref and self._traffic_light_ref \
                == other.traffic_light_ref:
            return True

        warnings.warn(f"Inequality of StopLine {repr(self)} and the other one {repr(other)}")
        return False

    def __hash__(self):
        if self._start is None:
            start_string = None
        else:
            start_string = np.array2string(np.around(self._start.astype(float), 10), precision=10)
        if self._end is None:
            end_string = None
        else:
            end_string = np.array2string(np.around(self._end.astype(float), 10), precision=10)
        sign_ref = None if self._traffic_sign_ref is None else frozenset(self._traffic_sign_ref)
        light_ref = None if self._traffic_light_ref is None else frozenset(self._traffic_light_ref)
        return hash((start_string, end_string, self._line_marking, sign_ref, light_ref))

    def __str__(self):
        return f'StopLine from {self._start} to {self._end}'

    def __repr__(self):
        return f"StopLine(start={self._start.tolist()}, end={self._end.tolist()}, line_marking={self._line_marking}, " \
               f"traffic_sign_ref={self._traffic_sign_ref}, traffic_light_ref={self._traffic_light_ref})"

    @property
    def start(self) -> np.ndarray:
        return self._start

    @start.setter
    def start(self, value: np.ndarray):
        self._start = value

    @property
    def end(self) -> np.ndarray:
        return self._end

    @end.setter
    def end(self, value: np.ndarray):
        self._end = value

    @property
    def line_marking(self) -> LineMarking:
        return self._line_marking

    @line_marking.setter
    def line_marking(self, marking: LineMarking):
        self._line_marking = marking

    @property
    def traffic_sign_ref(self) -> Set[int]:
        return self._traffic_sign_ref

    @traffic_sign_ref.setter
    def traffic_sign_ref(self, references: Set[int]):
        self._traffic_sign_ref = references

    @property
    def traffic_light_ref(self) -> Set[int]:
        return self._traffic_light_ref

    @traffic_light_ref.setter
    def traffic_light_ref(self, references: Set[int]):
        self._traffic_light_ref = references

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        This method translates and rotates a stop line

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation, 2), '<Lanelet/translate_rotate>: provided translation ' \
                                                      'is not valid! translation = {}'.format(translation)
        assert is_valid_orientation(
                angle), '<Lanelet/translate_rotate>: provided angle is not valid! angle = {}'.format(angle)

        # create transformation matrix
        t_m = commonroad.geometry.transform.translation_rotation_matrix(translation, angle)
        line_vertices = np.array([self._start, self._end])
        # transform center vertices
        tmp = t_m.dot(np.vstack((line_vertices.transpose(), np.ones((1, line_vertices.shape[0])))))
        tmp = tmp[0:2, :].transpose()
        self._start, self._end = tmp[0], tmp[1]

