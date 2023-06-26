import enum

import enum
from pathlib import Path

import numpy as np
import math
from scipy.spatial.transform.rotation import Rotation
from scipy.spatial.transform import Slerp
from typing import Tuple
from commonroad.common import validity
from commonroad.common.validity import *

Path_T = Union[str, bytes, Path]


def interpolate_angle(x: Union[float, np.array], xp: np.array, fp: np.array, degrees=False):
    """
    :param x: The x-coordinates at which to evaluate the interpolated values.
    :param xp: The x-coordinates of the data points.
    :param fp: The y-coordinates (angles) of the data points, same length as xp.
    :param degrees: True if the input and returned angles are in degrees
    :return: The interpolated angles in radian, same shape as x.
    """

    rotations = Rotation.from_euler("z", fp, degrees=degrees)
    slerp = Slerp(xp, rotations)
    return slerp(x).as_euler("zxy", degrees=degrees)[0]


def subtract_orientations(lhs, rhs):
    """Return the signed difference between angles lhs and rhs
    :param lhs: lhs of the subtraction
    :param rhs: rhs of the subtraction
    :return: ``(lhs - rhs)``, the value will be within ``[-math.pi, math.pi)``.
    Both ``lhs`` and ``rhs`` may either be zero-based (within
    ``[0, 2*math.pi]``), or ``-pi``-based (within ``[-math.pi, math.pi]``).
    """

    return math.fmod((lhs - rhs) + math.pi * 3.0, 2.0 * math.pi) - math.pi


def make_valid_orientation(angle: float) -> float:
    while angle > TWO_PI:
        angle = angle - TWO_PI
    while angle < -TWO_PI:
        angle = angle + TWO_PI
    return angle


def make_valid_orientation_interval(angle_start: float, angle_end: float) -> Tuple[float, float]:
    while angle_start > TWO_PI or angle_end > TWO_PI:
        angle_start -= TWO_PI
        angle_end -= TWO_PI
    while angle_start < -TWO_PI or angle_start < -TWO_PI:
        angle_start += TWO_PI
        angle_end += TWO_PI
    return angle_start, angle_end


def vectorized_angle_difference(lhs: float, rhs: float) -> float:
    """
    Calculates the difference between given two angles by calculating
    the arctan2 between the unit vectors with the given angles.

    :param lhs: lhs of the subtraction, ``[- 2k * math.pi, 2k * math.pi]``
    :param rhs: rhs of the subtraction, ``[- 2k * math.pi, 2k * math.pi]``
    :return: angle difference within range of ``[-math.pi, math.pi)``.
    """
    diff = math.fmod(lhs - rhs, TWO_PI)
    angle_diff = np.arctan2(np.sin(diff), np.cos(diff))
    return angle_diff


class Interval:
    def __init__(self, start: Union[int, float], end: Union[int, float]):
        self._start = None
        self._end = None
        self.start = start
        self.end = end

    def __eq__(self, other):
        if not isinstance(other, Interval):
            warnings.warn(f"Inequality between Interval {repr(self)} and different type {type(other)}")
            return False

        return self._start == other.start and self._end == other.end

    def __hash__(self):
        return hash((self._start, self._end))

    def __iter__(self):
        yield self._start
        yield self._end

    def __truediv__(self, other) -> 'Interval':
        if other > 0.0:
            return type(self)(self._start/other, self._end/other)
        else:
            return type(self)(self._end / other, self._start / other)

    def __mul__(self, other: Union[int, float]) -> 'Interval':
        if other > 0.0:
            return type(self)(self._start * other, self._end * other)
        else:
            return type(self)(self._end * other, self._start * other)

    def __round__(self, n=None) -> 'Interval':
        return type(self)(round(self._start, n), round(self._end, n))

    def __add__(self, other: Union[float, int]) -> 'Interval':
        return type(self)(self._start + other, self._end + other)

    def __sub__(self, other: Union[float, int]) -> 'Interval':
        return type(self)(self._start - other, self._end - other)

    @property
    def start(self) -> Union[int, float]:
        return self._start

    @start.setter
    def start(self, start: Union[int, float]):
        if self._end is not None:
            assert start <= self._end, '<common.util/Interval> start of interval must be <= end, ' \
                                       'but start {} > end {}'.format(start, self._end)
        self._start = start

    @property
    def end(self) -> Union[int, float]:
        return self._end

    @end.setter
    def end(self, end: Union[int, float]):
        if self._start is not None:
            assert end >= self._start, '<common.util/Interval> start of interval must be <= end, ' \
                                       'but start {} > end {}'.format(self._start, end)
        self._end = end

    def contains(self, other: Union[int, float, 'Interval']) -> bool:
        if type(other) is Interval:
            return self.start <= other.start and other.end <= self.end
        else:
            return self.start <= other <= self.end

    def __contains__(self, value: Union[int, float, "Interval"]):
        return self.contains(value)

    def overlaps(self, interval: 'Interval') -> bool:
        return self.end >= interval.start and interval.end >= self.start

    def intersection(self, other: 'Interval') -> Union[None, 'Interval']:
        """
        Returns the intersections with another interval.
        :param other: Interval
        :return: Interval or None if intersection is empty
        """
        if not self.overlaps(other):
            return None
        else:
            return Interval(max(self._start, other._start), min(self._end, other._end))

    @property
    def length(self):
        return self.end - self.start

    def __str__(self):
        info_str = "Interval\n"
        info_str += "start: {}\n".format(self._start)
        info_str += "end: {}\n".format(self._end)
        return info_str

    def __gt__(self, other):
        if isinstance(other, validity.ValidTypes.NUMBERS):
            return True if self._start > other else False
        else:
            return True if self._start > other.end else False

    def __lt__(self, other):
        if isinstance(other, validity.ValidTypes.NUMBERS):
            return True if self._end < other else False
        else:
            return True if self._end < other.start else False


class AngleInterval(Interval):
    """ Allows only angles from interval [-2pi,2pi]"""
    def __init__(self, start: Union[int, float], end: Union[int, float]):
        start, end = make_valid_orientation_interval(start, end)
        assert end - start < TWO_PI, '<common.util/AngleInterval> Interval must not be |start-end| > 2pi'
        Interval.__init__(self, start, end)

    @property
    def start(self) -> Union[int, float]:
        return self._start

    @start.setter
    def start(self, start: Union[int, float]):
        assert is_valid_orientation(start), '<common.util/AngleInterval> start angle needs to be in interval [-2pi,2pi]'
        if self._end is not None:
            assert start <= self._end, '<common.util/Interval> start of interval must be <= end, ' \
                                       'but start {} > end {}'.format(start, self._end)
        self._start = start

    @property
    def end(self) -> Union[int, float]:
        return self._end

    @end.setter
    def end(self, end: Union[int, float]):
        assert is_valid_orientation(end), '<common.util/AngleInterval> end angle needs to be in interval [-2pi,2pi]'
        if self._start is not None:
            assert end >= self._start, '<common.util/Interval> start of interval must be <= end, ' \
                                       'but start {} > end {}'.format(self._start, end)
        self._end = end

    def intersect(self, other: 'AngleInterval'):
        raise NotImplementedError()

    def contains(self, other: Union[float, 'AngleInterval']) -> bool:
        if isinstance(other, float):
            return self.__contains__(other)
        interval_diff = vectorized_angle_difference(self.end, self.start)
        assert interval_diff >= 0
        start_diff = vectorized_angle_difference(other.start, self.start)
        end_diff = vectorized_angle_difference(other.end, self.start)
        return 0 <= start_diff <= interval_diff and 0 <= end_diff <= interval_diff

    def __contains__(self, value: Union[int, float]):
        interval_diff = vectorized_angle_difference(self.end, self.start)
        assert interval_diff >= 0
        diff = vectorized_angle_difference(value, self.start)
        return 0 <= diff <= interval_diff


class FileFormat(enum.Enum):
    """
    Specifies the format of file.
    """
    XML = ".xml"
    PROTOBUF = ".pb"


class Time:
    """
    Class which describes the fictive time when a scenario starts.
    """

    def __init__(self, hours: int, minutes: int, day: int = None, month: int = None, year: int = None):
        """
        Constructor of a time object

        :param hours: hours at start of scenario (0-24)
        :param minutes: minutes at start of scenario (0-60)
        :param day: day at start of scenario (1-31)
        :param month: month at start of scenario (1-12)
        :param year: year at start of scenario
        """
        self._hours = hours
        self._minutes = minutes
        self._day = day
        self._month = month
        self._year = year

    def __eq__(self, other):
        if not isinstance(other, Time):
            return False

        return self._hours == other.hours and self._minutes == other.minutes and self._day == other.day and \
            self._month == other.month and self._year == other.year

    def __str__(self):
        return f"Year {self._year}, month {self._month}, day {self._day}, hour {self._hours}, minute {self.minutes}"

    def __hash__(self):
        return hash((self._hours, self._minutes, self._day, self._month, self._year))

    @property
    def hours(self) -> int:
        """ Hours at start of scenario (0-24)"""
        return self._hours

    @hours.setter
    def hours(self, hours: int):
        self._hours = hours

    @property
    def minutes(self) -> int:
        """ Minutes at start of scenario (0-60)"""
        return self._minutes

    @minutes.setter
    def minutes(self, minutes: int):
        self._minutes = minutes

    @property
    def day(self) -> Union[None, int]:
        """ Day at start of scenario (1-31)"""
        return self._day

    @day.setter
    def day(self, day: Union[None, int]):
        self._day = day

    @property
    def month(self) -> Union[None, int]:
        """ Month at start of scenario (1-12)"""
        return self._month

    @month.setter
    def month(self, month: Union[None, int]):
        self._month = month

    @property
    def year(self) -> Union[None, int]:
        """ Year at start of scenario"""
        return self._year

    @year.setter
    def year(self, year: Union[None, int]):
        self._year = year

