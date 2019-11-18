import numpy as np
from typing import Union, Tuple

from commonroad.common.validity import *

__author__ = "Stefanie Manzinger, Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "2019.1"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad@in.tum.de"
__status__ = "Released"


def make_valid_orientation(angle: float) -> float:
    while angle > 2 * np.pi:
        angle = angle - 2 * np.pi
    while angle < -2 * np.pi:
        angle = angle + 2 * np.pi
    return angle


def make_valid_orientation_interval(angle_start: float, angle_end: float) -> Tuple[float, float]:
    while angle_start > 2 * np.pi or angle_end > 2 * np.pi :
        angle_start -= 2 * np.pi
        angle_end -= 2 * np.pi
    while angle_start < -2 * np.pi or angle_start < -2 * np.pi:
        angle_start += 2 * np.pi
        angle_end += 2 * np.pi
    return angle_start, angle_end


class Interval:
    def __init__(self, start: Union[int, float], end: Union[int, float]):
        self.start = start
        self.end = end

    def __iter__(self):
        yield self._start
        yield self._end

    def __truediv__(self, other) -> 'Interval':
        return type(self)(self._start/other, self._end/other)

    def __mul__(self, other: Union[int,float]) -> 'Interval':
        return type(self)(self._start * other, self._end * other)

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
        if hasattr(self, '_end'):
            assert start <= self._end, '<common.util/Interval> start of interval must be <= end, but start {} > end {}'.format(start,self._end)
        self._start = start

    @property
    def end(self) -> Union[int, float]:
        return self._end

    @end.setter
    def end(self, end: Union[int, float]):
        if hasattr(self, '_start'):
            assert end >= self._start, '<common.util/Interval> start of interval must be <= end, but start {} > end {}'.format(self._start, end)
        self._end = end

    def contains(self, value: float) -> bool:
        return np.greater_equal(value, self.start) and np.greater_equal(self.end, value)

    def overlaps(self, interval: 'Interval') -> bool:
        return np.greater_equal(self.end, interval.start) and np.greater_equal(interval.end, self.start)

    def __str__(self):
        info_str = "Interval\n"
        info_str += "start: {}\n".format(self._start)
        info_str += "end: {}\n".format(self._end)
        return info_str


class AngleInterval(Interval):
    """ Allows only angles from interval [-2pi,2pi]"""
    def __init__(self, start: Union[int, float], end: Union[int, float]):
        start, end = make_valid_orientation_interval(start,end)
        assert end - start < 2 * np.pi, '<common.util/AngleInterval> Interval must not be |start-end| > 2pi'
        Interval.__init__(self, start, end)

    @property
    def start(self) -> Union[int, float]:
        return self._start

    @start.setter
    def start(self, start: Union[int, float]):
        assert is_valid_orientation(start), '<common.util/AngleInterval> start angle needs to be in interval [-2pi,2pi]'
        if hasattr(self, '_end'):
            assert start <= self._end, '<common.util/Interval> start of interval must be <= end, but start {} > end {}'.format(start,self._end)
        self._start = start

    @property
    def end(self) -> Union[int, float]:
        return self._end

    @end.setter
    def end(self, end: Union[int, float]):
        assert is_valid_orientation(end), '<common.util/AngleInterval> end angle needs to be in interval [-2pi,2pi]'
        if hasattr(self, '_start'):
            assert end >= self._start, '<common.util/Interval> start of interval must be <= end, but start {} > end {}'.format(self._start, end)
        self._end = end

