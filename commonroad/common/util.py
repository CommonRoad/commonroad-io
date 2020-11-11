from typing import Union, Tuple

from commonroad.common import validity
from commonroad.common.validity import *

__author__ = "Stefanie Manzinger, Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "2020.3"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


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


class Interval:
    def __init__(self, start: Union[int, float], end: Union[int, float]):
        self.start = start
        self.end = end

    def __iter__(self):
        yield self._start
        yield self._end

    def __truediv__(self, other) -> 'Interval':
        if other > 0.0:
            return type(self)(self._start/other, self._end/other)
        else:
            return type(self)(self._end / other, self._start / other)

    def __mul__(self, other: Union[int,float]) -> 'Interval':
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

    def contains(self, other: Union[float, 'Interval']) -> bool:
        if type(other) is Interval:
            return self.start <= other.start and other.end <= self.end
        else:
            return self.start <= other <= self.end

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
        start, end = make_valid_orientation_interval(start,end)
        assert end - start < TWO_PI, '<common.util/AngleInterval> Interval must not be |start-end| > 2pi'
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

    def intersect(self, other: 'Interval'):
        raise NotImplementedError()

