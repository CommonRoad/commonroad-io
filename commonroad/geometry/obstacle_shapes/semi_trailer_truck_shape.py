from __future__ import annotations

import warnings
from dataclasses import dataclass

import shapely

from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.obstacle_shapes.truck_shape import TruckShape
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.occupancy_group import OccupancyGroup
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.scenario.state import TraceState


@dataclass(frozen=True)
class SemiTrailerTruckShape(ObstacleShape):
    """Shape of a semi-trailer truck, consisting of a truck and a trailer.

    :param truck_shape: shape of the truck part of the semi-trailer truck
    :type truck_shape: :class:`TruckShape`
    :param trailer_dims: dimensions of the trailer part of the semi-trailer truck
    :type trailer_dims: :class:`TrailerDimensions`
    """

    truck_shape: TruckShape
    trailer_dims: TrailerDimensions

    @property
    def total_length(self) -> float:
        """
        Length of the whole sytem.
        """
        return (
            self.truck_shape.total_length
            + self.trailer_dims.length
            - self.truck_shape.truck_dims.dist_from_rear_axle_to_hitch
            - self.truck_shape.truck_dims.dist_from_rear_to_rear_axle
            - self.trailer_dims.dist_from_front_to_hitch
        )

    @staticmethod
    def create_default() -> SemiTrailerTruckShape:
        """Creates a :class:`SemiTrailerTruckShape` with default dimensions.

        :return: a :class:`SemiTrailerTruckShape` object with default dimensions
        """
        return SemiTrailerTruckShape(
            truck_shape=TruckShape.create_default(),
            trailer_dims=TrailerDimensions.create_default(),
        )

    @staticmethod
    def with_length(length: float) -> SemiTrailerTruckShape:
        """
        Create a :class:`SemiTrailerTruckShape` with the given total length by scaling a default
        semi-trailer truck shape.

        :param length: desired total length of the semi-trailer truck shape
        :return: a :class:`SemiTrailerTruckShape` object with the given total length
        """
        default_truck = SemiTrailerTruckShape.create_default()
        length_scale = length / default_truck.total_length
        return SemiTrailerTruckShape(
            default_truck.truck_shape.scale(length_scale),
            default_truck.trailer_dims.scale(length_scale),
        )

    @property
    def hitch_shift_from_origin(self) -> float:
        default_truck_x_shift = self.truck_shape.truck_dims.default_origin_x_shift()
        return self.truck_shape.truck_dims.dist_from_rear_axle_to_hitch + (
            default_truck_x_shift - self.truck_shape.origin_x_shift
        )

    def compute_occupancy_for_state(self, state: TraceState) -> OccupancyGroup:
        """
        Compute the occupancy of the semi-trailer truck for a given state, considering
        the position, orientation, and hitch angle of the state.

        :param state: the state for which to compute the occupancy
        :return: the occupancy group of the semi-trailer truck for the given state, consisting
            of two rectangles: one for the truck and one for the trailer.
        """

        truck_rect = self.truck_shape.compute_occupancy_for_state(state)

        # Trailer: the origin is the hitch point first for rotating by the hitch angle...
        trailer_center_x = -(
            self.trailer_dims.length / 2 - self.trailer_dims.dist_from_front_to_hitch
        )
        trailer_rect = RectOccupancy(
            rect_center=shapely.Point(trailer_center_x, 0.0),
            width=self.trailer_dims.width,
            length=self.trailer_dims.length,
            orientation=0.0,
        )

        if not hasattr(state, "hitch_angle"):
            hitch_angle = 0.0
            warnings.warn("State does not have attribute 'hitch_angle'. Assuming hitch_angle = 0.0")
        else:
            hitch_angle = state.hitch_angle
        # ...we first rotate the trailer by the hitch angle:
        trailer_rect = trailer_rect.rotate(hitch_angle)
        # ...then we shift the trailer such that the truck's origin is the origin:
        trailer_rect = trailer_rect.translate(self.hitch_shift_from_origin, 0.0)

        # Now we rotate the trailer by the orientation:
        trailer_rect = trailer_rect.rotate(state.orientation)

        # Lastly, we translate the trailer to the global position:
        xoff, yoff = state.position
        trailer_rect = trailer_rect.translate(xoff, yoff)

        return OccupancyGroup(occupancies=(truck_rect, trailer_rect))

    def compute_occupancy_for_state_set(self, state: TraceState) -> Occupancy:
        raise NotImplementedError("TruckTrailer does not support transformation to state set")


@dataclass(frozen=True)
class TrailerDimensions:
    """Dimensions of the trailer part of a semi-trailer truck.

    :param length: length of the trailer
    :type length: float
    :param width: width of the trailer
    :type width: float
    :param wheelbase: distance between the front and rear axles of the trailer
    :type wheelbase: float
    :param dist_from_front_to_hitch: distance from the front of the trailer to the hitch point
    :type dist_from_front_to_hitch: float
    """

    length: float
    width: float
    wheelbase: float
    dist_from_front_to_hitch: float

    @staticmethod
    def create_default() -> TrailerDimensions:
        """Creates a :class:`TrailerDimensions` object with default dimensions.

        :return: a :class:`TrailerDimensions` object with default dimensions
        """
        return TrailerDimensions(
            length=13.6,
            width=2.55,
            wheelbase=7.8,
            dist_from_front_to_hitch=0.9,
        )

    def scale(self, length_scale: float) -> TrailerDimensions:
        """Scales the trailer dimensions by a given length scale.

        :param length_scale: the scale factor for the length dimensions
        :return: a new :class:`TrailerDimensions` object with scaled dimensions
        """
        return TrailerDimensions(
            length=length_scale * self.length,
            width=self.width,
            wheelbase=length_scale * self.wheelbase,
            dist_from_front_to_hitch=length_scale * self.dist_from_front_to_hitch,
        )
