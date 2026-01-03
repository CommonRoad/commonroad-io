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
    """represents the shape of a truck-trailer-system. The state's position refers to the truck's rear axle"""

    truck_shape: TruckShape
    trailer_dims: TrailerDimensions

    @property
    def total_length(self) -> float:
        return (
            self.truck_shape.total_length
            + self.trailer_dims.length
            - self.truck_shape.truck_dims.dist_from_rear_axle_to_hitch
            - self.truck_shape.truck_dims.dist_from_rear_to_rear_axle
            - self.trailer_dims.dist_from_front_to_hitch
        )

    @staticmethod
    def create_default() -> SemiTrailerTruckShape:
        return SemiTrailerTruckShape(
            truck_shape=TruckShape.create_default(),
            trailer_dims=TrailerDimensions.create_default(),
        )

    @staticmethod
    def with_length(length: float) -> SemiTrailerTruckShape:
        """
        Create a :SemiTrailerTruckShape with total length :length by scaling a default semi-trailer truck shape.

        :param length: desired total length of the semi-trailer truck shape
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

    def compute_occupancy_for_state(self, state: TraceState) -> Occupancy:
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
    length: float
    width: float
    wheelbase: float
    dist_from_front_to_hitch: float

    @staticmethod
    def create_default() -> TrailerDimensions:
        return TrailerDimensions(
            length=13.6,
            width=2.55,
            wheelbase=7.8,
            dist_from_front_to_hitch=0.9,
        )

    def scale(self, length_scale: float) -> TrailerDimensions:
        return TrailerDimensions(
            length=length_scale * self.length,
            width=self.width,
            wheelbase=length_scale * self.wheelbase,
            dist_from_front_to_hitch=length_scale * self.dist_from_front_to_hitch,
        )
