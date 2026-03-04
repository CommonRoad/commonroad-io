from __future__ import annotations

from dataclasses import dataclass

from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.scenario.state import TraceState


@dataclass(frozen=True)
class TruckShape(ObstacleShape):
    """represents the shape of a truck. The state's position refers to the truck's rear axle"""

    truck_dims: TruckDimensions
    # Shift of the origin in x-direction relative to the truck's center.
    # Defaults to the rear axle position
    origin_x_shift: float = None

    def __post_init__(self):
        if self.origin_x_shift is None:
            object.__setattr__(self, "origin_x_shift", self.truck_dims.default_origin_x_shift())

    @property
    def total_length(self):
        return self.truck_dims.length

    @staticmethod
    def create_default() -> TruckShape:
        return TruckShape(
            truck_dims=TruckDimensions.create_default(),
        )

    def scale(self, length_scale: float) -> TruckShape:
        return TruckShape(
            truck_dims=self.truck_dims.scale(length_scale),
            origin_x_shift=length_scale * self.origin_x_shift,
        )

    @staticmethod
    def with_length(length: float) -> TruckShape:
        """
        Create a :TruckShape with length :length by scaling a default truck shape.

        :param length: desired length of the truck shape
        """
        default_truck = TruckShape.create_default()
        length_scale = length / default_truck.total_length
        return default_truck.scale(length_scale)

    def compute_occupancy_for_state(self, state: TraceState) -> Occupancy:
        truck_rect = RectObstacleShape(
            width=self.truck_dims.width,
            length=self.truck_dims.length,
            origin_x_shift=self.origin_x_shift,
        )
        return truck_rect.compute_occupancy_for_state(state)

    def compute_occupancy_for_state_set(self, state: TraceState) -> Occupancy:
        raise NotImplementedError("TruckTrailer does not support transformation to state set")


@dataclass(frozen=True)
class TruckDimensions:
    length: float
    width: float
    wheelbase: float
    dist_from_rear_to_rear_axle: float
    cabin_length: float
    dist_from_rear_axle_to_hitch: float

    def default_origin_x_shift(self) -> float:
        return -(self.length / 2 - self.dist_from_rear_to_rear_axle)

    @property
    def truck_dist_from_front_to_front_axle(self) -> float:
        return self.length - self.wheelbase - self.dist_from_rear_to_rear_axle

    @staticmethod
    def create_default() -> TruckDimensions:
        return TruckDimensions(
            length=5.1,
            width=2.55,
            wheelbase=3.6,
            dist_from_rear_to_rear_axle=0.5,
            cabin_length=2.5,
            dist_from_rear_axle_to_hitch=0.45,
        )

    def scale(self, length_scale: float) -> TruckDimensions:
        return TruckDimensions(
            length=length_scale * self.length,
            width=self.width,
            wheelbase=length_scale * self.wheelbase,
            dist_from_rear_to_rear_axle=length_scale * self.dist_from_rear_to_rear_axle,
            cabin_length=length_scale * self.cabin_length,
            dist_from_rear_axle_to_hitch=length_scale * self.dist_from_rear_axle_to_hitch,
        )
