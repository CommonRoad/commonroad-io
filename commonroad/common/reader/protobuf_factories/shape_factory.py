from commonroad.common.reader.protobuf_factories.point_factory import PointFactory
from commonroad.geometry.obstacle_shapes.circle_obstacle_shape import CircleObstacleShape
from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.obstacle_shapes.polygon_obstacle_shape import PolygonObstacleShape
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.obstacle_shapes.semi_trailer_truck_shape import (
    SemiTrailerTruckShape,
    TrailerDimensions,
)
from commonroad.geometry.obstacle_shapes.truck_shape import TruckDimensions, TruckShape
from commonroad.scenario_definition.protobuf_format.generated_scripts import util_pb2


class ObstacleShapeFactory:
    @classmethod
    def create_from_message(cls, shape_msg: util_pb2.Shape) -> ObstacleShape:
        if shape_msg.HasField("rectangle"):
            shape = RectObstacleShapeFactory.create_from_message(shape_msg.rectangle)
        elif shape_msg.HasField("circle"):
            shape = CircleObstacleShapeFactory.create_from_message(shape_msg.circle)
        elif shape_msg.HasField("polygon"):
            shape = PolygonObstacleShapeFactory.create_from_message(shape_msg.polygon)
        elif shape_msg.HasField("truck_shape"):
            shape = TruckFactory.create_from_message(shape_msg.truck_shape)
        elif shape_msg.HasField("semi_trailer_truck_shape"):
            shape = SemiTrailerTruckFactory.create_from_message(shape_msg.semi_trailer_truck_shape)
        else:
            raise ValueError(f"Unknown shape type in message: {shape_msg}")

        return shape


class RectObstacleShapeFactory:
    @classmethod
    def create_from_message(cls, rect_msg: util_pb2.Rect) -> RectObstacleShape:
        return RectObstacleShape(width=rect_msg.width, length=rect_msg.length)


class CircleObstacleShapeFactory:
    @classmethod
    def create_from_message(cls, circle_msg: util_pb2.Circle) -> CircleObstacleShape:
        return CircleObstacleShape(radius=circle_msg.radius)


class PolygonObstacleShapeFactory:
    @classmethod
    def create_from_message(cls, polygon_msg: util_pb2.Polygon) -> PolygonObstacleShape:
        vertices = tuple(
            tuple(PointFactory.create_from_message(point_msg)) for point_msg in polygon_msg.vertices
        )
        return PolygonObstacleShape(vertices=vertices)


class TruckFactory:
    @classmethod
    def create_from_message(cls, truck_msg: util_pb2.TruckShape) -> TruckShape:
        return TruckShape(
            truck_dims=TruckDimsFactory.create_from_message(truck_msg.truck_dims),
            origin_x_shift=truck_msg.origin_x_shift,
        )


class TruckDimsFactory:
    @classmethod
    def create_from_message(cls, truck_dims_msg: util_pb2.TruckDims) -> TruckDimensions:
        return TruckDimensions(
            length=truck_dims_msg.length,
            width=truck_dims_msg.width,
            wheelbase=truck_dims_msg.wheelbase,
            dist_from_rear_to_rear_axle=truck_dims_msg.dist_from_rear_to_rear_axle,
            cabin_length=truck_dims_msg.cabin_length,
            dist_from_rear_axle_to_hitch=truck_dims_msg.dist_from_rear_axle_to_hitch,
        )


class SemiTrailerTruckFactory:
    @classmethod
    def create_from_message(
        cls, semi_trailer_truck_msg: util_pb2.SemiTrailerTruckShape
    ) -> SemiTrailerTruckShape:
        return SemiTrailerTruckShape(
            truck_shape=TruckFactory.create_from_message(semi_trailer_truck_msg.truck_shape),
            trailer_dims=TrailerDimsFactory.create_from_message(
                semi_trailer_truck_msg.trailer_dims
            ),
        )


class TrailerDimsFactory:
    @classmethod
    def create_from_message(cls, trailer_dims_msg: util_pb2.TrailerDims) -> TrailerDimensions:
        return TrailerDimensions(
            length=trailer_dims_msg.length,
            width=trailer_dims_msg.width,
            wheelbase=trailer_dims_msg.wheelbase,
            dist_from_front_to_hitch=trailer_dims_msg.dist_from_front_to_hitch,
        )
