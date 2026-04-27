from commonroad.common.protobuf.common import util_pb2
from commonroad.common.writer.protobuf_messages.point_message import PointMessage
from commonroad.geometry.obstacle_shapes.circle_obstacle_shape import CircleObstacleShape
from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.obstacle_shapes.polygon_obstacle_shape import PolygonObstacleShape
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.obstacle_shapes.semi_trailer_truck_shape import (
    SemiTrailerTruckShape,
    TrailerDimensions,
)
from commonroad.geometry.obstacle_shapes.truck_shape import TruckDimensions, TruckShape


class ShapeMessage:
    @classmethod
    def create_message(cls, shape: ObstacleShape) -> util_pb2.Shape:
        shape_msg = util_pb2.Shape()

        if isinstance(shape, RectObstacleShape):
            shape_msg.rectangle.CopyFrom(RectMessage.create_message(shape))
        elif isinstance(shape, CircleObstacleShape):
            shape_msg.circle.CopyFrom(CircleMessage.create_message(shape))
        elif isinstance(shape, PolygonObstacleShape):
            shape_msg.polygon.CopyFrom(PolygonMessage.create_message(shape))
        elif isinstance(shape, TruckShape):
            shape_msg.truck_shape.CopyFrom(TruckMessage.create_message(shape))
        elif isinstance(shape, SemiTrailerTruckShape):
            shape_msg.semi_trailer_truck_shape.CopyFrom(
                SemiTrailerTruckMessage.create_message(shape)
            )
        else:
            raise ValueError(f"Unsupported shape type: {type(shape)}")

        return shape_msg


class RectMessage:
    @classmethod
    def create_message(cls, rect: RectObstacleShape) -> util_pb2.Rect:
        rect_msg = util_pb2.Rect()

        rect_msg.length = rect.length
        rect_msg.width = rect.width

        return rect_msg


class CircleMessage:
    @classmethod
    def create_message(cls, circle: CircleObstacleShape) -> util_pb2.Circle:
        circle_msg = util_pb2.Circle()

        circle_msg.radius = circle.radius

        return circle_msg


class PolygonMessage:
    @classmethod
    def create_message(cls, polygon: PolygonObstacleShape) -> util_pb2.Polygon:
        polygon_msg = util_pb2.Polygon()

        for vertex in polygon.vertices:
            point_msg = PointMessage.create_message(vertex)
            polygon_msg.vertices.append(point_msg)

        return polygon_msg


class TruckMessage:
    @classmethod
    def create_message(cls, truck: TruckShape) -> util_pb2.TruckShape:
        truck_msg = util_pb2.TruckShape()
        truck_msg.truck_dims.CopyFrom(TruckDimsMessage.create_message(truck.truck_dims))
        truck_msg.origin_x_shift = truck.origin_x_shift

        return truck_msg


class TruckDimsMessage:
    @classmethod
    def create_message(cls, truck_dims: TruckDimensions) -> util_pb2.TruckDims:
        truck_dims_msg = util_pb2.TruckDims()

        truck_dims_msg.length = truck_dims.length
        truck_dims_msg.width = truck_dims.width
        truck_dims_msg.wheelbase = truck_dims.wheelbase
        truck_dims_msg.dist_from_rear_to_rear_axle = truck_dims.dist_from_rear_to_rear_axle
        truck_dims_msg.cabin_length = truck_dims.cabin_length
        truck_dims_msg.dist_from_rear_axle_to_hitch = truck_dims.dist_from_rear_axle_to_hitch

        return truck_dims_msg


class SemiTrailerTruckMessage:
    @classmethod
    def create_message(
        cls, semi_trailer_truck: SemiTrailerTruckShape
    ) -> util_pb2.SemiTrailerTruckShape:
        semi_trailer_truck_msg = util_pb2.SemiTrailerTruckShape()

        semi_trailer_truck_msg.truck_shape.CopyFrom(
            TruckMessage.create_message(semi_trailer_truck.truck_shape)
        )
        semi_trailer_truck_msg.trailer_dims.CopyFrom(
            TrailerDimsMessage.create_message(semi_trailer_truck.trailer_dims)
        )

        return semi_trailer_truck_msg


class TrailerDimsMessage:
    @classmethod
    def create_message(cls, trailer_dims: TrailerDimensions) -> util_pb2.TrailerDims:
        trailer_dims_msg = util_pb2.TrailerDims()

        trailer_dims_msg.length = trailer_dims.length
        trailer_dims_msg.width = trailer_dims.width
        trailer_dims_msg.wheelbase = trailer_dims.wheelbase
        trailer_dims_msg.dist_from_front_to_hitch = trailer_dims.dist_from_front_to_hitch

        return trailer_dims_msg
