from commonroad.common.protobuf.common import util_pb2
from commonroad.common.writer.protobuf_messages.point_message import PointMessage
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.occupancy_group import OccupancyGroup
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy


class OccupancyMessage:
    @classmethod
    def create_message(cls, occ: Occupancy) -> util_pb2.Occupancy:
        occ_msg = util_pb2.Occupancy()

        if isinstance(occ, RectOccupancy):
            occ_msg.rectangle.CopyFrom(RectOccupancyMessage.create_message(occ))
        elif isinstance(occ, CircleOccupancy):
            occ_msg.circle.CopyFrom(CircleOccupancyMessage.create_message(occ))
        elif isinstance(occ, PolygonOccupancy):
            occ_msg.polygon.CopyFrom(PolygonOccupancyMessage.create_message(occ))
        elif isinstance(occ, OccupancyGroup):
            occ_msg.shape_group.CopyFrom(OccupancyGroupMessage.create_message(occ))

        return occ_msg


class RectOccupancyMessage:
    @classmethod
    def create_message(cls, rect_occ: RectOccupancy) -> util_pb2.RectOccupancy:
        rect_occ_msg = util_pb2.RectOccupancy()

        rect_occ_msg.length = rect_occ.length
        rect_occ_msg.width = rect_occ.width

        point_msg = PointMessage.create_message((rect_occ.center.x, rect_occ.center.y))
        rect_occ_msg.center.CopyFrom(point_msg)

        rect_occ_msg.orientation = rect_occ.orientation

        return rect_occ_msg


class CircleOccupancyMessage:
    @classmethod
    def create_message(cls, circle_occ: CircleOccupancy) -> util_pb2.CircleOccupancy:
        circle_occ_msg = util_pb2.CircleOccupancy()

        circle_occ_msg.radius = circle_occ.radius

        point_msg = PointMessage.create_message((circle_occ.center.x, circle_occ.center.y))
        circle_occ_msg.center.CopyFrom(point_msg)

        return circle_occ_msg


class PolygonOccupancyMessage:
    @classmethod
    def create_message(cls, polygon_occ: PolygonOccupancy) -> util_pb2.PolygonOccupancy:
        polygon_msg = util_pb2.PolygonOccupancy()

        for vertex in polygon_occ.vertices:
            point_msg = PointMessage.create_message(vertex)
            polygon_msg.vertices.append(point_msg)

        return polygon_msg


class OccupancyGroupMessage:
    @classmethod
    def create_message(cls, occ_group: OccupancyGroup) -> util_pb2.OccupancyGroup:
        shape_group_msg = util_pb2.OccupancyGroup()

        for shape in occ_group.occupancies:
            shape_msg = OccupancyMessage.create_message(shape)
            shape_group_msg.shapes.append(shape_msg)

        return shape_group_msg
