import shapely

from commonroad.common.reader.protobuf_factories.point_factory import PointFactory
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.occupancy_group import OccupancyGroup
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.scenario_definition.protobuf_format.generated_scripts import util_pb2


class OccupancyFactory:
    @classmethod
    def create_from_message(cls, occ_msg: util_pb2.Occupancy) -> Occupancy:
        if occ_msg.HasField("rectangle"):
            shape = RectOccupancyFactory.create_from_message(occ_msg.rectangle)
        elif occ_msg.HasField("circle"):
            shape = CircleOccupancyFactory.create_from_message(occ_msg.circle)
        elif occ_msg.HasField("polygon"):
            shape = PolygonOccupancyFactory.create_from_message(occ_msg.polygon)
        else:
            shape = OccupancyGroupFactory.create_from_message(occ_msg.shape_group)

        return shape


class OccupancyGroupFactory:
    @classmethod
    def create_from_message(cls, occ_group_msg: util_pb2.OccupancyGroup) -> OccupancyGroup:
        occs = (OccupancyFactory.create_from_message(occ_msg) for occ_msg in occ_group_msg.shapes)
        return OccupancyGroup(occupancies=tuple(occs))


class RectOccupancyFactory:
    @classmethod
    def create_from_message(cls, rect_occ_msg: util_pb2.RectOccupancy) -> RectOccupancy:
        width = rect_occ_msg.width
        length = rect_occ_msg.length
        center = PointFactory.create_from_message(rect_occ_msg.center)
        orientation = rect_occ_msg.orientation

        return RectOccupancy(
            rect_center=shapely.Point(center), width=width, length=length, orientation=orientation
        )


class CircleOccupancyFactory:
    @classmethod
    def create_from_message(cls, circle_occ_msg: util_pb2.CircleOccupancy) -> CircleOccupancy:
        radius = circle_occ_msg.radius
        center = PointFactory.create_from_message(circle_occ_msg.center)
        return CircleOccupancy(radius=radius, circle_center=shapely.Point(center))


class PolygonOccupancyFactory:
    @classmethod
    def create_from_message(cls, polygon_occ_msg: util_pb2.PolygonOccupancy) -> PolygonOccupancy:
        vertices = tuple(
            PointFactory.create_from_message(point_msg) for point_msg in polygon_occ_msg.vertices
        )
        return PolygonOccupancy(shapely.Polygon(vertices))
