from typing import Tuple
from xml.etree import ElementTree

from commonroad.common.reader.xml_factories.point_factory import PointListFactory
from commonroad.geometry.obstacle_shapes.circle_obstacle_shape import CircleObstacleShape
from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.obstacle_shapes.polygon_obstacle_shape import PolygonObstacleShape
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.obstacle_shapes.semi_trailer_truck_shape import (
    SemiTrailerTruckShape,
    TrailerDimensions,
)
from commonroad.geometry.obstacle_shapes.truck_shape import TruckDimensions, TruckShape


class ObstacleShapeFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> ObstacleShape:
        shape_list = tuple(cls.read_shape(c) for c in list(xml_node))
        shape = cls._get_single_shape(shape_list)
        return shape

    @classmethod
    def read_shape(cls, xml_node: ElementTree.Element) -> ObstacleShape:
        tag_string = xml_node.tag
        if tag_string == "rectangle":
            return RectangleFactory.create_from_xml_node(xml_node)
        elif tag_string == "circle":
            return CircleFactory.create_from_xml_node(xml_node)
        elif tag_string == "polygon":
            return PolygonFactory.create_from_xml_node(xml_node)
        elif tag_string == "truckShape":
            return TruckFactory.create_from_xml_node(xml_node)
        elif tag_string == "semiTrailerTruckShape":
            return SemiTrailerTruckFactory.create_from_xml_node(xml_node)

    @classmethod
    def _get_single_shape(cls, shape_list: Tuple[ObstacleShape, ...]) -> ObstacleShape:
        if len(shape_list) > 1:
            raise ValueError("Shape groups are deprecated and not supported anymore.")
        else:
            return shape_list[0]


class RectangleFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> RectObstacleShape:
        length = float(xml_node.find("length").text)
        width = float(xml_node.find("width").text)
        if xml_node.find("originXShift") is not None:
            origin_x_shift = float(xml_node.find("originXShift").text)
        else:
            origin_x_shift = 0.0
        return RectObstacleShape(width=width, length=length, origin_x_shift=origin_x_shift)


class CircleFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> CircleObstacleShape:
        radius = float(xml_node.find("radius").text)
        return CircleObstacleShape(radius)


class PolygonFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> PolygonObstacleShape:
        vertices = PointListFactory.create_from_xml_node(xml_node)
        return PolygonObstacleShape(vertices=tuple(vertices))


class TruckDimsFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> TruckDimensions:
        length = float(xml_node.find("length").text)
        width = float(xml_node.find("width").text)
        wheelbase = float(xml_node.find("wheelbase").text)
        dist_from_rear_to_rear_axle = float(xml_node.find("distFromRearToRearAxle").text)
        cabin_length = float(xml_node.find("cabinLength").text)
        dist_from_rear_axle_to_hitch = float(xml_node.find("distFromRearAxleToHitch").text)

        return TruckDimensions(
            length=length,
            width=width,
            wheelbase=wheelbase,
            dist_from_rear_to_rear_axle=dist_from_rear_to_rear_axle,
            cabin_length=cabin_length,
            dist_from_rear_axle_to_hitch=dist_from_rear_axle_to_hitch,
        )


class TrailerDimsFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> TrailerDimensions:
        length = float(xml_node.find("length").text)
        width = float(xml_node.find("width").text)
        wheelbase = float(xml_node.find("wheelbase").text)
        dist_from_front_to_hitch = float(xml_node.find("distFromFrontToHitch").text)
        return TrailerDimensions(
            length=length,
            width=width,
            wheelbase=wheelbase,
            dist_from_front_to_hitch=dist_from_front_to_hitch,
        )


class TruckFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> TruckShape:
        truck_dims = TruckDimsFactory.create_from_xml_node(xml_node.find("truckDims"))
        origin_x_shift = float(xml_node.find("originXShift").text)
        return TruckShape(truck_dims=truck_dims, origin_x_shift=origin_x_shift)


class SemiTrailerTruckFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> SemiTrailerTruckShape:
        truck = TruckFactory.create_from_xml_node(xml_node.find("truckShape"))
        trailer_dims = TrailerDimsFactory.create_from_xml_node(xml_node.find("trailerDims"))
        return SemiTrailerTruckShape(truck_shape=truck, trailer_dims=trailer_dims)
