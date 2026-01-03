from typing import List

import numpy as np
from lxml import etree

from commonroad.common.writer.xml_nodes.point import Point
from commonroad.geometry.obstacle_shapes.circle_obstacle_shape import CircleObstacleShape
from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.obstacle_shapes.polygon_obstacle_shape import PolygonObstacleShape
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.obstacle_shapes.semi_trailer_truck_shape import (
    SemiTrailerTruckShape,
    TrailerDimensions,
)
from commonroad.geometry.obstacle_shapes.truck_shape import TruckDimensions, TruckShape


class ShapeXMLNode:
    @classmethod
    def create_node(cls, shape) -> List[etree.Element]:
        """
        Create XML-Node for a shape
        :param shape: shape for creating a node
        :return: node
        """
        assert isinstance(shape, ObstacleShape)
        shape_node = cls._create_single_element(shape)
        shape_node_list = [shape_node]
        return shape_node_list

    @classmethod
    def _create_single_element(cls, shape: ObstacleShape) -> etree.Element:
        """
        Create XML-Node for a single shape element
        :param shape: shape for creating a node
        :return: node
        """
        if isinstance(shape, RectObstacleShape):
            node = RectangleXMLNode.create_rectangle_node(shape)
        elif isinstance(shape, CircleObstacleShape):
            node = CircleXMLNode.create_circle_node(shape)
        elif isinstance(shape, PolygonObstacleShape):
            node = PolygonXMLNode.create_polygon_node(shape)
        elif isinstance(shape, TruckShape):
            node = TruckXMLNode.create_truck_node(shape)
        elif isinstance(shape, SemiTrailerTruckShape):
            node = SemiTrailerTruckXMLNode.create_semi_trailer_truck_node(shape)
        else:
            raise TypeError(
                "<ShapeXMLNode/_create_single_element> Expected type Polygon, Circle or Rectangle but got %s"
                % (type(shape))
            )
        return node


class RectangleXMLNode:
    @classmethod
    def create_rectangle_node(cls, rectangle: RectObstacleShape) -> etree.Element:
        """
        Create XML-Node for a rectangle
        :param rectangle: rectangle for creating a node
        :return: node
        """
        rectangle_node = etree.Element("rectangle")

        length_node = etree.Element("length")
        length_node.text = str(rectangle.length)
        rectangle_node.append(length_node)

        width_node = etree.Element("width")
        width_node.text = str(rectangle.width)
        rectangle_node.append(width_node)

        origin_x_shift_node = etree.Element("originXShift")
        origin_x_shift_node.text = str(rectangle.origin_x_shift)
        rectangle_node.append(origin_x_shift_node)

        return rectangle_node


class CircleXMLNode:
    @classmethod
    def create_circle_node(cls, circle: CircleObstacleShape) -> etree.Element:
        """
        Create XML-Node for a circle
        :param circle: circle for creating a node
        :return: node
        """
        circle_node = etree.Element("circle")

        radius_node = etree.Element("radius")
        radius_node.text = str(np.float64(circle.radius))
        circle_node.append(radius_node)
        return circle_node


class PolygonXMLNode:
    @classmethod
    def create_polygon_node(cls, polygon: PolygonObstacleShape) -> etree.Element:
        """
        Create XML-Node for a polygon
        :param polygon: polygon for creating a node
        :return: node
        """
        polygon_node = etree.Element("polygon")
        for p in polygon.vertices:
            polygon_node.append(Point(p[0], p[1]).create_node())
        return polygon_node


class TruckDimsXMLNode:
    @classmethod
    def create_truck_dims_node(cls, truck_dims: TruckDimensions) -> etree.Element:
        truck_dims_node = etree.Element("truckDims")

        length_node = etree.Element("length")
        length_node.text = str(np.float64(truck_dims.length))
        truck_dims_node.append(length_node)

        width_node = etree.Element("width")
        width_node.text = str(np.float64(truck_dims.width))
        truck_dims_node.append(width_node)

        wheelbase_node = etree.Element("wheelbase")
        wheelbase_node.text = str(np.float64(truck_dims.wheelbase))
        truck_dims_node.append(wheelbase_node)

        dist_from_rear_to_rear_axle_node = etree.Element("distFromRearToRearAxle")
        dist_from_rear_to_rear_axle_node.text = str(
            np.float64(truck_dims.dist_from_rear_to_rear_axle)
        )
        truck_dims_node.append(dist_from_rear_to_rear_axle_node)

        cabin_length_node = etree.Element("cabinLength")
        cabin_length_node.text = str(np.float64(truck_dims.cabin_length))
        truck_dims_node.append(cabin_length_node)

        truck_dist_from_rear_axle_to_hitch_node = etree.Element("distFromRearAxleToHitch")
        truck_dist_from_rear_axle_to_hitch_node.text = str(
            np.float64(truck_dims.dist_from_rear_axle_to_hitch)
        )
        truck_dims_node.append(truck_dist_from_rear_axle_to_hitch_node)

        return truck_dims_node


class TrailerDimsXMLNode:
    @classmethod
    def create_trailer_dims_node(cls, trailer_dims: TrailerDimensions) -> etree.Element:
        trailer_dims_node = etree.Element("trailerDims")

        length_node = etree.Element("length")
        length_node.text = str(np.float64(trailer_dims.length))
        trailer_dims_node.append(length_node)

        width_node = etree.Element("width")
        width_node.text = str(np.float64(trailer_dims.width))
        trailer_dims_node.append(width_node)

        wheelbase_node = etree.Element("wheelbase")
        wheelbase_node.text = str(np.float64(trailer_dims.wheelbase))
        trailer_dims_node.append(wheelbase_node)

        dist_from_front_to_hitch_node = etree.Element("distFromFrontToHitch")
        dist_from_front_to_hitch_node.text = str(np.float64(trailer_dims.dist_from_front_to_hitch))
        trailer_dims_node.append(dist_from_front_to_hitch_node)

        return trailer_dims_node


class TruckXMLNode:
    @classmethod
    def create_truck_node(cls, truck: TruckShape) -> etree.Element:
        res_node = etree.Element("truckShape")

        truck_dims_node = TruckDimsXMLNode.create_truck_dims_node(truck.truck_dims)
        res_node.append(truck_dims_node)

        origin_node = etree.Element("originXShift")
        origin_node.text = str(np.float64(truck.origin_x_shift))
        res_node.append(origin_node)

        return res_node


class SemiTrailerTruckXMLNode:
    @classmethod
    def create_semi_trailer_truck_node(
        cls, semi_trailer_truck: SemiTrailerTruckShape
    ) -> etree.Element:
        res_node = etree.Element("semiTrailerTruckShape")

        truck_node = TruckXMLNode.create_truck_node(semi_trailer_truck.truck_shape)
        res_node.append(truck_node)

        trailer_dims_node = TrailerDimsXMLNode.create_trailer_dims_node(
            semi_trailer_truck.trailer_dims
        )
        res_node.append(trailer_dims_node)

        return res_node
