from typing import List

import numpy as np
from lxml import etree

from commonroad.common.writer.xml_nodes.float_to_str import float_to_str
from commonroad.common.writer.xml_nodes.point import Point
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.occupancy_group import OccupancyGroup
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy


class OccupancyXMLNode:
    @classmethod
    def create_node(cls, occ) -> List[etree.Element]:
        """
        Create XML-Node for a occ
        :param occ: occ for creating a node
        :return: node
        """
        assert isinstance(occ, Occupancy)
        if isinstance(occ, OccupancyGroup):
            occ_node_list = []
            for s in occ.occupancies:
                occ_node_list.append(cls._create_single_element(s))
        else:
            occ_node = cls._create_single_element(occ)
            occ_node_list = [occ_node]
        return occ_node_list

    @classmethod
    def _create_single_element(cls, occ: Occupancy) -> etree.Element:
        """
        Create XML-Node for a single occ element
        :param occ: occ for creating a node
        :return: node
        """
        if isinstance(occ, RectOccupancy):
            node = RectOccupancyXMLNode.create_rectangle_node(occ)
        elif isinstance(occ, CircleOccupancy):
            node = CircleOccupancyXMLNode.create_circle_node(occ)
        elif isinstance(occ, PolygonOccupancy):
            node = PolygonOccupancyXMLNode.create_polygon_node(occ)
        else:
            raise TypeError(
                "<OccupancyXMLNode/_create_single_element> Expected type Polygon, Circle or Rectangle but got %s"
                % (type(occ))
            )
        return node


class RectOccupancyXMLNode:
    @classmethod
    def create_rectangle_node(cls, rect_occ: RectOccupancy) -> etree.Element:
        """
        Create XML-Node for a rect_occ
        :param rect_occ: rect_occ for creating a node
        :return: node
        """
        rect_occ_node = etree.Element("rectangle")
        length_node = etree.Element("length")
        length_node.text = str(rect_occ.length)
        rect_occ_node.append(length_node)

        width_node = etree.Element("width")
        width_node.text = str(rect_occ.width)
        rect_occ_node.append(width_node)

        orientation_node = etree.Element("orientation")
        orientation_node.text = str(np.float64(rect_occ.orientation))
        rect_occ_node.append(orientation_node)

        center_node = etree.Element("center")
        x_node = etree.Element("x")
        x_node.text = float_to_str(np.float64(rect_occ.center.x))
        center_node.append(x_node)
        y_node = etree.Element("y")
        y_node.text = float_to_str(np.float64(rect_occ.center.y))
        center_node.append(y_node)
        rect_occ_node.append(center_node)
        return rect_occ_node


class CircleOccupancyXMLNode:
    @classmethod
    def create_circle_node(cls, circle_occ: CircleOccupancy) -> etree.Element:
        """
        Create XML-Node for a circle_occ
        :param circle_occ: circle_occ for creating a node
        :return: node
        """
        circle_occ_node = etree.Element("circle")

        radius_node = etree.Element("radius")
        radius_node.text = str(np.float64(circle_occ.radius))
        circle_occ_node.append(radius_node)

        center_node = etree.Element("center")
        x_node = etree.Element("x")
        x_node.text = float_to_str(np.float64(circle_occ.center.x))
        center_node.append(x_node)
        y_node = etree.Element("y")
        y_node.text = float_to_str(np.float64(circle_occ.center.y))
        center_node.append(y_node)
        circle_occ_node.append(center_node)
        return circle_occ_node


class PolygonOccupancyXMLNode:
    @classmethod
    def create_polygon_node(cls, polygon_occ: PolygonOccupancy) -> etree.Element:
        """
        Create XML-Node for a polygon_occ
        :param polygon_occ: polygon_occ for creating a node
        :return: node
        """
        polygon_occ_node = etree.Element("polygon")
        for p in polygon_occ.vertices:
            polygon_occ_node.append(Point(p[0], p[1]).create_node())
        return polygon_occ_node
