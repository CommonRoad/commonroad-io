from typing import Tuple
from xml.etree import ElementTree

import numpy as np
import shapely

from commonroad.common.reader.xml_factories.point_factory import PointFactory, PointListFactory
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.occupancy_group import OccupancyGroup
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy


class OccupancyFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> Occupancy:
        occ_list = tuple(cls._read_single_occ(c) for c in list(xml_node))
        occ = cls._create_occ_group_if_needed(occ_list)
        return occ

    @classmethod
    def _read_single_occ(cls, xml_node: ElementTree.Element) -> Occupancy:
        tag_string = xml_node.tag
        if tag_string == "rectangle":
            return RectOccupancyFactory.create_from_xml_node(xml_node)
        elif tag_string == "circle":
            return CircleOccupancyFactory.create_from_xml_node(xml_node)
        elif tag_string == "polygon":
            return PolygonOccupancyFactory.create_from_xml_node(xml_node)

    @classmethod
    def _create_occ_group_if_needed(cls, occ_list: Tuple[Occupancy, ...]) -> Occupancy:
        if len(occ_list) > 1:
            return OccupancyGroup(occ_list)
        else:
            return occ_list[0]


class RectOccupancyFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> RectOccupancy:
        length = float(xml_node.find("length").text)
        width = float(xml_node.find("width").text)
        if xml_node.find("orientation") is not None:
            orientation = float(xml_node.find("orientation").text)
        else:
            orientation = 0.0
        if xml_node.find("center") is not None:
            center = PointFactory.create_from_xml_node(xml_node.find("center"))
        else:
            center = np.array([0.0, 0.0])
        return RectOccupancy(
            rect_center=shapely.Point(center), width=width, length=length, orientation=orientation
        )


class CircleOccupancyFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> CircleOccupancy:
        radius = float(xml_node.find("radius").text)
        if xml_node.find("center") is not None:
            center = PointFactory.create_from_xml_node(xml_node.find("center"))
        else:
            center = np.array([0.0, 0.0])
        return CircleOccupancy(radius=radius, circle_center=shapely.Point(center))


class PolygonOccupancyFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> PolygonOccupancy:
        vertices = PointListFactory.create_from_xml_node(xml_node)
        return PolygonOccupancy(polygon=shapely.Polygon(vertices))
