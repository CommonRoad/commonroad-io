from xml.etree import ElementTree

import numpy as np


class PointFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> np.ndarray:
        x = float(xml_node.find("x").text)
        y = float(xml_node.find("y").text)
        if xml_node.find("z") is None:
            return np.array([x, y])
        else:
            z = float(xml_node.find("z").text)
            return np.array([x, y, z])


class PointListFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> np.ndarray:
        point_list = []
        for point_node in xml_node.findall("point"):
            point_list.append(PointFactory.create_from_xml_node(point_node))
        return np.array(point_list)
