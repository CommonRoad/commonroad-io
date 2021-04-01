"""Module for drawing obstacle icons."""
from typing import Union
import numpy as np
import matplotlib as mpl

import sys, pathlib
sys.path.append(str(pathlib.Path(__file__).resolve().parents[2]))


__author__ = "Simon Sagmeister"
__copyright__ = "TUM Cyber-Physical Systems Group"
# __credits__ = ["Tobias Geissenberger"]
__version__ = "2020.4"
# __maintainer__ = "Luis Gressenbuch"
# __email__ = "commonroad-i06@in.tum.de"
__status__ = "Development"

from commonroad.scenario.obstacle import ObstacleType


def _obstacle_icon_assignment():

    assign_dict = {
        ObstacleType.CAR: draw_car_icon,
        ObstacleType.PARKED_VEHICLE: draw_car_icon,
        ObstacleType.TAXI: draw_car_icon,
        }

    return assign_dict


def supported_icons():
    return list(_obstacle_icon_assignment().keys())


def get_obstacle_icon_patch(obstacle_type: ObstacleType,  pos_x: Union[int, float], pos_y: Union[int, float],
                            rotate: Union[int, float], length: Union[int, float],
                            width: Union[int, float],
                            zorder: float = 5, carcolor: str = '#ffffff', edgecolor='black', lw=0.5):
    draw_func = _obstacle_icon_assignment()[obstacle_type]
    patch = draw_func(pos_x, pos_y, rotate, length, width, zorder, carcolor, edgecolor, lw)
    return patch


def draw_car_icon(pos_x: Union[int, float], pos_y: Union[int, float],
                  rotate: Union[int, float], length: Union[int, float],
                  width: Union[int, float],
                  zorder: float = 5, carcolor: str = '#ffffff', edgecolor='black', lw=0.5):
    rotate = rotate + np.pi

    def reshape_and_addup(verts):
        verts = verts.reshape((int(len(verts) / 2), 2))
        for i in range(1, len(verts)):
            verts[i] = verts[i] + verts[i - 1]
        return verts

    def transform(vertices, pos_x, pos_y, rotate):
        vertices = np.asarray(np.bmat([vertices, np.ones((len(vertices), 1))]))
        tmat = np.array([[np.cos(rotate), -np.sin(rotate), pos_x],
                         [np.sin(rotate), np.cos(rotate), pos_y], [0, 0, 1]])
        scalemat = np.array(
                [[length/884.071996, 0, 0], [0, width/318.57232, 0],
                 [0, 0, 1]])
        centermat = np.array([[1, 0, -250], [0, 1, -889], [0, 0, 1]])
        tmat = tmat.dot(scalemat.dot(centermat))
        vertices = tmat.dot(vertices.transpose())
        vertices = np.array([[1, 0, 0], [0, 1, 0]]).dot(vertices).transpose()
        return vertices

    verts1 = np.array(
            [193.99383, 752.94359, -22.66602, 38, -9.33398, 52.66797, -2.60743,
             44.82812, -0.0586, 0, 0.0293, 0.91992, -0.0293, 0.91797, 0.0586, 0,
             2.60743, 44.82813, 9.33398, 52.66797, 22.66602, 38.00003, 59.33398,
             -7.334, 62, -10.666, -6, -50.00003, -2.65234, -68.41407, 2.65234,
             -68.41601, 6, -50, -62, -10.66602])

    verts2 = np.array(
            [715.99381, 768.27757, -101.332, 6.66602, 10.666, 62.66797, 3.3223,
             50.41797, -3.3223, 50.41796, -10.666, 62.66601, 101.332, 6.668, 22,
             -42.66799, 9.9824, -76.83594,  # 0.018,0,
             # -0.01,-0.24804,
             # 0.01,-0.24805,
             # -0.018,0,
             -9.9824, -76.83789, 0, 0])

    verts3 = np.array(
            [421.06111, 751.61113, 190.2667, 3.33333, 13.3333, 5.33334,
             -108.6666, 12.66666, -134, 0, -119.3334, -18.66666, 127.6456,
             -2.96473])

    verts4 = np.array(
            [271.32781, 712.14446, -6, -0.8, -16, 12, -14.8, 19.2, -4, 6, 20.4,
             0.4, 3.6, -4.4, 4.8, -2.8])
    verts5 = np.array(
            [191.32781, 753.94359, -99.999996, 11, -63, 18.5, -59, 38.5, -20,
             77, 20, 59.52734, 57, 36.49998, 65, 20.49999, 99.999996, 11.0001])

    verts6 = np.array([421.06111, 1027.399, 190.2667, -3.3333, 13.3333, -5.3333,
                       -108.6666, -12.6667, -134, 0, -119.3334, 18.6667,
                       127.6456, 2.9647])

    verts7 = np.array(
            [271.32781, 1066.8657, -6, 0.8, -16, -12, -14.8, -19.2, -4, -6,
             20.4, -0.4, 3.6, 4.4, 4.8, 2.8])

    verts8 = np.array(
            [389.79851, 728.34788, -343.652396, 10.16016, -68.666, 22.42969,
             -29.2558, 74.57031, -7.3164, 60.35742, -0.074, 0, 0.037, 0.76758,
             -0.037, 0.76758, 0.074, 0, 7.3164, 42.35937, 29.2558, 74.57031,
             68.666, 22.4278, 343.652396, 10.1621, 259.5859, -4.6192, 130.2539,
             -17.5527, 24.0196, -18.4766, 17.5527, -65.58788, 3.6953, -37.42773,
             0, -13.24414, -3.6953, -55.42774, -17.5527, -65.58984, -24.0196,
             -18.47656, -130.2539, -17.55274])

    verts1 = reshape_and_addup(verts1)  # vorderscheibe
    verts2 = reshape_and_addup(verts2)  # rueckscheibe
    verts3 = reshape_and_addup(verts3)  # seitenscheibe rechts
    verts4 = reshape_and_addup(verts4)  # rueckspiegel links
    verts5 = reshape_and_addup(verts5)  # motorhaube
    verts6 = reshape_and_addup(verts6)  # fenster rechts
    verts7 = reshape_and_addup(verts7)  # rueckspiegel rechts
    verts8 = reshape_and_addup(verts8)  # umriss

    verts1 = transform(verts1, pos_x, pos_y, rotate)
    verts2 = transform(verts2, pos_x, pos_y, rotate)
    verts3 = transform(verts3, pos_x, pos_y, rotate)
    verts4 = transform(verts4, pos_x, pos_y, rotate)
    verts5 = transform(verts5, pos_x, pos_y, rotate)
    verts6 = transform(verts6, pos_x, pos_y, rotate)
    verts7 = transform(verts7, pos_x, pos_y, rotate)
    verts8 = transform(verts8, pos_x, pos_y, rotate)

    windowcolor = edgecolor

    patch_list = [
            mpl.patches.Polygon(verts1, closed=True,
                                facecolor=windowcolor,
                                zorder=zorder + 1, lw=0),
            mpl.patches.Polygon(verts2, closed=True,
                                facecolor=windowcolor,
                                zorder=zorder + 1, lw=0),
            mpl.patches.Polygon(verts3, closed=True,
                                facecolor=windowcolor,
                                zorder=zorder + 1, lw=0),
            mpl.patches.Polygon(verts4, closed=True,
                                facecolor=windowcolor,
                                zorder=zorder + 1, lw=0),
            # mpl.patches.PathPatch(Path(verts5, closed=True), fill=False,
            # zorder=zorder + 1,
            #                       color='#000000', lw=lw),
            mpl.patches.Polygon(verts6, closed=True,
                                facecolor=windowcolor,
                                zorder=zorder + 1, lw=0),
            mpl.patches.Polygon(verts7, closed=True,
                                facecolor=windowcolor,
                                zorder=zorder + 1, lw=0),
            mpl.patches.Polygon(verts8, closed=True,
                                edgecolor=edgecolor,
                                facecolor=carcolor,
                                zorder=zorder, lw=lw)]
    return patch_list
