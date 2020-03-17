import os
from collections import defaultdict
from typing import Dict, Callable, Tuple, Union, Any, Set
import commonroad.geometry.shape
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.image as mplimage
import matplotlib.cm as cm
import matplotlib.patches as patches
import matplotlib.collections as collections
import PIL.Image
from matplotlib.cbook import get_sample_data
# import pylustrator

import commonroad.prediction.prediction
import commonroad.scenario.obstacle
import commonroad.visualization.draw_dispatch_cr
from commonroad.common.util import Interval
from commonroad.geometry.shape import *
from commonroad.scenario.intersection import Intersection
from commonroad.scenario.traffic_sign import TrafficSign, TrafficSignIDGermany, TrafficLight, TrafficLightState, \
    TrafficSignIDUsa, TrafficSignIDChina, TrafficSignIDZamunda
from matplotlib.offsetbox import OffsetImage, AnnotationBbox, HPacker, TextArea, VPacker
from matplotlib.path import Path
from commonroad.prediction.prediction import Occupancy
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet, LineMarking
from commonroad.scenario.obstacle import DynamicObstacle, StaticObstacle, ObstacleRole
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory

# path to traffic sign .png files
traffic_sign_path = os.path.join(os.path.dirname(__file__), 'traffic_signs/')


def additional_value_position_dict() -> dict:
    """Describes vertical offset of additional_value's text in relation to the traffic sign image."""
    return {'274': -16.5}


def text_prop_dict() -> dict:
    """Properties of text for additional_value."""
    return {'274': {'weight':'bold'}}


def file_path_dict() -> dict:
    """Assigns path to country of a traffic sign."""
    return {TrafficSignIDGermany: 'germany',
            TrafficSignIDUsa: 'usa',
            TrafficSignIDChina: 'china',
            TrafficSignIDZamunda: 'zamunda'}


def draw_traffic_sign(traffic_signs: Union[List[TrafficSign], TrafficSign],
                      plot_limits: Union[List[Union[int,float]], None], ax: mpl.axes.Axes, draw_params: dict,
                      draw_func: Dict[type,Callable],
                      handles: Dict[Any,List[Union[mpl.patches.Patch,mpl.collections.Collection]]],
                      call_stack: Tuple[str,...]) -> None:

    if type(traffic_signs) is not list:
        traffic_signs = [traffic_signs]

    call_stack = tuple(list(call_stack) + ['traffic_sign'])

    scale_factor = commonroad.visualization.draw_dispatch_cr._retrieve_value(
        draw_params, call_stack,
        ('traffic_sign', 'scale_factor'))

    pos_dict = additional_value_position_dict()
    path_dict = file_path_dict()
    prop_dict = text_prop_dict()
    # plt.autoscale()
    # plt.axis('equal')
    # plt.draw()
    # plt.pause(0.001)

    for traffic_sign in traffic_signs:
        if traffic_sign.virtual is True: continue
        imageboxes = []
        for element in traffic_sign.traffic_sign_elements:
            el_id = element.traffic_sign_element_id
            path = os.path.join(traffic_sign_path, path_dict[type(el_id)], el_id.value + '.png')

            # get position of additional values
            text = None
            sep = 0
            if len(element.additional_values) > 0:
                if el_id.value in pos_dict:
                    sep = pos_dict[el_id.value]

                text = '\n'.join(element.additional_values)

                print(traffic_sign.position, text)
            # plot traffic sign
            if os.path.exists(path):
                img = PIL.Image.open(path)
            else:
                warnings.warn('No png file for traffic sign id {} exists under path {}, skipped plotting.'
                              .format(el_id, path))
                continue

            img = OffsetImage(img, zoom=scale_factor, zorder=30)
            if text is not None:
                props = prop_dict[el_id.value] if el_id.value in prop_dict else {}
                img = VPacker(children=[img, TextArea(text, textprops=props)],pad=0, sep=sep,align='center')

            imageboxes.append(img)

        if len(imageboxes) > 0:
            hbox = HPacker(children=imageboxes, pad=0, sep=0, align='center')
            ab = AnnotationBbox(hbox, traffic_sign.position, xybox=traffic_sign.position, xycoords='data', frameon=False)
            ab.zorder = 30
            ax.add_artist(ab)
