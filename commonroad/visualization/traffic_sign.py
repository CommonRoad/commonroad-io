import copy
import enum
import os
from collections import defaultdict
from typing import Dict, Callable, Tuple, Union, Any, Set, Iterable
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
    return {'274': -16.5,
            '275': -16.5,
            '278': -16.5,
            '279': -16.5,
            '380': -16.5,
            '381': -16.5,
            'R2-1': -12.2}


def text_prop_dict() -> dict:
    """Properties of text for additional_value."""
    return {'274': {'weight': 'bold', 'size': 11},
            '275': {'weight': 'bold', 'color': 'white', 'size': 11},
            '278': {'weight': 'bold', 'color': 'gray', 'size': 11},
            '279': {'weight': 'bold', 'color': 'white', 'size': 11},
            '380': {'weight': 'bold', 'color': 'white', 'size': 11},
            '381': {'weight': 'bold', 'color': 'white', 'size': 11},
            'R2-1': {'weight': 'normal', 'color': 'black', 'size': 10}}


def draw_traffic_sign(traffic_signs: Union[List[TrafficSign], TrafficSign],
                      plot_limits: Union[List[Union[int,float]], None], ax: mpl.axes.Axes, draw_params: dict,
                      draw_func: Dict[type,Callable],
                      handles: Dict[Any,List[Union[mpl.patches.Patch,mpl.collections.Collection]]],
                      call_stack: Tuple[str,...]) -> None:

    if type(traffic_signs) is not list:
        traffic_signs = [traffic_signs]

    scale_factor = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_sign', 'scale_factor'), ('scenario','lanelet_network','traffic_sign', 'scale_factor'))
    show_label_default = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_sign', 'show_label'), ('scenario', 'lanelet_network', 'traffic_sign', 'show_label'))
    show_traffic_signs = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_sign', 'show_traffic_signs'), ('scenario','lanelet_network','traffic_sign', 'show_traffic_signs'))
    zorder = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_sign', 'zorder'), ('scenario','lanelet_network','traffic_sign', 'zorder'))
    kwargs = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_sign', 'kwargs'), ('scenario','lanelet_network','traffic_sign', 'kwargs'))

    assert any([show_traffic_signs == 'all',
                isinstance(show_traffic_signs, list) and type(show_traffic_signs[0] is enum)]),\
        'Plotting option traffic_sign.show_traffic_signs must be either "all" or list of type TrafficSignID'

    # add default AnnotationBox args if not specified by user
    default_params = dict(xycoords='data', frameon=False)
    for param, value in default_params.items():
        if param not in kwargs:
            kwargs[param] = value

    #call_stack = tuple(list(call_stack) + ['traffic_sign'])
    pos_dict = additional_value_position_dict()
    prop_dict = text_prop_dict()

    for traffic_sign in traffic_signs:
        if traffic_sign.virtual is True: continue
        imageboxes = []
        for element in traffic_sign.traffic_sign_elements:
            el_id = element.traffic_sign_element_id
            if not (show_traffic_signs == 'all' or el_id in show_traffic_signs): continue
            show_label = show_label_default
            path = os.path.join(traffic_sign_path, el_id.__class__.__name__, el_id.value + '.png')
            plot_img = True
            if not os.path.exists(path):
                path = os.path.join(traffic_sign_path, 'TrafficSignIDZamunda', el_id.value + '.png')
                if not os.path.exists(path):
                    show_label = True
                    warnings.warn('No png file for traffic sign id {} exists under path {}, skipped plotting.'
                                  .format(el_id, path))
                    plot_img = False

            boxes = []  # collect matplotlib offset boxes for text and images
            if show_label:
                boxes.append(TextArea(el_id.name))

            if plot_img:
                # plot traffic sign
                sign_img = PIL.Image.open(path)
                boxes.append(OffsetImage(sign_img, zoom=scale_factor, zorder=zorder, interpolation='bicubic'))

            # already stack label and img in case both are shown (prevents misalignment with additional text)
            if len(boxes) > 1:
                boxes = [VPacker(children=boxes, pad=0, sep=0, align='center')]

            # get additional values
            sep = 0
            if len(element.additional_values) > 0:
                if plot_img and el_id.value in pos_dict:
                    sep = pos_dict[el_id.value]

                add_text = '\n'.join(element.additional_values)
                props = prop_dict[el_id.value] if el_id.value in prop_dict else {}
                boxes.append(TextArea(add_text, textprops=props))

            # stack boxes vertically
            img = VPacker(children=boxes,pad=0, sep=sep,align='center')
            imageboxes.append(img)

        # horizontally stack all traffic sign elements of the traffic sign
        if len(imageboxes) > 0:
            hbox = HPacker(children=imageboxes, pad=0, sep=0.05, align='baseline')
            kwargs_tmp = copy.deepcopy(kwargs)
            if 'xybox' not in kwargs_tmp:
                kwargs_tmp['xybox'] = traffic_sign.position

            ab = AnnotationBbox(hbox, traffic_sign.position, **kwargs_tmp)
            ab.zorder = 30
            ax.add_artist(ab)
