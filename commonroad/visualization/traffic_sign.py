import copy
import enum
import os
from collections import defaultdict, OrderedDict
from typing import Dict, Callable, Tuple, Union, Any
import commonroad.geometry.shape
import matplotlib as mpl
import matplotlib.patches as patches
import matplotlib.collections as collections
from PIL import Image

import commonroad.prediction.prediction
import commonroad.scenario.obstacle
import commonroad.visualization.draw_dispatch_cr
from commonroad.geometry.shape import *
from commonroad.scenario.traffic_sign import TrafficSign, TrafficSignIDGermany, TrafficLight, TrafficLightState, \
    TrafficSignIDUsa, TrafficSignIDChina, TrafficSignIDZamunda, SupportedTrafficSignCountry
from matplotlib.offsetbox import OffsetImage, AnnotationBbox, HPacker, TextArea, VPacker, OffsetBox

# path to traffic sign .png files
traffic_sign_path = os.path.join(os.path.dirname(__file__), 'traffic_signs/')


speed_limit_factors = {'mph': 2.23694, 'kmh': 3.6, 'ms': 1.0}


def isfloat(value: str):
  try:
    float(value)
    return True
  except ValueError:
    return False


def speed_limit_factor(country_code) -> float:
    """Determine factor for speed_limit_unit by country code."""
    # dicts for units other than kph
    mph_countries = [TrafficSignIDUsa]

    if type(country_code) in mph_countries:
        return speed_limit_factors['mph']
    else:
        return speed_limit_factors['kmh']


# denotes traffic signs that are speed limits
is_speed_limit_id = ['274', '275', 'R2-1']


def text_prop_dict() -> dict:
    """Properties of text for additional_value."""
    return {'274': {'mpl_args':{'weight': 'bold', 'size': 13.5},
                    'rescale_threshold': 2, 'position_offset': -21.0},
            '275': {'mpl_args':{'weight': 'bold', 'color': 'white', 'size': 13.5},
                    'rescale_threshold': 2, 'position_offset':-21.0},
            '278': {'mpl_args':{'weight': 'bold', 'color': 'gray', 'size': 10},
                    'position_offset': -16.5},
            '279': {'mpl_args':{'weight': 'bold', 'color': 'white', 'size': 10},
                    'position_offset': -16.5},
            '310': {'mpl_args': {'weight': 'normal', 'color': 'black', 'size': 10}},
            '380': {'mpl_args':{'weight': 'bold', 'color': 'white', 'size': 10},
                    'position_offset': -16.5},
            '381': {'mpl_args':{'weight': 'bold', 'color': 'white', 'size': 10},
                    'position_offset': -16.5},
            'R2-1': {'mpl_args':{'weight': 'normal', 'color': 'black', 'size': 10.5},
                     'position_offset': -13.5}}


def rescale_text(string:str, prop:dict, scale_factor:float, default_scale_factor:float) -> dict:
    """Rescales text size proportionally to the max. number of strings given by prop['rescale_threshold'] and to the
    'scale_factor' compared to the default scale_factor. Used e.g. for fitting speed limits into the traffic sign."""
    prop = copy.deepcopy(prop)
    if default_scale_factor != scale_factor:
        tmp_scale_factor = scale_factor / default_scale_factor
        if 'position_offset' in prop:
            prop['position_offset'] *= tmp_scale_factor

        if 'mpl_args' in prop and 'size' in prop['mpl_args']:
            prop['mpl_args']['size'] *= tmp_scale_factor

    if 'rescale_threshold' in prop:
        if len(string) > prop['rescale_threshold']:
            if 'mpl_args' in prop and 'size' in prop['mpl_args']:
                prop['mpl_args']['size'] *= prop['rescale_threshold'] / len(string) * 1.1
            if 'position_offset' in prop:
                prop['position_offset'] *= prop['rescale_threshold'] / len(string) * 1.35

    return prop


def create_img_boxes_traffic_sign(traffic_signs: Union[List[TrafficSign], TrafficSign], draw_params: dict,
                                  call_stack: Tuple[str,...]) -> Dict[Tuple[float,float],List[OffsetBox]]:
    """
    For each Traffic sign an OffsetBox is created, containing the png image and optionally labels. These boxes can
    be stacked horizontally later when multiple signs or lights share the same position.
    :param traffic_signs:
    :param draw_params:
    :param call_stack:
    :return:
    """
    if type(traffic_signs) is not list:
        traffic_signs = [traffic_signs]

    if len(traffic_signs) == 0:
        return dict()

    scale_factor = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_sign', 'scale_factor'), ('scenario','lanelet_network','traffic_sign', 'scale_factor'))
    speed_limit_unit = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_sign', 'speed_limit_unit'), ('scenario', 'lanelet_network', 'traffic_sign', 'speed_limit_unit'))
    show_label_default = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_sign', 'show_label'), ('scenario', 'lanelet_network', 'traffic_sign', 'show_label'))
    show_traffic_signs = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_sign', 'show_traffic_signs'), ('scenario','lanelet_network','traffic_sign', 'show_traffic_signs'))
    zorder = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_sign', 'zorder'), ('scenario','lanelet_network','traffic_sign', 'zorder'))

    scale_factor_default = commonroad.visualization.draw_dispatch_cr._retrieve_value(
        commonroad.visualization.draw_dispatch_cr.default_draw_params, call_stack,
        ('scenario', 'lanelet_network', 'traffic_sign', 'scale_factor'))

    assert any([show_traffic_signs == 'all',
                isinstance(show_traffic_signs, list) and type(show_traffic_signs[0] is enum)]),\
        'Plotting option traffic_sign.show_traffic_signs must be either "all" or list of type TrafficSignID'

    prop_dict = text_prop_dict()
    imageboxes_all = defaultdict(list)

    for traffic_sign in traffic_signs:
        if traffic_sign.virtual is True or traffic_sign.position is None: continue
        imageboxes = []
        for element in traffic_sign.traffic_sign_elements:
            el_id = element.traffic_sign_element_id
            if not (show_traffic_signs == 'all' or el_id in show_traffic_signs): continue
            show_label = show_label_default
            path = os.path.join(traffic_sign_path, el_id.__class__.__name__, el_id.value + '.png')
            plot_img = True
            # get png image
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
                sign_img = Image.open(path)
                boxes.append(OffsetImage(sign_img, zoom=scale_factor, zorder=zorder, interpolation='bicubic'))

            # already stack label and img in case both are shown (prevents misalignment with additional text)
            if len(boxes) > 1:
                boxes = [VPacker(children=boxes, pad=0, sep=0, align='center')]

            # get additional values string (like speed limits)
            sep = 0
            if len(element.additional_values) > 0:
                if element.traffic_sign_element_id.value in is_speed_limit_id \
                    and isfloat(element.additional_values[0]):
                    if speed_limit_unit == 'auto':
                        speed_factor = speed_limit_factor(element.traffic_sign_element_id)
                    else:
                        speed_factor = speed_limit_factors[speed_limit_unit]

                    add_text = str(round(speed_factor * float(element.additional_values[0])))
                else:
                    add_text = '\n'.join(element.additional_values)

                props = prop_dict[el_id.value] if el_id.value in prop_dict else {'mpl_args':{}}
                props = rescale_text(add_text, props, scale_factor, scale_factor_default)
                boxes.append(TextArea(add_text, textprops=props['mpl_args']))

                # position text label on png image
                if plot_img and 'position_offset' in props:
                    sep = props['position_offset']

            # stack boxes vertically
            img = VPacker(children=boxes, pad=0, sep=sep, align='center')
            imageboxes.append(img)

        # horizontally stack all traffic sign elements of the traffic sign
        if len(imageboxes) > 0:
            hbox = HPacker(children=imageboxes, pad=0, sep=0.05, align='baseline')
            imageboxes_all[tuple(traffic_sign.position.tolist())].append(hbox)

    return imageboxes_all


def create_img_boxes_traffic_lights(traffic_lights: Union[List[TrafficLight], TrafficLight], draw_params: dict,
                                  call_stack: Tuple[str,...]) -> Dict[Tuple[float,float],List[OffsetBox]]:
    """
    For each Traffic light an OffsetBox is created, containing the png image and optionally labels. These boxes can
    be stacked horizontally later when multiple signs or lights share the same position.
    :param traffic_lights:
    :param draw_params:
    :param call_stack:
    :return:
    """
    if type(traffic_lights) is not list:
        traffic_lights = [traffic_lights]

    if len(traffic_lights) == 0:
        return dict()

    time_begin = commonroad.visualization.draw_dispatch_cr._retrieve_value(
        draw_params, call_stack,
        ('time_begin',))
    scale_factor = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_light', 'scale_factor'), ('scenario','lanelet_network','traffic_light', 'scale_factor'))
    show_label = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_light', 'show_label'), ('scenario', 'lanelet_network', 'traffic_light', 'show_label'))
    zorder = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_light', 'zorder'), ('scenario','lanelet_network','traffic_light', 'zorder'))

    # plots all group members horizontally stacked
    imageboxes_all = defaultdict(list)
    for traffic_light in traffic_lights:
        if traffic_light.position is None:
            continue
        if traffic_light.active:
            state = traffic_light.get_state_at_time_step(time_begin)
            path = os.path.join(traffic_sign_path, 'traffic_light_state_' + str(state.value) + '.png')
        else:
            path = os.path.join(traffic_sign_path, 'traffic_light_state_inactive.png')

        boxes = []  # collect matplotlib offset boxes for text and images
        sign_img = Image.open(path)
        boxes.append(OffsetImage(sign_img, zoom=scale_factor, zorder=zorder, interpolation='bicubic'))

        if show_label:
            boxes.append(TextArea(str(state.value)))

        # stack boxes vertically
        img_box = VPacker(children=boxes,pad=0, sep=0, align='center')

        imageboxes_all[tuple(traffic_light.position.tolist())].append(img_box)

    return imageboxes_all


def draw_traffic_light_signs(traffic_lights_signs: Union[List[Union[TrafficLight,TrafficSign]],
                                                         Union[TrafficLight,TrafficSign]],
                             plot_limits: Union[List[Union[int,float]], None], ax: mpl.axes.Axes, draw_params: dict,
                             draw_func: Dict[type,Callable],
                             handles: Dict[Any,List[Union[mpl.patches.Patch,mpl.collections.Collection]]],
                             call_stack: Tuple[str,...]) -> None:
    """
    Draws OffsetBoxes which are first collected for all traffic signs and -lights. Boxes are stacked together when they
    share the same position.
    :param traffic_lights_signs:
    :param plot_limits:
    :param ax:
    :param draw_params:
    :param draw_func:
    :param handles:
    :param call_stack:
    :return:
    """
    kwargs = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('kwargs_traffic_light_signs',), ('scenario','lanelet_network','kwargs_traffic_light_signs'))

    zorder_0 = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_light', 'zorder'), ('scenario','lanelet_network','traffic_light', 'zorder'))

    zorder_1 = commonroad.visualization.draw_dispatch_cr._retrieve_alternate_value(
        draw_params, call_stack,
        ('traffic_sign', 'zorder'), ('scenario','lanelet_network','traffic_sign', 'zorder'))

    zorder = min(zorder_0, zorder_1)
    threshold_grouping = 0.8  # [m] distance threshold for grouping traffic light and/or signs

    if not isinstance(traffic_lights_signs, list):
        traffic_lights_signs = [traffic_lights_signs]

    traffic_signs = []
    traffic_lights = []

    for obj in traffic_lights_signs:
        if isinstance(obj, TrafficSign):
            traffic_signs.append(obj)
        elif isinstance(obj, TrafficLight):
            traffic_lights.append(obj)
        else:
            warnings.warn('Object of type {}, but expected type TrafficSign or TrafficLight'.format(type(obj)))

    # collect ImageBoxes of traffic signs/lights grouped by their positions
    boxes_tl = create_img_boxes_traffic_lights(traffic_lights, draw_params, call_stack)
    boxes_signs = create_img_boxes_traffic_sign(traffic_signs, draw_params, call_stack)
    img_boxes = defaultdict(list)  # {position: List[OffsetBox]}
    [img_boxes[pos].extend(box_list) for pos, box_list in boxes_tl.items()]
    [img_boxes[pos].extend(box_list) for pos, box_list in boxes_signs.items()]

    if not img_boxes:
        return None

    positions = list(img_boxes.keys())
    box_lists = list(img_boxes.values())

    # group objects based on their positions' distances
    groups = dict()
    grouped = set()  # set of already assigned keys
    i = 1
    for pos, box_list in zip(positions[:-1], box_lists[:-1]):
        i += 1
        group_tmp = list(box_list)
        if pos in grouped: continue
        gr_pos_tmp = [np.array(pos)]  # collect positions of members
        for pos2, box_list2 in zip(positions[i:], box_lists[i:]):
            if pos2 in grouped: continue
            if np.linalg.norm(np.array(pos) - np.array(pos2), ord=np.inf) < threshold_grouping:
                group_tmp.extend(box_list2)
                gr_pos_tmp.append(np.array(pos2))

        grouped.add(pos)  # collect ids of all objects
        groups[tuple(np.average(gr_pos_tmp, axis=0).tolist())] = group_tmp

    if positions[-1] not in grouped:
        groups[positions[-1]] = box_lists[-1]

    # add default AnnotationBox args if not specified by user
    default_params = dict(xycoords='data', frameon=False)
    for param, value in default_params.items():
        if param not in kwargs:
            kwargs[param] = value

    # stack imageboxes of each group and draw
    for position_tmp, box_list_tmp in groups.items():
        position_tmp = np.array(position_tmp)
        kwargs_tmp = copy.deepcopy(kwargs)
        if 'xybox' not in kwargs_tmp:
            kwargs_tmp['xybox'] = position_tmp

        hbox = HPacker(children=box_list_tmp, pad=0, sep=0.1, align='baseline')
        ab = AnnotationBbox(hbox, position_tmp, **kwargs_tmp)
        ab.zorder = zorder
        ax.add_artist(ab)


