import itertools
import math
import os
from collections import defaultdict
from typing import Dict, Callable, Tuple, Union, Any, Set
import commonroad.geometry.shape
import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.patches as patches
import matplotlib.collections as collections
# import pylustrator

import commonroad.prediction.prediction
import commonroad.scenario.obstacle
import commonroad.visualization.draw_dispatch_cr
from commonroad.common.util import Interval
from commonroad.geometry.shape import *
from commonroad.scenario.intersection import Intersection
from commonroad.scenario.traffic_sign import TrafficSign, TrafficLight, TrafficLightState
from commonroad.visualization.traffic_sign import draw_traffic_light_signs
from matplotlib.path import Path
from commonroad.prediction.prediction import Occupancy
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet, LineMarking
from commonroad.scenario.obstacle import DynamicObstacle, StaticObstacle, ObstacleRole
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory, State

from commonroad.visualization.util import draw_polygon_as_patch, draw_polygon_collection_as_patch, LineDataUnits, \
    collect_center_line_colors

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = [""]
__version__ = "2020.3"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"

traffic_sign_path = os.path.join(os.path.dirname(__file__), 'traffic_signs/')

def create_default_draw_params() -> dict:
    basic_shape_parameters_static = {'opacity': 1.0,
                                   'facecolor': '#d95558',
                                   'edgecolor': '#831d20',
                                   'linewidth': 0.5,
                                   'zorder': 20}

    basic_shape_parameters_dynamic = {'opacity': 1.0,
                                     'facecolor': '#1d7eea',
                                     'edgecolor': '#0066cc',
                                     'linewidth':0.5,
                                     'zorder': 20}

    draw_params = {'scenario': {
                        'dynamic_obstacle': {
                            'draw_shape': True,
                            'draw_icon': False,
                            'draw_bounding_box': True,
                            'show_label': False,  # show id of obstacle
                            'zorder': 20,
                            'draw_signals': True,  # draw signal states
                            'signal_radius': 0.5,  # size of a signal states
                            'indicator_color': '#ebc200',  # turn light
                            'braking_color': 'red',  # braking light
                            'blue_lights_color': 'blue',
                            'horn_color': 'red',  # horn is visualized as red center dot
                            'initial_state': {
                                'draw_initial_state': False,  # visualize initial state by arrow proportional to velocity
                                'scale_factor': 0.3,  # length of arrow in m per m/s
                                'kwargs':{
                                    'linewidth': 1.5,
                                    'length_includes_head': True,
                                    'edgecolor': 'black',
                                    'facecolor': 'black',
                                    },  # further parameters for FancyArrow
                            },
                            'occupancy': {
                                'draw_occupancies': 0,  # -1= never, 0= if prediction of vehicle is set-based, 1=always
                                'shape': {
                                    'polygon': {
                                    'opacity': 0.2,
                                       'facecolor': '#1d7eea',
                                       'edgecolor': '#0066cc',
                                       'linewidth': 0.5,
                                       'zorder': 18,
                                    },
                                    'rectangle': {
                                       'opacity': 0.2,
                                       'facecolor': '#1d7eea',
                                       'edgecolor': '#0066cc',
                                       'linewidth': 0.5,
                                       'zorder': 18,
                                    },
                                    'circle': {
                                       'opacity': 0.2,
                                       'facecolor': '#1d7eea',
                                       'edgecolor': '#0066cc',
                                       'linewidth': 0.5,
                                       'zorder': 18,
                                    }
                                },
                            },
                            'shape': {
                                'polygon': basic_shape_parameters_dynamic,
                                'rectangle': basic_shape_parameters_dynamic,
                                'circle': basic_shape_parameters_dynamic
                            },
                             'trajectory': {'draw_trajectory':True,
                                            'facecolor': '#000000',
                                            'draw_continuous': False,
                                            'unique_colors': False,
                                            'line_width': 0.17,
                                            'z_order': 24}
                        },
                        'static_obstacle': {
                           'shape': {
                               'polygon': basic_shape_parameters_static,
                               'rectangle':basic_shape_parameters_static,
                               'circle': basic_shape_parameters_static,
                           }
                        },
                        'lanelet_network': {
                            'kwargs_traffic_light_signs': {}, # further properties for AnnotationBox of traffic signs or lights, see # https://matplotlib.org/3.1.0/gallery/text_labels_and_annotations/demo_annotation_box.html
                            'traffic_light': {'draw_traffic_lights': True,
                                              'red_color': 'red',
                                              'yellow_color': '#feb609',
                                              'green_color': '#00aa16',
                                              'red_yellow_color': '#fe4009ff',
                                              'show_label': False,
                                              'scale_factor': 0.25,
                                              'zorder': 30},
                            'traffic_sign': {'draw_traffic_signs': False,
                                             'show_traffic_signs': 'all',  # 'all' or list of TrafficSignIDs
                                             'speed_limit_unit': 'auto',  # 'mph', 'kmh', 'ms', 'auto'
                                             'show_label': False,
                                             'scale_factor': 0.25,
                                             'zorder': 30},
                            'intersection': {'draw_intersections': False,
                                             'draw_incoming_lanelets': True,
                                             'incoming_lanelets_color': '#3ecbcf',
                                             'draw_crossings': True,
                                             'crossings_color': '#b62a55',
                                             'draw_successors': True,
                                             'successors_left_color': '#ff00ff',
                                             'successors_straight_color': 'blue',
                                             'successors_right_color': '#ccff00',
                                             'show_label': False,  # show incoming id and incoming left
                                             },
                            'lanelet': {'left_bound_color': '#555555',
                                       'right_bound_color': '#555555',
                                       'center_bound_color': '#dddddd',
                                       'unique_colors': False,  # colorizes center_vertices and labels of each lanelet differently
                                       'draw_stop_line':True,
                                       'stop_line_color':'#ffffff',
                                       'draw_line_markings':True,
                                       'draw_left_bound': True,
                                       'draw_right_bound': True,
                                       'draw_center_bound': True,
                                       'draw_border_vertices': False,
                                       'draw_start_and_direction': True,
                                       'show_label': False,  # show lanelet_id
                                       'draw_linewidth': 0.5,
                                       'fill_lanelet': True,
                                       'facecolor': '#c7c7c7'}},
                   }
    }
    # ensure that parameters are also available on higher levels
    draw_params.update(draw_params['scenario'])
    draw_params['shape'] = basic_shape_parameters_static
    draw_params['shape'].update(draw_params['scenario']['static_obstacle']['shape'])
    draw_params['occupancy'] = draw_params['scenario']['dynamic_obstacle']['occupancy']
    draw_params['static_obstacle'] = draw_params['scenario']['static_obstacle']
    draw_params['dynamic_obstacle'] = draw_params['scenario']['dynamic_obstacle']
    draw_params['trajectory'] = draw_params['scenario']['dynamic_obstacle']['trajectory']
    draw_params['lanelet_network'] = draw_params['scenario']['lanelet_network']
    draw_params['lanelet'] = draw_params['scenario']['lanelet_network']['lanelet']
    draw_params['intersection'] = draw_params['scenario']['lanelet_network']['intersection']
    draw_params['traffic_sign'] = draw_params['scenario']['lanelet_network']['traffic_sign']
    draw_params['traffic_light'] = draw_params['scenario']['lanelet_network']['traffic_light']
    draw_params['scenario']['lanelet'] = draw_params['scenario']['lanelet_network']['lanelet']

    return draw_params


def line_marking_to_linestyle(line_marking: LineMarking) -> Tuple:
    """:returns: Tuple[line_style, dashes, line_width] for matplotlib plotting options."""
    return {LineMarking.DASHED: ('--', (10,10), 0.25,),
            LineMarking.SOLID: ('-', (None,None), 0.25),
            LineMarking.BROAD_DASHED: ('--',(10,10), 0.5),
            LineMarking.BROAD_SOLID: ('-', (None,None), 0.5)}[line_marking]


def traffic_light_color_dict(traffic_light_state: TrafficLightState, params:dict):
    """Retrieve color code for traffic light state."""
    return {TrafficLightState.RED: params['red_color'],
            TrafficLightState.YELLOW: params['yellow_color'],
            TrafficLightState.GREEN: params['green_color'],
            TrafficLightState.RED_YELLOW: params['red_yellow_color']}[traffic_light_state]


def draw_scenario(obj: Scenario, plot_limits: Union[List[Union[int,float]], None], ax: mpl.axes.Axes,
                  draw_params: dict, draw_func: Dict[type,Callable],
                  handles: Dict[int,List[mpl.patches.Patch]], call_stack: Tuple[str,...]) -> None:
    """
    :param obj: object to be plotted
    :param plot_limits: draw only objects inside limits [x_ min, x_max, y_min, y_max]
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """
    call_stack = tuple(list(call_stack) + ['scenario'])

    commonroad.visualization.draw_dispatch_cr.draw_object(
        obj.lanelet_network, plot_limits, ax, draw_params, draw_func, handles, call_stack)

    # draw only obstacles inside plot limits
    if plot_limits is not None:
        time_begin = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, tuple(), ('time_begin',))
        # dynamic obstacles
        commonroad.visualization.draw_dispatch_cr.draw_object(
            obj.obstacles_by_position_intervals([Interval(plot_limits[0], plot_limits[1]),
                                                 Interval(plot_limits[2], plot_limits[3])], tuple([ObstacleRole.DYNAMIC]),time_begin),
            None, ax, draw_params, draw_func, handles, call_stack)
        #static obstacles
        commonroad.visualization.draw_dispatch_cr.draw_object(
            obj.obstacles_by_position_intervals([Interval(plot_limits[0], plot_limits[1]),
                                                 Interval(plot_limits[2], plot_limits[3])], tuple([ObstacleRole.STATIC])),
            None, ax, draw_params, draw_func, handles, call_stack)
    else:
        commonroad.visualization.draw_dispatch_cr.draw_object(
            obj.dynamic_obstacles, None, ax, draw_params, draw_func,
            handles, call_stack)
        commonroad.visualization.draw_dispatch_cr.draw_object(
            obj.static_obstacles, None, ax, draw_params, draw_func,
            handles, call_stack)

def draw_static_obstacles(obj: Union[List[StaticObstacle],StaticObstacle],
                          plot_limits: Union[List[Union[int,float]], None], ax: mpl.axes.Axes, draw_params: dict,
                          draw_func: Dict[type,Callable],
                          handles: Dict[int,List[mpl.patches.Patch]], call_stack: Tuple[str,...]) -> None:
    """
    :param obj: object to be plotted
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """
    time_begin = commonroad.visualization.draw_dispatch_cr._retrieve_value(
        draw_params, tuple(), ('time_begin',))

    if type(obj) is StaticObstacle:
        obj = [obj]

    call_stack = tuple(list(call_stack) + ['static_obstacle'])

    shape_list = list()
    for obstacle in obj:
        if type(obstacle) is not StaticObstacle:
            warnings.warn('<visualization/scenario> Only lists with objects of the same type can be plotted',
                          UserWarning, stacklevel=1)
            continue
        shape_list.append(obstacle.occupancy_at_time(time_begin).shape)

    collection = shape_batch_dispatcher(shape_list, None, ax, draw_params, draw_func, handles, call_stack)

    handles.setdefault(StaticObstacle, []).extend(collection)


def draw_trajectories(obj: Union[List[Trajectory],Trajectory], plot_limits: Union[List[Union[int,float]], None],
                      ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable],
                      handles: Dict[int,List[mpl.patches.Patch]], call_stack: Tuple[str,...])\
        -> List[mpl.collections.Collection]:
    """
    :param obj: object to be plotted
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """

    if type(obj) is Trajectory:
        obj = [obj]

    try:
        facecolor = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,('trajectory', 'facecolor'))
        unique_colors = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('trajectory', 'unique_colors'))
        line_width = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack, ('trajectory', 'line_width'))
        draw_continuous = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack, ('trajectory', 'draw_continuous'))
        z_order = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack, ('trajectory', 'z_order'))
        time_begin = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,('time_begin',))
        time_end = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack, tuple(['time_end']))
    except KeyError:
        print("Cannot find stylesheet for trajectory. Called through:")
        print(call_stack)

    if time_begin==time_end:
        return []

    # select unique colors from colormap for each lanelet's center_line
    colormap = None
    if unique_colors is True:
        norm = mpl.colors.Normalize(vmin=0, vmax=len(obj))
        colormap = cm.ScalarMappable(norm=norm, cmap=cm.brg)

    patchlist = list()
    traj_list =list()

    for traj in obj:
        traj_points = list()
        for time_step in range(time_begin, time_end):
            tmp = traj.state_at_time_step(time_step)
            if tmp is not None:
                traj_points.append(tmp.position)
            else:
                if time_begin > traj.initial_time_step:
                    break

        if traj_points:
            traj_list.append(traj_points)

    if len(traj_list) > 0:
        if draw_continuous is True:
            paths = []
            for traj in traj_list:
                paths.append(Path(traj, closed=False))
            if unique_colors is True:
                collection = []
                for i_p, path in enumerate(paths):
                    facecolor = colormap.to_rgba(i_p)
                    collection.append(collections.PathCollection([path], color=facecolor, lw=line_width, zorder=z_order,
                                               facecolor='none'))
                    ax.add_collection(collection[-1])
            else:
                collection = [collections.PathCollection(paths, color=facecolor, lw=line_width, zorder=z_order, facecolor='none')]
        else:
            if unique_colors is True:
                collection = []
                for i_p, traj in enumerate(traj_list):
                    facecolor = colormap.to_rgba(i_p)
                    traj = np.array(traj)
                    collection.append(collections.EllipseCollection(np.ones([traj.shape[0],1]) * line_width,
                                                           np.ones([traj.shape[0],1]) * line_width,
                                                           np.zeros([traj.shape[0],1]),
                                                           offsets=traj,
                                                           units='xy',
                                                           linewidths=0,
                                                       zorder=z_order, transOffset=ax.transData, facecolor=facecolor))
                    ax.add_collection(collection[-1])
            else:
                try:
                    traj_list = np.array(np.concatenate(traj_list))
                except:
                    dsdf=0
                collection = [collections.EllipseCollection(np.ones([traj_list.shape[0],1]) * line_width,
                                                           np.ones([traj_list.shape[0],1]) * line_width,
                                                           np.zeros([traj_list.shape[0],1]),
                                                           offsets=traj_list,
                                                           units='xy',
                                                           linewidths=0,
                                                       zorder=z_order, transOffset=ax.transData, facecolor=facecolor)]
        if draw_continuous is False:
            ax.add_collection(collection[0])
    else:
        collection = []

    return collection


def draw_lanelet_network(obj: LaneletNetwork , plot_limits: Union[List[Union[int,float]], None],
                         ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable],
                         handles: Dict[int,List[mpl.patches.Patch]], call_stack: Tuple[str,...]) -> None:
    """
    :param obj: object to be plotted
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """
    call_stack = tuple(list(call_stack) + ['lanelet_network'])
    # if plot_limits is not None:
    #     center = [plot_limits[1]+plot_limits[0]*0.5, plot_limits[3]+plot_limits[2]*0.5]
    #     radius = np.sqrt((plot_limits[1]-plot_limits[0])**2 + (plot_limits[3]-plot_limits[2])**2)
    #     lanelets = obj.lanelets_in_proximity(center, radius)
    # else:

    _draw_lanelets_intersection(
        obj, obj._traffic_lights, obj._traffic_signs, obj.intersections,
        None, ax, draw_params, draw_func, handles, call_stack)


def draw_lanelet_list(obj: List[Lanelet] , plot_limits: Union[List[Union[int,float]], None],
                         ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable],
                         handles: Dict[int,List[mpl.patches.Patch]], call_stack: Tuple[str,...]) -> None:
        """
        Draws list of lanelets.
        """
        if isinstance(obj, Lanelet):
            obj = [obj]

        _draw_lanelets_intersection(LaneletNetwork.create_from_lanelet_list(obj), None, None, None, plot_limits, ax, draw_params, draw_func, handles,
                                    call_stack)


def _draw_lanelets_intersection(obj: LaneletNetwork,
                                traffic_lights: Union[Dict[int, TrafficLight], None],
                                traffic_signs: Union[Dict[int,TrafficSign],None],
                                intersections: Union[List[Intersection],None],
                                plot_limits: Union[List[Union[int,float]], None],
                                ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable],
                                handles: Dict[int,List[mpl.patches.Patch]], call_stack: Tuple[str,...]) -> None:
    """
    :param obj: object to be plotted
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """
    lanelets = obj.lanelets

    try:
        time_begin = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack, ('time_begin',))
        if traffic_lights is not None:
            draw_traffic_lights = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'traffic_light', 'draw_traffic_lights'))

            traffic_light_colors = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'traffic_light'))
        else:
            draw_traffic_lights = False

        if traffic_signs is not None:
            draw_traffic_signs = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'traffic_sign', 'draw_traffic_signs'))
            show_traffic_sign_label = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'traffic_sign', 'show_label'))
        else:
            draw_traffic_signs=show_traffic_sign_label = False

        if intersections is not None and len(intersections) > 0:
            draw_intersections = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'intersection', 'draw_intersections'))
        else:
            draw_intersections = False

        if draw_intersections is True:
            draw_incoming_lanelets = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'intersection', 'draw_incoming_lanelets'))
            incoming_lanelets_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'intersection', 'incoming_lanelets_color'))
            draw_crossings = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'intersection', 'draw_crossings'))
            crossings_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'intersection', 'crossings_color'))
            draw_successors = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'intersection', 'draw_successors'))
            successors_left_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'intersection', 'successors_left_color'))
            successors_straight_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'intersection', 'successors_straight_color'))
            successors_right_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'intersection', 'successors_right_color'))
            show_intersection_labels = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('lanelet_network', 'intersection', 'show_label'))
        else:
            draw_incoming_lanelets=draw_crossings=draw_successors=show_intersection_labels = False

        left_bound_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'left_bound_color'))
        right_bound_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'right_bound_color'))
        center_bound_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'center_bound_color'))
        unique_colors = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'unique_colors'))
        draw_stop_line = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'draw_stop_line'))
        stop_line_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'stop_line_color'))
        draw_line_markings = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'draw_line_markings'))
        show_label = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'show_label'))
        draw_border_vertices = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'draw_border_vertices'))
        draw_left_bound = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'draw_left_bound'))
        draw_right_bound = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'draw_right_bound'))
        draw_center_bound = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'draw_center_bound'))
        draw_start_and_direction = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'draw_start_and_direction'))
        draw_linewidth = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'draw_linewidth'))
        fill_lanelet = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'fill_lanelet'))
        facecolor = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'facecolor'))
        antialiased = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, tuple(),
            tuple(['antialiased']))

    except KeyError:
        print("Cannot find stylesheet for lanelet. Called through:")
        print(call_stack)

    incoming_lanelets = set()
    incomings_left = {}
    incomings_id = {}
    crossings = set()
    all_successors = set()
    successors_left = set()
    successors_straight = set()
    successors_right = set()
    if draw_intersections:
        # collect incoming lanelets
        if draw_incoming_lanelets:
            incomings: List[set] = [incoming.incoming_lanelets for intersection in intersections
                                    for incoming in intersection.incomings]
            incoming_lanelets: Set[int] = set.union(*incomings)
            for intersection in intersections:
                for incoming in intersection.incomings:
                    for l_id in incoming.incoming_lanelets:
                        incomings_left[l_id] = incoming.left_of
                        incomings_id[l_id] = incoming.incoming_id

        if draw_crossings:
            tmp_list: List[set] = [intersection.crossings for intersection in intersections]
            crossings: Set[int] = set.union(*tmp_list)

        if draw_successors:
            tmp_list: List[set] = [incoming.successors_left for intersection in intersections
                                    for incoming in intersection.incomings]
            successors_left: Set[int] = set.union(*tmp_list)
            tmp_list: List[set] = [incoming.successors_straight for intersection in intersections
                                    for incoming in intersection.incomings]
            successors_straight: Set[int] = set.union(*tmp_list)
            tmp_list: List[set] = [incoming.successors_right for intersection in intersections
                                    for incoming in intersection.incomings]
            successors_right: Set[int] = set.union(*tmp_list)
            all_successors = set.union(successors_straight, successors_right, successors_left)

    # direction arrow
    codes_direction = [Path.MOVETO,
             Path.LINETO,
             Path.LINETO,
             Path.CLOSEPOLY]

    scale_direction = 1.5
    pts = np.array([[0.0, -0.5, 1.0],
                    [1.0, 0.0, 1.0],
                    [0.0, 0.5, 1.0],
                    [0.0, -0.5, 1.0]])
    scale_m = np.array([[scale_direction, 0, 0],
                        [0, scale_direction, 0],
                        [0, 0, 1]])

    def direction(x, y, angle):
        """Returns path of arrow shape"""
        transform = np.array([[np.cos(angle), -np.sin(angle), x],
                              [np.sin(angle), np.cos(angle), y],
                              [0, 0, 1]])
        ptr_trans = transform.dot(scale_m.dot(pts.transpose()))
        ptr_trans = ptr_trans[0:2, :]
        ptr_trans = ptr_trans.transpose()

        path = Path(ptr_trans, codes_direction)
        return path


    # select unique colors from colormap for each lanelet's center_line
    colormap = None
    if unique_colors is True:
        norm = mpl.colors.Normalize(vmin=0, vmax=len(lanelets))
        colormap = cm.ScalarMappable(norm=norm, cmap=cm.jet)

    incoming_vertices_fill = list()
    crossing_vertices_fill = list()
    succ_left_paths = list()
    succ_straight_paths = list()
    succ_right_paths = list()

    vertices_fill = list()
    coordinates_left_border_vertices = np.empty((0,2))
    coordinates_right_border_vertices = np.empty((0,2))
    direction_list = list()
    center_paths = list()
    left_paths=list()
    right_paths = list()

    if draw_traffic_lights:
        center_line_color_dict = collect_center_line_colors(obj, traffic_lights, time_begin)

    # collect paths for drawing
    for i_lanelet, lanelet in enumerate(lanelets):
        if unique_colors:
            # set center bound color to unique value
            center_bound_color = colormap.to_rgba(i_lanelet)

        if draw_start_and_direction:
            center = lanelet.center_vertices[0]
            tan_vec = np.array(lanelet.right_vertices[0]) - np.array(lanelet.left_vertices[0])
            if unique_colors is False:
                direction_list.append(direction(center[0], center[1], np.arctan2(tan_vec[1], tan_vec[0]) + 0.5 * np.pi))
            else:
                direction_list = [direction(center[0], center[1], np.arctan2(tan_vec[1], tan_vec[0]) + 0.5 * np.pi)]
                ax.add_collection(collections.PathCollection(direction_list, color=center_bound_color,
                                                             lw=0.5, zorder=10.1, antialiased=antialiased))

        # left bound
        if draw_border_vertices or draw_left_bound:
            if draw_border_vertices:
                coordinates_left_border_vertices = np.vstack((coordinates_left_border_vertices,lanelet.left_vertices))

            if draw_line_markings and lanelet.line_marking_left_vertices is not LineMarking.UNKNOWN \
                    and lanelet.line_marking_left_vertices is not LineMarking.NO_MARKING:
                linestyle, dashes, linewidth_metres  = line_marking_to_linestyle(lanelet.line_marking_left_vertices)
                tmp_left = lanelet.left_vertices.copy()
                tmp_left[0, :] = lanelet.interpolate_position(linewidth_metres/2)[2]
                tmp_left[-1, :] = lanelet.interpolate_position(lanelet.distance[-1] - linewidth_metres / 2)[2]
                line = LineDataUnits(tmp_left[:, 0], tmp_left[:, 1], zorder=12, linewidth=linewidth_metres,
                              alpha=1.0,
                              color=left_bound_color, linestyle=linestyle,
                              dashes=dashes)
                ax.add_line(line)
            else:
                left_paths.append(Path(lanelet.left_vertices, closed=False))

        # right bound
        if draw_border_vertices or draw_right_bound:

            if draw_border_vertices:
                coordinates_right_border_vertices = np.vstack((coordinates_right_border_vertices,
                                                               lanelet.right_vertices))

            if draw_line_markings and lanelet.line_marking_right_vertices is not LineMarking.UNKNOWN \
                    and lanelet.line_marking_right_vertices is not LineMarking.NO_MARKING:
                linestyle, dashes, linewidth_metres = line_marking_to_linestyle(lanelet.line_marking_right_vertices)
                tmp_right = lanelet.right_vertices.copy()
                tmp_right[0, :] = lanelet.interpolate_position(linewidth_metres/2)[1]
                tmp_right[-1, :] = lanelet.interpolate_position(lanelet.distance[-1] - linewidth_metres / 2)[1]
                line = LineDataUnits(tmp_right[:, 0], tmp_right[:, 1], zorder=10.5, linewidth=linewidth_metres,
                                     alpha=1.0,
                                     color=right_bound_color, linestyle=linestyle,
                                     dashes=dashes)
                ax.add_line(line)
            else:
                right_paths.append(Path(lanelet.right_vertices, closed=False))

        # stop line
        if draw_stop_line and lanelet.stop_line:
            stop_line = np.vstack([lanelet.stop_line.start, lanelet.stop_line.end])
            linestyle, dashes, linewidth_metres = line_marking_to_linestyle(lanelet.stop_line.line_marking)
            # cut off in the beginning, because linewidth_metres is added later
            vec = stop_line[1, :] - stop_line[0, :]
            tangent = vec / np.linalg.norm(vec)
            stop_line[0, :] += linewidth_metres * tangent / 2
            stop_line[1, :] -= linewidth_metres * tangent / 2
            line = LineDataUnits(stop_line[:, 0], stop_line[:, 1], zorder=11, linewidth=linewidth_metres,
                                 alpha=1.0,
                                 color=stop_line_color, linestyle=linestyle,
                                 dashes=dashes)
            ax.add_line(line)

        # visualize traffic light state through colored center bound
        has_traffic_light = draw_traffic_lights and lanelet.lanelet_id in center_line_color_dict
        if has_traffic_light:
            light_state = center_line_color_dict[lanelet.lanelet_id]

            if light_state is not TrafficLightState.INACTIVE:
                linewidth_metres = 0.75
                # dashed line for red_yellow
                linestyle = '--' if light_state == TrafficLightState.RED_YELLOW else '-'
                dashes = (5, 5) if linestyle == '--' else (None, None)

                # cut off in the beginning, because linewidth_metres is added later
                tmp_center = lanelet.center_vertices.copy()
                if lanelet.distance[-1] > linewidth_metres:
                    tmp_center[0,:] = lanelet.interpolate_position(linewidth_metres)[0]
                zorder = 10.05 if light_state == TrafficLightState.GREEN else 10.0
                line = LineDataUnits(tmp_center[:,0], tmp_center[:,1], zorder=zorder, linewidth=linewidth_metres, alpha=0.7,
                                     color=traffic_light_color_dict(light_state, traffic_light_colors), linestyle=linestyle, dashes=dashes)
                ax.add_line(line)

        # draw colored center bound. Hierarchy or colors: successors > usual center bound
        is_successor = draw_intersections and draw_successors and lanelet.lanelet_id in all_successors
        if is_successor:
            if lanelet.lanelet_id in successors_left:
                succ_left_paths.append(Path(lanelet.center_vertices, closed=False))
            elif lanelet.lanelet_id in successors_straight:
                succ_straight_paths.append(Path(lanelet.center_vertices, closed=False))
            else:
                succ_right_paths.append(Path(lanelet.center_vertices, closed=False))

        elif draw_center_bound:
            if unique_colors is False:
                center_paths.append(Path(lanelet.center_vertices, closed=False))
            else:
                center_paths = [Path(lanelet.center_vertices, closed=False)]
                ax.add_collection(
                    collections.PathCollection(center_paths, edgecolor=center_bound_color, facecolor='none',
                                               lw=draw_linewidth, zorder=10, antialiased=antialiased))

        is_incoming_lanelet = draw_intersections and draw_incoming_lanelets and\
                              (lanelet.lanelet_id in incoming_lanelets)
        is_crossing = draw_intersections and draw_crossings and\
                              (lanelet.lanelet_id in crossings)
        if fill_lanelet:
            if not is_incoming_lanelet and not is_crossing:
                vertices_fill.append(np.concatenate((lanelet.right_vertices, np.flip(lanelet.left_vertices, 0))))

        # collect incoming lanelets in separate list for plotting in different color
        if is_incoming_lanelet:
            incoming_vertices_fill.append(
                np.concatenate((lanelet.right_vertices, np.flip(lanelet.left_vertices, 0))))
        elif is_crossing:
            crossing_vertices_fill.append(
                np.concatenate((lanelet.right_vertices, np.flip(lanelet.left_vertices, 0))))

        # DRAW LABELS INTO LANELET CENTER
        if show_label or show_intersection_labels or draw_traffic_signs:
            strings = []
            if show_label:
                strings.append(str(lanelet.lanelet_id))
            if is_incoming_lanelet and show_intersection_labels:
                strings.append('inc_id: ' + str(incomings_id[lanelet.lanelet_id]))
                strings.append('inc_left: ' + str(incomings_left[lanelet.lanelet_id]))
            if draw_traffic_signs and show_traffic_sign_label is True:
                traffic_signs_tmp = [traffic_signs[id] for id in lanelet.traffic_signs]
                if traffic_signs_tmp:
                    # add as text to label
                    str_tmp = 'traffic signs: '
                    add_str = ''
                    for sign in traffic_signs_tmp:
                        for el in sign.traffic_sign_elements:
                            #TrafficSignIDGermany(el.traffic_sign_element_id).name would give the name
                            str_tmp += add_str + el.traffic_sign_element_id.value
                            add_str = ', '

                    strings.append(str_tmp)

            label_string = ', '.join(strings)
            if len(label_string) > 0:
                # compute normal angle of label box
                clr_positions = lanelet.interpolate_position(0.5 * lanelet.distance[-1])
                normal_vector = np.array(clr_positions[1]) - np.array(clr_positions[2])
                angle = np.rad2deg(np.arctan2(normal_vector[1], normal_vector[0])) - 90
                angle = angle if Interval(-90,90).contains(angle) else angle - 180

                ax.text(clr_positions[0][0], clr_positions[0][1],
                        label_string,
                        bbox={'facecolor': center_bound_color, 'pad': 2},
                        horizontalalignment='center', verticalalignment='center',
                        rotation=angle,
                        zorder=30.2)

    # draw paths and collect axis handles
    if draw_right_bound:
        ax.add_collection(collections.PathCollection(right_paths, edgecolor=right_bound_color, facecolor='none',
                                                     lw=draw_linewidth, zorder=10, antialiased=antialiased))
    if draw_left_bound:
        ax.add_collection(collections.PathCollection(left_paths, edgecolor=left_bound_color, facecolor='none',
                                                     lw=draw_linewidth, zorder=10, antialiased=antialiased))
    if unique_colors is False:
        if draw_center_bound:
            ax.add_collection(collections.PathCollection(center_paths, edgecolor=center_bound_color, facecolor='none',
                                           lw=draw_linewidth, zorder=10,  antialiased=antialiased))
        if draw_start_and_direction:
            ax.add_collection(collections.PathCollection(direction_list, color=center_bound_color,
                                      lw=0.5, zorder=10.1, antialiased=antialiased))

    if successors_left:
        ax.add_collection(collections.PathCollection(succ_left_paths, edgecolor=successors_left_color, facecolor='none',
                                                     lw=draw_linewidth * 3.0, zorder=11, antialiased=antialiased))
    if successors_straight:
        ax.add_collection(collections.PathCollection(succ_straight_paths, edgecolor=successors_straight_color, facecolor='none',
                                                     lw=draw_linewidth * 3.0, zorder=11, antialiased=antialiased))
    if successors_right:
        ax.add_collection(collections.PathCollection(succ_right_paths, edgecolor=successors_right_color, facecolor='none',
                                                     lw=draw_linewidth * 3.0, zorder=11, antialiased=antialiased))

    # fill lanelets with facecolor
    collection_tmp = collections.PolyCollection(vertices_fill, transOffset=ax.transData, zorder=9.0,
                                                facecolor=facecolor, edgecolor='none', antialiased=antialiased)
    ax.add_collection(collection_tmp)
    if incoming_vertices_fill:
        collection_tmp = collections.PolyCollection(incoming_vertices_fill, transOffset=ax.transData,
                                                    facecolor=incoming_lanelets_color, edgecolor='none', zorder=9.1,
                                                    antialiased=antialiased)
        ax.add_collection(collection_tmp)
    if crossing_vertices_fill:
        collection_tmp = collections.PolyCollection(crossing_vertices_fill, transOffset=ax.transData,
                                                    facecolor=crossings_color, edgecolor='none', zorder=9.2,
                                                    antialiased=antialiased)
        ax.add_collection(collection_tmp)

    # draw_border_vertices
    if draw_border_vertices:
        # left vertices
        collection_tmp = collections.EllipseCollection(np.ones([coordinates_left_border_vertices.shape[0],1]) * 1,
                                                       np.ones([coordinates_left_border_vertices.shape[0],1]) * 1,
                                                       np.zeros([coordinates_left_border_vertices.shape[0],1]),
                                                       offsets=coordinates_left_border_vertices,
                                                       color=left_bound_color, transOffset=ax.transData)
        ax.add_collection(collection_tmp)

        #right_vertices
        collection_tmp = collections.EllipseCollection(np.ones([coordinates_right_border_vertices.shape[0], 1]) * 1,
                                                       np.ones([coordinates_right_border_vertices.shape[0], 1]) * 1,
                                                       np.zeros([coordinates_right_border_vertices.shape[0], 1]),
                                                       offsets=coordinates_right_border_vertices,
                                                       color=right_bound_color, transOffset=ax.transData)
        ax.add_collection(collection_tmp)

    traffic_lights_signs = []
    if draw_traffic_signs:
        # draw actual traffic sign
        traffic_lights_signs.extend(list(traffic_signs.values()))

    if draw_traffic_lights:
        # draw actual traffic sign
        traffic_lights_signs.extend(list(traffic_lights.values()))

    if traffic_lights_signs:
        draw_traffic_light_signs(traffic_lights_signs, None, ax, draw_params, draw_func, handles, call_stack)




def draw_dynamic_obstacles(obj: Union[List[DynamicObstacle],DynamicObstacle],
                           plot_limits: Union[List[Union[int,float]], None], ax: mpl.axes.Axes, draw_params: dict,
                           draw_func: Dict[type,Callable],
                           handles: Dict[Any,List[Union[mpl.patches.Patch,mpl.collections.Collection]]],
                           call_stack: Tuple[str,...]) -> None:
    """
    :param obj: object to be plotted
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """
    def collecting(o: DynamicObstacle):
        occupancy_list=list()
        trajectory = None
        shape = None
        indicators = []
        braking = []
        horns = []
        bluelights = []

        if type(o) is not DynamicObstacle:
            warnings.warn('<visualization/scenario> Only lists with objects of the same type can be plotted',
                          UserWarning, stacklevel=1)
            return (occupancy_list, trajectory, shape)

        # draw occupancies
        if (draw_occupancies == 1 or
                (draw_occupancies == 0 and type(o.prediction) == commonroad.prediction.prediction.SetBasedPrediction)):
            if draw_shape:
                # occupancy already plotted
                time_begin_occ = time_begin + 1
            else:
                time_begin_occ = time_begin

            for time_step in range(time_begin_occ, time_end):
                tmp = o.occupancy_at_time(time_step)
                if tmp is not None:
                    occupancy_list.append(tmp)

        # get trajectory
        if draw_trajectory and type(o.prediction) == commonroad.prediction.prediction.TrajectoryPrediction:
            trajectory = o.prediction.trajectory

        # get shape
        if draw_shape:
            occ = o.occupancy_at_time(time_begin)
            if occ is None:
                shape = None
            else:
                shape = occ.shape

            # draw signal states
            if draw_signals and occ is not None:
                sig = o.signal_state_at_time_step(time_begin)
                if sig is not None:
                    if isinstance(occ.shape, Rectangle):
                        if hasattr(sig, 'hazard_warning_lights') and sig.hazard_warning_lights is True:
                            indicators.extend([occ.shape.vertices[0], occ.shape.vertices[1],  occ.shape.vertices[2],
                                               occ.shape.vertices[3]])
                        else:
                            if hasattr(sig, 'indicator_left') and sig.indicator_left is True:
                                indicators.extend([occ.shape.vertices[1], occ.shape.vertices[2]])
                            if hasattr(sig, 'indicator_right') and sig.indicator_right is True:
                                indicators.extend([occ.shape.vertices[0], occ.shape.vertices[3]])
                        if hasattr(sig, 'braking_lights') and sig.braking_lights is True:
                            braking.extend([occ.shape.vertices[0], occ.shape.vertices[1]])
                        if hasattr(sig, 'flashing_blue_lights') and sig.flashing_blue_lights is True:
                            bluelights.append(occ.shape.center)
                        if hasattr(sig, 'horn') and sig.horn is True:
                            horns.append(occ.shape.center)
                    else:
                        warnings.warn('Plotting signal states only implemented for obstacle_shapes Rectangle.')

        # draw car icon
        if draw_icon and type(o.prediction) == commonroad.prediction.prediction.TrajectoryPrediction:
            if time_begin == 0:
                inital_state = o.initial_state
            else:
                inital_state = o.prediction.trajectory.state_at_time_step(time_begin)
            if inital_state is not None:
                draw_car(inital_state.position[0], inital_state.position[1], inital_state.orientation, 2.5,
                         ax, zorder=30)

        initial_state = None
        if show_label:
            if time_begin == 0:
                initial_state = o.initial_state
                initial_position = initial_state.position
                handles.setdefault(DynamicObstacle, []).append(
                    ax.text(initial_position[0] + 0.5, initial_position[1], str(o.obstacle_id),
                        clip_on=True, zorder=1000))
            elif type(o.prediction) == commonroad.prediction.prediction.TrajectoryPrediction:
                initial_state = o.prediction.trajectory.state_at_time_step(time_begin)
                if initial_state is not None:
                    initial_position = o.prediction.trajectory.state_at_time_step(time_begin).position
                    handles.setdefault(DynamicObstacle, []).append(
                        ax.text(initial_position[0]+0.5, initial_position[1], str(o.obstacle_id),
                            clip_on=True, zorder=1000))

        if draw_initial_state:
            if initial_state is None:
                if time_begin == 0:
                    initial_state = o.initial_state
                elif type(o.prediction) == commonroad.prediction.prediction.TrajectoryPrediction:
                    initial_state = o.prediction.trajectory.state_at_time_step(time_begin)

        return [occupancy_list, trajectory, shape, np.array(indicators).reshape(-1, 2), np.array(braking).reshape(-1, 2),
                np.array(horns).reshape(-1, 2), np.array(bluelights).reshape(-1, 2), initial_state]

    if type(obj) is DynamicObstacle:
        obj = [obj]

    try:
        time_begin = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('time_begin',))
        time_end = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('time_end',))
        draw_icon = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'draw_icon'))
        show_label = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'show_label'))
        draw_shape = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'draw_shape'))
        draw_initial_state = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'initial_state', 'draw_initial_state'))
        scale_factor = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'initial_state', 'scale_factor'))
        kwargs_init_state = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'initial_state', 'kwargs'))
        draw_occupancies = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'occupancy', 'draw_occupancies'))
        draw_signals = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'draw_signals'))
        zorder = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'zorder'))
        signal_radius = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'signal_radius'))
        indicator_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'indicator_color'))
        braking_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'braking_color'))
        horn_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'horn_color'))
        blue_lights_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'blue_lights_color'))
        draw_trajectory = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'trajectory', 'draw_trajectory'))
    except KeyError:
        warnings.warn("Cannot find stylesheet for dynamic_obstacle. Called through:")
        print(call_stack)

    call_stack = tuple(list(call_stack) + ['dynamic_obstacle'])

    # collect objects from all vehicles to draw them efficiently in batches
    occupancy_list, trajectories_list, shapes_list, indicators, braking, horns, bluelights, initial_states\
        = zip(*list(map(collecting, obj)))

    occupancy_list = list(filter(None, list(occupancy_list)))
    trajectories_list = list(filter(None, list(trajectories_list)))
    shapes_list = list(filter(None, list(shapes_list)))
    indicators = np.vstack(indicators)
    braking = np.vstack(braking)
    horns = np.vstack(horns)
    bluelights = np.vstack(bluelights)
    initial_states = list(filter(None, list(initial_states)))

    # draw collected lists, store handles:
    if len(shapes_list) > 0:
        handles.setdefault(DynamicObstacle, []).extend(
            shape_batch_dispatcher(shapes_list, None, ax, draw_params, draw_func, handles, call_stack))
    if len(trajectories_list) > 0:
        handles.setdefault(DynamicObstacle, []).extend(
            commonroad.visualization.draw_dispatch_cr.
                draw_object(trajectories_list, None, ax, draw_params, draw_func, handles, call_stack))
    if len(occupancy_list) > 0:
        handles.setdefault(DynamicObstacle, []).extend(
            commonroad.visualization.draw_dispatch_cr.
                draw_object(occupancy_list, None, ax, draw_params, draw_func, handles, call_stack))

    # draw initial state
    if draw_initial_state is True:
        for initial_state in initial_states:
            handles.setdefault(DynamicObstacle, []).extend(draw_state(initial_state, call_stack=call_stack,
                                                                      ax=ax, draw_params=draw_params,
                                                                      scale_factor=scale_factor,
                                                                      plot_limits=None,
                                                                      arrow_args=kwargs_init_state))

    # draw signals
    if indicators.size > 0:
        diameters = signal_radius * np.ones(indicators.shape[0]) * 2
        handles.setdefault(DynamicObstacle, []).append(
            collections.EllipseCollection(diameters, diameters, angles=np.zeros_like(diameters), offsets=indicators,
                                          transOffset=ax.transData, units='xy', facecolor=indicator_color,
                                          edgecolor=indicator_color, zorder=zorder+0.2, linewidth=0))
        ax.add_collection(handles[DynamicObstacle][-1])
    if braking.size > 0:
        diameters = signal_radius * np.ones(braking.shape[0]) * 3.0
        handles.setdefault(DynamicObstacle, []).append(
            collections.EllipseCollection(diameters, diameters, angles=np.zeros_like(diameters), offsets=braking,
                                          transOffset=ax.transData, units='xy', facecolor=braking_color,
                                          edgecolor=braking_color, zorder=zorder + 0.1, linewidth=0))
        ax.add_collection(handles[DynamicObstacle][-1])
    if horns.size > 0:
        diameters = signal_radius * np.ones(horns.shape[0]) * 3.0
        handles.setdefault(DynamicObstacle, []).append(
            collections.EllipseCollection(diameters, diameters, angles=np.zeros_like(diameters), offsets=horns,
                                          transOffset=ax.transData, units='xy', facecolor=horn_color,
                                          edgecolor=braking_color, zorder=zorder + 0.1, linewidth=0))
        ax.add_collection(handles[DynamicObstacle][-1])
    if bluelights.size > 0:
        diameters = signal_radius * np.ones(bluelights.shape[0]) * 2
        handles.setdefault(DynamicObstacle, []).append(
            collections.EllipseCollection(diameters, diameters, angles=np.zeros_like(diameters), offsets=bluelights,
                                          transOffset=ax.transData, units='xy', facecolor=blue_lights_color,
                                          edgecolor=braking_color, zorder=zorder + 0.1, linewidth=0))
        ax.add_collection(handles[DynamicObstacle][-1])


def draw_occupancies(obj: Union[List[Occupancy], Occupancy], plot_limits: Union[List[Union[int,float]], None],
                     ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable],
                     handles: Dict[int,List[mpl.patches.Patch]], call_stack: Tuple[str,...]) -> List[mpl.collections.Collection]:
    """
    :param obj: object to be plotted
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """
    call_stack = tuple(list(call_stack) + ['occupancy'])

    if type(obj) is Occupancy:
        obj = [obj]

    # collect occupancy shape(s)
    shape_list = list()
    if not 'dynamic_obstacle' in call_stack:
        time_begin = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('time_begin',))
        time_end = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('time_end',))

        for occupancy in obj:
            if time_begin <= occupancy.time_step <= time_end:
                shape_list.append(occupancy.shape)
    else:
        for occupancy in obj:
            shape_list.append(occupancy.shape)

    patch_list = shape_batch_dispatcher(shape_list, None, ax, draw_params, draw_func, handles, call_stack)

    return patch_list


def draw_shape_group(obj: ShapeGroup, plot_limits: Union[List[Union[int,float]], None], ax: mpl.axes.Axes,
                     draw_params: dict, draw_func: Dict[type,Callable], handles: Dict[int,List[mpl.patches.Patch]],
                     call_stack: Tuple[str,...]) -> List[mpl.patches.Patch]:
    """
    :param obj: object to be plotted
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """
    patch_list: List[mpl.patches.Patch] = list()

    for shape in obj.shapes:
        patch = commonroad.visualization.draw_dispatch_cr.draw_object(
                shape, None, ax, draw_params, draw_func, handles, call_stack)
        patch_list.extend(patch)

    return patch_list


def shape_batch_dispatcher(obj: List[Shape], plot_limits: Union[List[Union[int,float]], None], ax: mpl.axes.Axes,
                           draw_params: dict, draw_func: Dict[type,Callable], handles: Dict[int,List[mpl.patches.Patch]],
                           call_stack: Tuple[str,...]) -> List[mpl.collections.Collection]:
    """
    Groups a list of shapes by their type and draws them each in a batch.
    :param obj: list of shapes to be plotted
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """
    shapes_ordered_by_type = dict()
    
    # sorts shapes by type
    for shape in obj:
        if shape is not None:
            shapes_ordered_by_type.setdefault(type(shape), []).append(shape)

    collection_list = list()
    # plots each type
    for shape_list_tmp in shapes_ordered_by_type.values():
        collection_tmp = commonroad.visualization.draw_dispatch_cr.draw_object(shape_list_tmp, None, ax, draw_params,
                                                                               draw_func, handles, call_stack)
        collection_list.extend(collection_tmp)

    return collection_list


def draw_polygons(obj: Polygon, plot_limits: Union[List[Union[int,float]], None], ax: mpl.axes.Axes,
                  draw_params: dict, draw_func: Dict[type,Callable],
                  handles: Dict[int,List[mpl.patches.Patch]], call_stack: Tuple[str,...])\
        -> List[mpl.collections.Collection]:
    """
    :param obj: object to be plotted
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """
    if type(obj) is Polygon:
        obj = [obj]

    call_stack = tuple(list(call_stack) + ['shape','polygon'])
    try:
        facecolor = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['facecolor']))
        edgecolor = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['edgecolor']))
        zorder = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['zorder']))
        opacity = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['opacity']))
        linewidth = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['linewidth']))
        antialiased = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, tuple(),
            tuple(['antialiased']))
    except KeyError:
        print("Cannot find stylesheet for polygon. Called through:")
        print(call_stack)

    vertices_list = list()

    for poly in obj:
        if type(poly) is not Polygon:
            warnings.warn('<visualization/scenario> Only lists with objects of the same type can be plotted',
                          UserWarning, stacklevel=1)
            continue
        vertices_list.append(np.array(poly.vertices))
    collection = draw_polygon_collection_as_patch(vertices_list, ax, zorder=zorder,
                          facecolor=facecolor,
                          edgecolor=edgecolor, lw=linewidth, alpha=opacity, antialiased=antialiased)
    return [collection]


def draw_circle(obj:  Union[Circle,List[Circle]], plot_limits: Union[List[Union[int,float]], None],
                ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable],
                handles: Dict[int,List[mpl.patches.Patch]], call_stack: Tuple[str,...]) -> List[mpl.collections.Collection]:
    """
    :param obj: object to be plotted
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """
    if type(obj) is Circle:
        obj: List[Circle] =[obj]
        
    call_stack = tuple(list(call_stack) + ['shape', 'circle'])
    try:
        facecolor = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['facecolor']))
        edgecolor = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['edgecolor']))
        zorder = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['zorder']))
        opacity = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['opacity']))
        linewidth = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['linewidth']))
    except KeyError:
        print("Cannot find stylesheet for circle. Called through:")
        print(call_stack)

    centers = list()
    radii = list()
    for circle in obj:
        if type(circle) is not Circle:
            warnings.warn('<visualization/scenario> Only lists with objects of the same type can be plotted',
                          UserWarning, stacklevel=1)
            continue
        centers.append(circle.center)
        radii.append(circle.radius)

    centers = np.array(centers)
    diameters = np.array(radii) * 2
    collection = collections.EllipseCollection(diameters, diameters, angles=np.zeros_like(radii), offsets=centers, transOffset=ax.transData,  units='xy', facecolor=facecolor,
                                              edgecolor=edgecolor, zorder=zorder, linewidth=linewidth, alpha=opacity,)

    ax.add_collection(collection)
    
    return [collection]


def draw_rectangle(obj: Union[Rectangle,List[Rectangle]], plot_limits: Union[List[Union[int,float]], None],
                   ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable],
                   handles: Dict[int,List[mpl.patches.Patch]], call_stack: Tuple[str,...]) -> List[mpl.collections.Collection]:
    """
    :param obj: object to be plotted
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """
    if type(obj) is Rectangle:
        obj =[obj]

    call_stack = tuple(list(call_stack) + ['shape', 'rectangle'])
    try:
        facecolor = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['facecolor']))
        edgecolor = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['edgecolor']))
        zorder = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['zorder']))
        opacity = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['opacity']))
        linewidth = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['linewidth']))
        antialiased = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, tuple(),
            tuple(['antialiased']))
    except KeyError:
        print("Cannot find stylesheet for rectangle. Called through:")
        print(call_stack)

    vertices = list()

    for rect in obj:
        if type(rect) is not Rectangle:
            warnings.warn('<visualization/scenario> Only lists with objects of the same type can be plotted',
                          UserWarning, stacklevel=1)
            continue
        vertices.append(np.array(rect.vertices))

    collection = collections.PolyCollection(vertices, closed=True, zorder=zorder, transOffset=ax.transData,
                                            facecolor=facecolor, edgecolor=edgecolor, alpha=opacity, antialiased=antialiased, linewidth=linewidth)
    ax.add_collection(collection)

    return [collection]


def draw_car(pos_x: Union[int,float], pos_y: Union[int,float], rotate: Union[int,float],
             scale: Union[int,float],  ax: mpl.axes.Axes, zorder: int = 5, carcolor: str ='#ffffff',
             lw=0.5):
    rotate = rotate + np.pi

    def reshape_and_addup(verts):
        verts = verts.reshape((int(len(verts)/2), 2))
        for i in range(1, len(verts)):
            verts[i] = verts[i]+verts[i-1]
        return verts

    def transform(vertices, pos_x, pos_y, rotate, scale):
        vertices = np.asarray(np.bmat([vertices, np.ones((len(vertices), 1))]))
        tmat = np.array([[np.cos(rotate), -np.sin(rotate), pos_x],
                         [np.sin(rotate), np.cos(rotate), pos_y], [0, 0, 1]])
        scalemat = np.array([[scale*1/356.3211, 0, 0], [0, scale*1/356.3211, 0],
                             [0, 0, 1]])
        centermat = np.array([[1, 0, -250], [0, 1, -889], [0, 0, 1]])
        tmat = tmat.dot(scalemat.dot(centermat))
        vertices = tmat.dot(vertices.transpose())
        vertices = np.array([[1, 0, 0], [0, 1, 0]]).dot(vertices).transpose()
        return vertices

    verts1 = np.array([193.99383, 752.94359,
                       -22.66602, 38,
                       -9.33398, 52.66797,
                       -2.60743, 44.82812,
                       -0.0586, 0,
                       0.0293, 0.91992,
                       -0.0293, 0.91797,
                       0.0586, 0,
                       2.60743, 44.82813,
                       9.33398, 52.66797,
                       22.66602, 38.00003,
                       59.33398, -7.334,
                       62, -10.666,
                       -6, -50.00003,
                       -2.65234, -68.41407,
                       2.65234, -68.41601,
                       6, -50,
                       -62, -10.66602])

    verts2 = np.array([715.99381, 768.27757,
                       -101.332, 6.66602,
                       10.666, 62.66797,
                       3.3223, 50.41797,
                       -3.3223, 50.41796,
                       -10.666, 62.66601,
                       101.332, 6.668,
                       22, -42.66799,
                       9.9824, -76.83594,
                       # 0.018,0,
                       # -0.01,-0.24804,
                       # 0.01,-0.24805,
                       # -0.018,0,
                       -9.9824, -76.83789,
                       0, 0])

    verts3 = np.array([421.06111, 751.61113,
                       190.2667, 3.33333,
                       13.3333, 5.33334,
                       -108.6666, 12.66666,
                       -134, 0,
                       -119.3334, -18.66666,
                       127.6456, -2.96473])

    verts4 = np.array([271.32781, 712.14446,
                       -6, -0.8,
                       -16, 12,
                       -14.8, 19.2,
                       -4, 6,
                       20.4, 0.4,
                       3.6, -4.4,
                       4.8, -2.8])
    verts5 = np.array([191.32781, 753.94359,
                       -99.999996, 11,
                       -63, 18.5,
                       -59, 38.5,
                       -20, 77,
                       20, 59.52734,
                       57, 36.49998,
                       65, 20.49999,
                       99.999996, 11.0001])

    verts6 = np.array([421.06111, 1027.399,
                       190.2667, -3.3333,
                       13.3333, -5.3333,
                       -108.6666, -12.6667,
                       -134, 0,
                       -119.3334, 18.6667,
                       127.6456, 2.9647])

    verts7 = np.array([271.32781, 1066.8657,
                       -6, 0.8,
                       -16, -12,
                       -14.8, -19.2,
                       -4, -6,
                       20.4, -0.4,
                       3.6, 4.4,
                       4.8, 2.8])

    verts8 = np.array([389.79851, 728.34788,
                       -343.652396, 10.16016,
                       -68.666, 22.42969,
                       -29.2558, 74.57031,
                       -7.3164, 60.35742,
                       -0.074, 0,
                       0.037, 0.76758,
                       -0.037, 0.76758,
                       0.074, 0,
                       7.3164, 42.35937,
                       29.2558, 74.57031,
                       68.666, 22.4278,
                       343.652396, 10.1621,
                       259.5859, -4.6192,
                       130.2539, -17.5527,
                       24.0196, -18.4766,
                       17.5527, -65.58788,
                       3.6953, -37.42773,
                       0, -13.24414,
                       -3.6953, -55.42774,
                       -17.5527, -65.58984,
                       -24.0196, -18.47656,
                       -130.2539, -17.55274])

    verts1 = reshape_and_addup(verts1)  # vorderscheibe
    verts2 = reshape_and_addup(verts2)  # rueckscheibe
    verts3 = reshape_and_addup(verts3)  # seitenscheibe rechts
    verts4 = reshape_and_addup(verts4)  # rueckspiegel links
    verts5 = reshape_and_addup(verts5)  # motorhaube
    verts6 = reshape_and_addup(verts6)  # fenster rechts
    verts7 = reshape_and_addup(verts7)  # rueckspiegel rechts
    verts8 = reshape_and_addup(verts8)  # umriss

    verts1 = transform(verts1, pos_x, pos_y, rotate, scale)
    verts2 = transform(verts2, pos_x, pos_y, rotate, scale)
    verts3 = transform(verts3, pos_x, pos_y, rotate, scale)
    verts4 = transform(verts4, pos_x, pos_y, rotate, scale)
    verts5 = transform(verts5, pos_x, pos_y, rotate, scale)
    verts6 = transform(verts6, pos_x, pos_y, rotate, scale)
    verts7 = transform(verts7, pos_x, pos_y, rotate, scale)
    verts8 = transform(verts8, pos_x, pos_y, rotate, scale)

    windowcolor = '#555555'
    draw_polygon_as_patch(verts1, ax, facecolor=windowcolor, zorder=zorder + 1,
                          lw=lw)
    draw_polygon_as_patch(verts2, ax, facecolor=windowcolor, zorder=zorder + 1,
                          lw=lw)
    draw_polygon_as_patch(verts3, ax, facecolor=windowcolor, zorder=zorder + 1,
                          lw=lw)
    draw_polygon_as_patch(verts4, ax, facecolor='#ffffff', zorder=zorder + 1,
                          lw=lw)
    ax.plot(verts5[:, 0], verts5[:, 1], zorder=zorder+1, color='#000000',
            lw=lw)
    draw_polygon_as_patch(verts6, ax, facecolor=windowcolor, zorder=zorder + 1,
                          lw=lw)
    draw_polygon_as_patch(verts7, ax, facecolor='#ffffff', zorder=zorder + 1,
                          lw=lw)
    draw_polygon_as_patch(verts8, ax, facecolor=carcolor, edgecolor='#000000',
                          zorder=zorder, lw=lw)


def draw_state(state: State, plot_limits: Union[List[Union[int,float]], None],
               ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable] = None,
               handles: Dict[int,List[mpl.patches.Patch]] = None, call_stack: Tuple[str,...] = None,
               scale_factor=None,
               arrow_args=None) -> List[mpl.collections.Collection]:
    """
    :param state: state to be plotted
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
    :param draw_func: specifies the drawing function
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object
    :return: None
    """
    try:
        if scale_factor is None:
            scale_factor = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('initial_state', 'scale_factor'))
        if arrow_args is None:
            arrow_args = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,
                ('initial_state', 'kwargs'))
    except KeyError:
        print("Cannot find stylesheet for initial_state. Called through:")
        print(call_stack)
        return []

    cos = math.cos(state.orientation)
    sin = math.sin(state.orientation)
    length = 1.5
    x = state.position[0] + cos * length
    y = state.position[1] + sin * length
    artist = ax.arrow(x=x, y=y,
                     dx=state.velocity * cos * scale_factor,
                     dy=state.velocity * sin * scale_factor,
                     zorder=100,
                     **arrow_args)
    return [artist]


# default function dict, which assigns drawing functions to object classes
draw_func_dict = {commonroad.scenario.scenario.Scenario: draw_scenario,
                  commonroad.scenario.lanelet.Lanelet: draw_lanelet_list,
                  commonroad.scenario.lanelet.LaneletNetwork: draw_lanelet_network,
                  commonroad.scenario.traffic_sign.TrafficSign: draw_traffic_light_signs,
                  commonroad.scenario.traffic_sign.TrafficLight: draw_traffic_light_signs,
                  commonroad.scenario.obstacle.DynamicObstacle: draw_dynamic_obstacles,
                  commonroad.scenario.obstacle.StaticObstacle: draw_static_obstacles,
                  commonroad.scenario.trajectory.Trajectory: draw_trajectories,
                  commonroad.geometry.shape.ShapeGroup: draw_shape_group,
                  commonroad.geometry.shape.Polygon: draw_polygons,
                  commonroad.geometry.shape.Circle: draw_circle,
                  commonroad.geometry.shape.Rectangle: draw_rectangle,
                  commonroad.prediction.prediction.Occupancy: draw_occupancies}
