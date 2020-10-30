import sys
import warnings
from typing import Dict, Callable, Tuple, Any

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import commonroad
from commonroad.geometry.shape import *
from commonroad.planning.planning_problem import GoalRegion, PlanningProblemSet, \
    PlanningProblem
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
from commonroad.scenario.obstacle import *
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_sign import TrafficSign
from commonroad.scenario.trajectory import Trajectory, State
import commonroad.visualization.scenario
import commonroad.visualization.planning

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = [""]
__version__ = "2020.3"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


def _create_drawers_dict() -> Dict[type, Callable]:
    """
    Creates default dict of drawer functions, which are later called for draw objects depending on their class

    :return: dict containing the drawer functions
    """

    draw_func = {}
    if 'commonroad.visualization.scenario' in sys.modules.keys():
        draw_func.update(commonroad.visualization.scenario.draw_func_dict)
    if 'commonroad.visualization.planning' in sys.modules.keys():
        draw_func.update(commonroad.visualization.planning.draw_func_dict)

    return draw_func


def _create_default_draw_params() -> dict:
    """
    Creates default settings for drawing objects

    :return: nested dict, containing the settings
    """

    draw_params = {'time_begin': 0, 'time_end': 200, 'antialiased': True}
    if 'commonroad.visualization.scenario' in sys.modules.keys():
        draw_params.update(
            commonroad.visualization.scenario.create_default_draw_params())
    if 'commonroad.visualization.planning' in sys.modules.keys():
        draw_params.update(
            commonroad.visualization.planning.create_default_draw_params())
    return draw_params


default_draw_params = _create_default_draw_params()


def _retrieve_value_by_path(style_sheet_caller: dict, value_path:  Tuple[str, ...]):
    """
    Retrieves value corresponding to value path from the nested dict style_sheet.

    :param style_sheet_caller: parameters for plotting given by a nested dict
    :param value_path: string tuple that contains the path to the value
    :return: the value from style_sheet defined by value_path
    """

    c_dict = style_sheet_caller
    for value_element in value_path[:-1]:
        try:
            c_dict = c_dict[value_element]
        except KeyError:
            raise KeyError()
    try:
        return_value = c_dict[value_path[-1]]
    except KeyError:
        raise KeyError()
    return return_value


def _retrieve_value(style_sheet: dict, call_stack: tuple, value_path: Tuple[str, ...]):
    """
    Retrieves value corresponding to value_path from the nested dict style_sheet. If value_path not contained in
    style_sheet, try to retrieve from default draw_params.
    Starts by searching for value_path beginning at first element in call stack, stepping down all frames

    :param style_sheet: parameters for plotting given by a nested dict
    :param call_stack: tuple of string containing the call stack
    :param value_path: string tuple that contains the path to the value
    :return: the value from style_sheet defined by value_path and call_stack
    """

    # first: try to retrieve value with call stack, start from deepest level
    for idx_r in range(0,len(call_stack)):
        style_sheet_caller = style_sheet
        try:
            for idx in range(0,len(call_stack)-idx_r):
                style_sheet_caller = style_sheet_caller[call_stack[idx]]
            try:
                value = _retrieve_value_by_path(style_sheet_caller, value_path)
                return value
            except KeyError:
                pass
        except KeyError:
            continue
    # try to retrieve value without call_stack (for fallback to top-level parameters)
    try:
        value = _retrieve_value_by_path(style_sheet, value_path)
        return value
    except KeyError:
        pass

    # try to retrieve from default parameters
    for idx_r in range(0,len(call_stack)):
        style_sheet_caller = default_draw_params
        try:
            for idx in range(0,len(call_stack)-idx_r):
                style_sheet_caller = style_sheet_caller[call_stack[idx]]
            try:
                value = _retrieve_value_by_path(style_sheet_caller, value_path)
                return value
            except KeyError:
                pass
        except KeyError:
            continue
    try:
        value = _retrieve_value_by_path(default_draw_params, value_path)
        return value
    except KeyError:
        pass

    # if value not found yet, it is neither given in default parameters nor in provided parameters,
    # possibly a wrong call_stack was provided
    try:
        value
    except NameError:
        raise KeyError
    return value


def _retrieve_alternate_value(style_sheet: dict, call_stack: Tuple[str, ...], value_path_1: Tuple[str, ...],
                              value_path_2: Tuple[str,...]) -> Any:
    """
    Like retrieve_value(...), but retrieves value from value_path_2 if value_path_1 does not exist in style_sheet

    :param style_sheet: parameters for plotting given by a nested dict (see draw_params in draw_object)
    :param call_stack: tuple of string containing the call stack
    :param value_path_1: string tuple that contains the path to the value
    :param value_path_2: alternate value_path
    :return: the value from style_sheet defined by value_path_1 (or value_path_2)
    """
    # first: try to retrieve value with call stack, start from deepest level
    for idx_r in range(0, len(call_stack)):
        style_sheet_caller = style_sheet
        try:
            for idx in range(0, len(call_stack) - idx_r):
                style_sheet_caller = style_sheet_caller[call_stack[idx]]
            try:
                value = _retrieve_value_by_path(style_sheet_caller, value_path_1)
                return value
            except KeyError:
                pass
        except KeyError:
            continue

    for idx_r in range(0, len(call_stack)):
        style_sheet_caller = style_sheet
        try:
            for idx in range(0, len(call_stack) - idx_r):
                style_sheet_caller = style_sheet_caller[call_stack[idx]]
            try:
                value = _retrieve_value_by_path(style_sheet_caller, value_path_2)
                return value
            except KeyError:
                pass
        except KeyError:
            continue
    # try to retrieve value without call_stack (for fallback to top-level parameters)
    try:
        value = _retrieve_value_by_path(style_sheet, value_path_1)
        return value
    except KeyError:
        pass

    try:
        value = _retrieve_value_by_path(style_sheet, value_path_2)
        return value
    except KeyError:
        pass

    # try to retrieve from default parameters
    try:
        value = _retrieve_value_by_path(default_draw_params, call_stack + value_path_1)
        return value
    except KeyError:
        pass

    try:
        value = _retrieve_value_by_path(default_draw_params, call_stack + value_path_2)
        return value
    except KeyError:
        pass

    try:
        value = _retrieve_value_by_path(default_draw_params, value_path_1)
        return value
    except KeyError:
        pass

    try:
        value = _retrieve_value_by_path(default_draw_params, value_path_2)
        return value
    except KeyError:
        pass

    # if value not found yet, it is neither given in default parameters nor in provided parameters,
    # possibly a wrong call_stack was provided
    try:
        value
    except NameError:
        raise KeyError
    return value


def _add_legend(legend: Dict[Tuple[str,...], str], draw_params):
    """
    Adds legend with color of objects specified by legend.keys() and texts specified by legend.values().
    :param legend: color of objects specified by path in legend.keys() and texts specified by legend.values()
    :param draw_params: draw parameters used for plotting (color is extracted using path in legend.keys())
    :return:
    """
    handles = []
    for obj_name, text in legend.items():
        try:
            color = _retrieve_value(draw_params, (), obj_name)
        except:
            color = None
        if color is not None:
            handles.append(mpatches.Patch(color=color, label=text))

    l = plt.legend(handles=handles)
    l.set_zorder(1000)


plottable_types=Union[list, Scenario, Trajectory, LaneletNetwork, Lanelet, Obstacle, ShapeGroup, Shape,
                      GoalRegion, PlanningProblem, PlanningProblemSet, State, Occupancy, TrafficSign]

def draw_object(obj: Union[plottable_types, List[plottable_types]],
                plot_limits: Union[List[Union[float,int]],None] = None,
                ax: Union[None, mpl.axes.Axes] = None,
                draw_params: Union[None, dict] = None,
                draw_func: Union[None, Dict[type,Callable]] = None,
                handles: Dict[int, List[mpl.patches.Patch]] = None,
                call_stack: Union[None,Tuple[str,...]] = None,
                legend: Union[Dict[Tuple[str,...],str],None]=None)-> Union[None, List[mpl.patches.Patch]]:
    """
    Main function for drawing objects from the scenario and planning modules.

    :param obj: the object or list of objects (with all the same type) to be plotted
    :param plot_limits:  list of [x_min, x_max, y_min, y_max that defines the plotted area of the scenario
    :param ax: axes object from matplotlib
    :param draw_params: parameters for plotting given by a nested dict that recreates the structure of an object,
           see documentation for full overview over the structure. If parameters are not set,
           the default setting are used. An example for specifying the color of circle obstacles:
           {'scenario': {'static_obstacle':{'shape':{'circle':{'facecolor':#3399ff, edgecolor': '#0066cc'}}}}}
    :param draw_func: specify the drawing function (usually not necessary to change default)
    :param handles: dict that assign to every object_id of all plotted obstacles the corresponding patch handles
    :param call_stack: tuple of string containing the call stack, which allows for differentiation of plotting styles
           depending on the call stack of draw_object, (usually 'None'!)
    :param legend: names of objects that should appear in the legend
    :return: Returns matplotlib patch object for draw_funcs that actually draw a patch (used internally for creating handles dict)
    """

    assert isinstance(draw_params,dict) or draw_params is None, '<visualization/draw_dispatch_cr/draw_object>:'\
        'draw_params must be either a (nested) dictionary contating the plotting parameters'\
                                                                'or None (for default parameters)'

    assert isinstance(plot_limits, (list,np.ndarray)) or plot_limits is None, '<visualization/draw_dispatch_cr/draw_object>:'\
                                                                  'plot_limits must be a list of [x_min, x_max, y_min, y_max]'\
                                                                  'or None'
    assert isinstance(call_stack, tuple) or call_stack is None

    if handles is None:
        handles = dict()

    if ax is None:
        ax = plt.gca()

    if plot_limits is not None:
        assert np.less(plot_limits[0], plot_limits[1]) and np.less(plot_limits[2], plot_limits[3]),\
            '<draw_dispatch_cr/draw_object>: position limits need to be given by [x_min, x_max, y_min, y_max]'
        ax.set_xlim(plot_limits[0], plot_limits[1])
        ax.set_ylim(plot_limits[2], plot_limits[3])

    if draw_func is None:
        draw_func = _create_drawers_dict()

    if draw_params is None:
        draw_params = _create_default_draw_params()

    if call_stack is None:
        call_stack = tuple()

    if legend is not None:
        _add_legend(legend, draw_params)

    if type(obj) is list:
        if len(obj)==0:
            return []
        elif isinstance(obj[0], (DynamicObstacle, Trajectory, Rectangle, Polygon, Lanelet, Circle)):
            patch_list = draw_func[type(obj[0])](obj, plot_limits, ax, draw_params, draw_func, handles, call_stack)
            return patch_list
        else:
            patch_list = list()
            for o in obj:
                patch_list_tmp = draw_object(o, None, ax, draw_params, draw_func, handles, call_stack)
                if patch_list_tmp is not None:
                    patch_list.extend(patch_list_tmp)
            return patch_list

    if type(obj) in draw_func.keys():
        # plotting the object
        patch_list = draw_func[type(obj)](obj, plot_limits, ax, draw_params, draw_func, handles, call_stack)
        return patch_list
    else:
        for obj_type, func in draw_func.items():
            if isinstance(obj,obj_type):
                patch_list = func(obj, plot_limits, ax, draw_params, draw_func, handles, call_stack)
                return patch_list
        # else: no draw function available
        warnings.warn("Cannot dispatch to plot " + str(type(obj)))
