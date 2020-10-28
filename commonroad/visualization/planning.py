import commonroad.visualization.draw_dispatch_cr
import commonroad.planning.planning_problem
import commonroad.visualization.draw_dispatch_cr
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem, GoalRegion
from commonroad.scenario.trajectory import State
import matplotlib as mpl
from typing import Union, List, Dict, Callable, Tuple

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = [""]
__version__ = "2020.3"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


def create_default_draw_params():
    draw_params_noparent = {'initial_state': {
                                'facecolor': '#000080',
                                'zorder': 25,
                                'label': '' # text for labeling this state, i.r. 'initial position'
                                },
                            'goal_region': {'draw_shape': True,
                                            'shape': {
                                                'polygon': {
                                                    'opacity': 1.0,
                                                    'linewidth': 0.5,
                                                    'facecolor': '#f1b514',
                                                    'edgecolor': '#302404',
                                                    'zorder': 15,
                                                },
                                                'rectangle': {
                                                    'opacity': 1.0,
                                                    'linewidth': 0.5,
                                                    'facecolor': '#f1b514',
                                                    'edgecolor': '#302404',
                                                    'zorder': 15,
                                                },
                                                'circle': {
                                                    'opacity': 1.0,
                                                    'linewidth': 0.5,
                                                    'facecolor': '#f1b514',
                                                    'edgecolor': '#302404',
                                                    'zorder': 15,
                                                }},
                                            'lanelet': {'left_bound_color': '#555555',
                                                        'right_bound_color': '#555555',
                                                        'center_bound_color': '#dddddd',
                                                        'draw_left_bound': True,
                                                        'draw_right_bound': True,
                                                        'draw_center_bound': True,
                                                        'draw_border_vertices': False,
                                                        'draw_start_and_direction': True,
                                                        'show_label': False,
                                                        'draw_linewidth': 0.5,
                                                        'fill_lanelet': True,
                                                        'facecolor': '#c7c7c7'
                                            }
                            }
    }

    draw_params = {'draw_ids': 'all',  # either 'all' or list of problem ids
                   'initial_state': {'facecolor': '#000080',
                                     'zorder': 25,
                                     'label': ''},  # text for labeling this state, i.r. 'initial position'
                   'goal_region':  {'draw_shape': True,
                                    'shape': {
                                        'polygon': {
                                            'opacity': 1.0,
                                            'linewidth': 0.5,
                                            'facecolor': '#f1b514',
                                            'edgecolor': '#302404',
                                            'zorder': 15,
                                        },
                                        'rectangle': {
                                            'opacity': 1.0,
                                            'linewidth': 0.5,
                                            'facecolor': '#f1b514',
                                            'edgecolor': '#302404',
                                            'zorder': 15,
                                        },
                                        'circle': {
                                            'opacity': 1.0,
                                            'linewidth': 0.5,
                                            'facecolor': '#f1b514',
                                            'edgecolor': '#302404',
                                            'zorder': 15,
                                        }},
                                        'lanelet': {'left_bound_color': '#555555',
                                                   'right_bound_color': '#555555',
                                                   'center_bound_color': '#dddddd',
                                                   'draw_left_bound': True,
                                                   'draw_right_bound': True,
                                                   'draw_center_bound': True,
                                                   'draw_border_vertices': False,
                                                   'draw_start_and_direction': True,
                                                   'show_label': False,
                                                   'draw_linewidth': 0.5,
                                                   'fill_lanelet': True,
                                                   'facecolor': '#c7c7c7'}
                                    },
                   'planning_problem': {
                       'initial_state': {'facecolor': '#000080',
                                         'zorder': 25,
                                         'label': ''},  # text for labeling this state, i.r. 'initial position'
                       'goal_region': {'draw_shape': True,
                                       'shape': {
                                           'polygon': {
                                               'opacity': 1.0,
                                               'linewidth': 0.5,
                                               'facecolor': '#f1b514',
                                               'edgecolor': '#302404',
                                               'zorder': 15,
                                           },
                                           'rectangle': {
                                               'opacity': 1.0,
                                               'linewidth': 0.5,
                                               'facecolor': '#f1b514',
                                               'edgecolor': '#302404',
                                               'zorder': 15,
                                           },
                                           'circle': {
                                               'opacity': 1.0,
                                               'linewidth': 0.5,
                                               'facecolor': '#f1b514',
                                               'edgecolor': '#302404',
                                               'zorder': 15,
                                           }},
                                        'lanelet': {'left_bound_color': '#555555',
                                                   'right_bound_color': '#555555',
                                                   'center_bound_color': '#dddddd',
                                                   'draw_left_bound': True,
                                                   'draw_right_bound': True,
                                                   'draw_center_bound': True,
                                                   'draw_border_vertices': False,
                                                   'draw_start_and_direction': True,
                                                   'show_label': False,
                                                   'draw_linewidth': 0.5,
                                                   'fill_lanelet': True,
                                                   'facecolor': '#c7c7c7'}
                                       }
                   }
    }
    draw_params_noparent.update({'planning_problem_set': draw_params})
    draw_params_noparent['planning_problem'] = draw_params_noparent['planning_problem_set']['planning_problem']
    return draw_params_noparent


def draw_planning_problem_set(obj: PlanningProblemSet, plot_limits: List[Union[int,float]], ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable], handles: Dict[int,List[mpl.patches.Patch]],
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
    call_stack = tuple(list(call_stack) + ['planning_problem_set'])
    try:
        draw_ids = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            tuple(['draw_ids']))
    except KeyError:
        print("Cannot find stylesheet for planning_problem. Called through:")
        print(call_stack)

    for id, problem in obj.planning_problem_dict.items():
        if draw_ids is 'all' or id in draw_ids:
            draw_planning_problem(problem,plot_limits, ax, draw_params, draw_func, handles, call_stack)


def draw_planning_problem(obj: PlanningProblem, plot_limits: List[Union[int,float]], ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable], handles: Dict[int,List[mpl.patches.Patch]],
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
    call_stack = tuple(list(call_stack) + ['planning_problem'])
    if not 'initial_state' in draw_params:
        draw_params['initial_state'] = {}
    draw_params['initial_state']['label'] = 'initial position'
    draw_initital_state(obj.initial_state,plot_limits, ax, draw_params, draw_func, handles, call_stack)
    draw_goal_region(obj.goal,plot_limits, ax, draw_params, draw_func, handles, call_stack)


def draw_initital_state(obj: State, plot_limits: List[Union[int,float]], ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable], handles: Dict[int,List[mpl.patches.Patch]],
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
    try:
        facecolor = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('initial_state', 'facecolor'))
        zorder = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('initial_state', 'zorder'))
        label = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('initial_state', 'label'))

    except KeyError:
        print("Cannot find stylesheet for state. Called through:")
        print(call_stack)
        facecolor = '#f1b514'
        zorder = 20
        label = 'intial position'

    ax.plot(obj.position[0], obj.position[1], 'o', color=facecolor, zorder=zorder, markersize=3)
    ax.annotate(label, xy=(obj.position[0]+1, obj.position[1]),
                textcoords='data', zorder=zorder+10)


def draw_goal_region(obj: GoalRegion, plot_limits: List[Union[int,float]], ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable], handles: Dict[int,List[mpl.patches.Patch]],
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
    if call_stack is ():
        call_stack = tuple(['planning_problem_set'])
    call_stack = tuple(list(call_stack) + ['goal_region'])
    for goal_state in obj.state_list:
        draw_goal_state(goal_state,plot_limits, ax, draw_params, draw_func, handles, call_stack)


def draw_goal_state(obj: State, plot_limits: List[Union[int,float]], ax: mpl.axes.Axes, draw_params: dict, draw_func: Dict[type,Callable], handles: Dict[int,List[mpl.patches.Patch]],
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
    if hasattr(obj, 'position'):
        if type(obj.position) == list:
            for pos in obj.position:
                commonroad.visualization.draw_dispatch_cr.draw_object(pos, plot_limits, ax, draw_params,
                                                                      draw_func, handles, call_stack)
        else:
            commonroad.visualization.draw_dispatch_cr.draw_object(obj.position, plot_limits, ax, draw_params,
                                                                  draw_func, handles, call_stack)

# default function dict, which assigns drawing functions to object classes
draw_func_dict = {commonroad.planning.planning_problem.GoalRegion: draw_goal_region,
                  commonroad.planning.planning_problem.PlanningProblem: draw_planning_problem,
                  commonroad.planning.planning_problem.PlanningProblemSet: draw_planning_problem_set,
                  State: draw_initital_state}
