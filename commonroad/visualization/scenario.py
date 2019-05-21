from typing import Dict, Callable, Tuple, Union, Any
import commonroad.geometry.shape
import matplotlib as mpl
import matplotlib.patches as patches
import matplotlib.collections as collections
import commonroad.prediction.prediction
import commonroad.scenario.obstacle
import commonroad.visualization.draw_dispatch_cr
from commonroad.common.util import Interval
from commonroad.geometry.shape import *
from matplotlib.path import Path
from commonroad.prediction.prediction import Occupancy
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet
from commonroad.scenario.obstacle import DynamicObstacle, StaticObstacle, ObstacleRole
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory

from commonroad.visualization.util import draw_polygon_as_patch, draw_polygon_collection_as_patch


__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = [""]
__version__ = "2019.1"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad@in.tum.de"
__status__ = "Released"


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
                            'show_label': False,
                            'trajectory_steps': 40,
                            'zorder': 20,
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
                             'trajectory': {'facecolor': '#000000'}
                        },
                        'static_obstacle': {
                           'shape': {
                               'polygon': basic_shape_parameters_static,
                               'rectangle':basic_shape_parameters_static,
                               'circle': basic_shape_parameters_static,
                           }
                        },
                        'lanelet_network': {
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
    draw_params['scenario']['lanelet'] = draw_params['scenario']['lanelet_network']['lanelet']

    return draw_params


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
        time_begin = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,('time_begin',))
        time_end = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack, tuple(['time_end']))
        if 'dynamic_obstacle' in call_stack:
            trajectory_steps = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack,tuple(['trajectory_steps']))
        else:
            trajectory_steps = commonroad.visualization.draw_dispatch_cr._retrieve_value(
                draw_params, call_stack, tuple(['time_end'])) - time_begin
    except KeyError:
        print("Cannot find stylesheet for trajectory. Called through:")
        print(call_stack)

    if trajectory_steps == 0 or time_begin==time_end:
        return []

    patchlist = list()
    coordinates =list()

    time_max = min(time_end, time_begin + trajectory_steps)

    for traj in obj:
        for time_step in range(time_begin, time_max):
            tmp = traj.state_at_time_step(time_step)
            if tmp is not None:
                coordinates.append(tmp.position)
            else:
                if time_begin > traj.initial_time_step:
                    break

    if len(coordinates) > 0:
        coordinates = np.array(coordinates)
        collection = collections.EllipseCollection(np.ones([coordinates.shape[0],1]) * 0.13,
                                                   np.ones([coordinates.shape[0],1]) * 0.13,
                                                   np.zeros([coordinates.shape[0],1]),
                                                   offsets=coordinates,
                                                   units='xy',
                                                   zorder=24, transOffset=ax.transData, facecolor=facecolor)
        ax.add_collection(collection)
    else:
        collection = None

    return [collection]


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
    lanelets = obj.lanelets

    commonroad.visualization.draw_dispatch_cr.draw_object(
        lanelets, None, ax, draw_params, draw_func, handles, call_stack)


def draw_lanelets(obj: Union[List[Lanelet],Lanelet], plot_limits: Union[List[Union[int,float]], None],
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
    if type(obj) is Lanelet:
        obj = [obj]

    try:
        left_bound_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'left_bound_color'))
        right_bound_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'right_bound_color'))
        center_bound_color = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('lanelet', 'center_bound_color'))
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

        transform = np.array([[np.cos(angle), -np.sin(angle), x],
                              [np.sin(angle), np.cos(angle), y],
                              [0, 0, 1]])
        ptr_trans = transform.dot(scale_m.dot(pts.transpose()))
        ptr_trans = ptr_trans[0:2, :]
        ptr_trans = ptr_trans.transpose()

        path = Path(ptr_trans, codes_direction)
        return path

    vertices_fill = list()
    right_bound_list = np.empty((0,2))
    coordinates_left_border_vertices = np.empty((0,2))
    coordinates_right_border_vertices = np.empty((0,2))
    coordinates_center_border_vertices = np.empty((0,2))
    direction_list = list()
    center_paths = list()
    left_paths=list()
    right_paths = list()


    for lanelet in obj:
        if draw_start_and_direction:
            center = lanelet.center_vertices[0] # 0.5 * (lanelet.left_vertices[0] + lanelet.right_vertices[0])
            tan_vec = np.array(lanelet.right_vertices[0]) - np.array(lanelet.left_vertices[0])
            direction_list.append(direction(center[0], center[1],
                    np.arctan2(tan_vec[1], tan_vec[0]) + 0.5 * np.pi))

        if draw_border_vertices or draw_left_bound:
            if draw_border_vertices:
                coordinates_left_border_vertices = np.vstack((coordinates_left_border_vertices,lanelet.left_vertices))
            left_paths.append(Path(lanelet.left_vertices, closed=False))

        if draw_border_vertices or draw_right_bound:
            if draw_border_vertices:
                coordinates_right_border_vertices = np.vstack((coordinates_right_border_vertices, lanelet.right_vertices))
            right_paths.append(Path(lanelet.right_vertices, closed=False))

        if draw_center_bound:
            center_paths.append(Path(lanelet.center_vertices, closed=False))

        if fill_lanelet:
            vertices_fill.append(np.concatenate((lanelet.right_vertices, np.flip(lanelet.left_vertices, 0))))

        if show_label:
            text_pos = lanelet.interpolate_position(0.5 * lanelet.distance[-1])[0]
            ax.text(text_pos[0], text_pos[1],
                    str(lanelet.lanelet_id),
                    bbox={'facecolor': center_bound_color, 'pad': 2},
                    horizontalalignment='center', verticalalignment='center',
                    zorder=10.2)

    if draw_right_bound:
        ax.add_collection(collections.PathCollection(right_paths, edgecolor=right_bound_color, facecolor='none',
                                                     lw=draw_linewidth, zorder=10, antialiased=antialiased))
    if draw_left_bound:
        ax.add_collection(collections.PathCollection(left_paths, edgecolor=left_bound_color, facecolor='none',
                                                     lw=draw_linewidth, zorder=10, antialiased=antialiased))
    if draw_center_bound:
        ax.add_collection(collections.PathCollection(center_paths, edgecolor=center_bound_color, facecolor='none',
                                       lw=draw_linewidth, zorder=10,  antialiased=antialiased))
    if draw_start_and_direction:
        ax.add_collection(collections.PathCollection(direction_list, color=center_bound_color,
                                  lw=0.5, zorder=10.1, antialiased=antialiased))

    collection_tmp = collections.PolyCollection(vertices_fill, transOffset=ax.transData, facecolor=facecolor, edgecolor='none', antialiaseds=antialiased)
    ax.add_collection(collection_tmp)

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

        if type(o) is not DynamicObstacle:
            warnings.warn('<visualization/scenario> Only lists with objects of the same type can be plotted',
                          UserWarning, stacklevel=1)
            return (occupancy_list, trajectory, shape)

        # draw occupancies
        if (draw_occupancies == 1 or
                (draw_occupancies == 0 and isinstance(o.prediction, commonroad.prediction.prediction.SetBasedPrediction))):
            if draw_shape:
                # occupancy already plotted
                time_begin_occ = time_begin + 1
            else:
                time_begin_occ = time_begin

            for time_step in range(time_begin_occ, time_end):
                tmp = o.occupancy_at_time(time_step)
                if tmp is not None:
                    occupancy_list.append(tmp)
                    # patch_list = draw_occupancy(tmp, None, ax, draw_params, draw_func, handles, call_stack)
                    # handles.setdefault(obj.obstacle_id,[]).append(patch_list)

        # get trajectory
        if isinstance(o.prediction, commonroad.prediction.prediction.TrajectoryPrediction):
            # trajectories_list.append(o.prediction.trajectory)
            trajectory = o.prediction.trajectory

        # get shape
        if draw_shape:
            occ = o.occupancy_at_time(time_begin)
            if occ is None:
                shape = None
            else:
                shape = occ.shape

        # draw car icon
        if draw_icon and isinstance(o.prediction, commonroad.prediction.prediction.TrajectoryPrediction):
            if time_begin == 0:
                inital_state = o.initial_state
            else:
                inital_state = o.prediction.trajectory.state_at_time_step(time_begin)
            if inital_state is not None:
                draw_car(inital_state.position[0], inital_state.position[1], inital_state.orientation, 2.5,
                         ax, zorder=30)

        if trajectory and show_label:
            if time_begin == 0:
                initial_position = o.initial_state.position
                ax.text(initial_position[0] + 0.5, initial_position[1], str(o.obstacle_id),
                        clip_on=True, zorder=1000)
            else:
                begin_state = o.prediction.trajectory.state_at_time_step(time_begin)
                if begin_state is not None:
                    initial_position = o.prediction.trajectory.state_at_time_step(time_begin).position
                    ax.text(initial_position[0]+0.5, initial_position[1], str(o.obstacle_id),
                            clip_on=True, zorder=1000)

        return (occupancy_list, trajectory, shape)

    if type(obj) is DynamicObstacle:
        obj = [obj]

    try:
        time_begin = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('time_begin',))
        time_end = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('time_end',))
        trajectory_steps = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'trajectory_steps'))
        draw_icon = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'draw_icon'))
        show_label = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'show_label'))
        draw_shape = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'draw_shape'))
        draw_occupancies = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('dynamic_obstacle', 'occupancy', 'draw_occupancies'))
    except KeyError:
        warnings.warn("Cannot find stylesheet for dynamic_obstacle. Called through:")
        print(call_stack)

    # time_max = min(time_end, time_begin + trajectory_steps)

    call_stack = tuple(list(call_stack) + ['dynamic_obstacle'])

    # collect objects from all vehicles to draw them in a batch
    tmp_array  = np.array(list(map(collecting,obj)))
    occupancy_list = list(filter(None,list(tmp_array[:,0])))
    trajectories_list = list(filter(None,list(tmp_array[:, 1])))
    shapes_list = list(filter(None,list(tmp_array[:, 2])))

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

    shape_list = list()
    if not 'dynamic_obstacle' in call_stack:
        time_begin = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('time_begin',))
        time_end = commonroad.visualization.draw_dispatch_cr._retrieve_value(
            draw_params, call_stack,
            ('time_end',))

        for occupancy in obj:
            if time_begin <= occupancy.time_step and time_end >= occupancy.time_step:
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
    Orders a list of shapes by their type and draws them in a batch.
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
    # try:
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
    # except KeyError:
    #     print("Cannot find stylesheet for rectangle. Called through:")
    #     print(call_stack)

    vertices = list()

    for rect in obj:
        if type(rect) is not Rectangle:
            warnings.warn('<visualization/scenario> Only lists with objects of the same type can be plotted',
                          UserWarning, stacklevel=1)
            continue
        vertices.append(np.array(rect.vertices))

    collection = collections.PolyCollection(vertices, closed=True, zorder=zorder, transOffset=ax.transData,
                                            facecolor=facecolor, edgecolor=edgecolor, alpha=opacity, antialiaseds=antialiased, linewidth=linewidth)
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


# default function dict, which assigns drawing functions to object classes
draw_func_dict = {commonroad.scenario.scenario.Scenario: draw_scenario,
                  commonroad.scenario.lanelet.Lanelet: draw_lanelets,
                  commonroad.scenario.lanelet.LaneletNetwork: draw_lanelet_network,
                  commonroad.scenario.obstacle.DynamicObstacle: draw_dynamic_obstacles,
                  commonroad.scenario.obstacle.StaticObstacle: draw_static_obstacles,
                  commonroad.scenario.trajectory.Trajectory: draw_trajectories,
                  commonroad.geometry.shape.ShapeGroup: draw_shape_group,
                  commonroad.geometry.shape.Polygon: draw_polygons,
                  commonroad.geometry.shape.Circle: draw_circle,
                  commonroad.geometry.shape.Rectangle: draw_rectangle,
                  commonroad.prediction.prediction.Occupancy: draw_occupancies}
