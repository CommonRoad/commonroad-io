import math
import os
from enum import Enum
from typing import Dict, Tuple, Union, Set, Optional

import commonroad.geometry.shape
import commonroad.prediction.prediction
import commonroad.scenario.obstacle
import matplotlib as mpl
import matplotlib.collections as collections
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib.colors
import matplotlib.artist as artists
from commonroad.visualization.drawable import IDrawable
from matplotlib.colors import hsv_to_rgb, rgb_to_hsv, to_rgb, to_hex
from commonroad.common.util import Interval
from commonroad.geometry.shape import *
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblemSet, \
    PlanningProblem
from commonroad.prediction.prediction import Occupancy
from commonroad.scenario.lanelet import LaneletNetwork, LineMarking
from commonroad.scenario.obstacle import DynamicObstacle, \
    StaticObstacle, \
    ObstacleRole, \
    SignalState
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_sign import TrafficLightState, \
    TrafficLight, \
    TrafficSign
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.visualization.param_server import ParamServer
from commonroad.visualization.traffic_sign import draw_traffic_light_signs
from commonroad.visualization.util import LineDataUnits, \
    collect_center_line_colors, \
    get_arrow_path_at, \
    colormap_idx, \
    get_car_patch, \
    line_marking_to_linestyle, \
    traffic_light_color_dict
from matplotlib.path import Path

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = [""]
__version__ = "2020.3"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"

traffic_sign_path = os.path.join(os.path.dirname(__file__), 'traffic_signs/')


class ZOrders:
    # Map
    LANELET_POLY = 9.0
    INCOMING_POLY = 9.1
    CROSSING_POLY = 9.2
    CENTER_BOUND = 10.0
    LIGHT_STATE_OTHER = 10.0
    LIGHT_STATE_GREEN = 10.05
    DIRECTION_ARROW = 10.1
    SUCCESSORS = 11.0
    STOP_LINE = 11.0
    RIGHT_BOUND = 12.0
    LEFT_BOUND = 12.0
    # Obstacles
    OBSTACLES = 20.0
    CAR_PATCH = 20.0
    # Labels
    LANELET_LABEL = 30.2
    STATE = 100.0
    LABELS = 1000.0
    # Values added to base value from drawing parameters
    INDICATOR_ADD = 0.2
    BRAKING_ADD = 0.2
    HORN_ADD = 0.1
    BLUELIGHT_ADD = 0.1


class MPRenderer:

    def __init__(self, draw_params: Union[ParamServer, dict, None] = None,
                 plot_limits: Union[List[Union[int, float]], None] = None,
                 ax: Union[mpl.axes.Axes, None] = None):
        if draw_params is None:
            self.draw_params = ParamServer()
        elif isinstance(draw_params, dict):
            self.draw_params = ParamServer(data=draw_params)
        else:
            self.draw_params = draw_params
        self.plot_limits = plot_limits
        if ax is None:
            self.ax = plt.gca()
        else:
            self.ax = ax
        self.f = self.ax.figure
        self.dynamic_artists = []

        # Draw elements
        self.dynamic_collections = []
        self.static_collections = []
        self.obstacle_patches = []
        self.traffic_signs = []
        self.traffic_sign_call_stack = tuple()
        self.traffic_sign_draw_params = self.draw_params

    def draw_list(self, drawable_list: List[IDrawable], draw_params=None,
                  call_stack=tuple()) -> None:
        draw_params = self._get_draw_params(draw_params)
        for elem in drawable_list:
            elem.draw(self, draw_params, call_stack)

    def _get_draw_params(self, draw_params):
        if draw_params is None:
            draw_params = self.draw_params
        elif isinstance(draw_params, dict):
            draw_params = ParamServer(data=draw_params)
        return draw_params

    def clear(self) -> None:
        self.static_collections.clear()
        self.obstacle_patches.clear()
        self.traffic_signs.clear()
        self.traffic_sign_call_stack = tuple()
        self.traffic_sign_draw_params = self.draw_params
        self.dynamic_collections.clear()

    def remove_dynamic(self) -> None:
        for art in self.dynamic_artists:
            art.remove()

    def render_dynamic(self) -> List[artists.Artist]:
        artist_list = []
        traffic_sign_artists = draw_traffic_light_signs(self.traffic_signs,
                                                        self.traffic_sign_draw_params,
                                                        self.traffic_sign_call_stack)
        for art in traffic_sign_artists:
            self.ax.add_artist(art)
            artist_list.append(art)
        for col in self.dynamic_collections:
            self.ax.add_collection(col)
            artist_list.append(col)
        self.obstacle_patches.sort(key=lambda x: x.zorder)
        patch_col = mpl.collections.PatchCollection(self.obstacle_patches,
                                                    match_original=True,
                                                    zorder=ZOrders.OBSTACLES)
        self.ax.add_collection(patch_col)
        artist_list.append(patch_col)
        self.dynamic_artists = artist_list
        return artist_list

    def render_static(self) -> List[collections.Collection]:
        for col in self.static_collections:
            self.ax.add_collection(col)
        return self.static_collections

    def render(self, show: bool = False, filename: str = None) -> List[
        artists.Artist]:
        self.ax.cla()
        artists_list = self.render_static()
        artists_list.extend(self.render_dynamic())
        if self.plot_limits is None:
            self.ax.autoscale(True)
        self.ax.set_aspect('equal')
        if filename is not None:
            self.f.savefig(filename)
        if show:
            self.f.show()
        self.clear()
        return artists_list

    def add_legend(self, legend: Dict[Tuple[str, ...], str],
                   draw_params: Union[ParamServer, dict, None] = None) -> None:
        """
        Adds legend with color of objects specified by legend.keys() and
        texts specified by legend.values().
        :param legend: color of objects specified by path in legend.keys()
        and texts specified by legend.values()
        :param draw_params: draw parameters used for plotting (color is
        extracted using path in legend.keys())
        :return:
        """
        draw_params = self._get_draw_params(draw_params)
        handles = []
        for obj_name, text in legend.items():
            try:
                color = draw_params[obj_name]
            except:
                color = None
            if color is not None:
                handles.append(mpl.patches.Patch(color=color, label=text))

        legend = self.ax.legend(handles=handles)
        legend.set_zorder(ZOrders.LABELS)

    def draw_scenario(self, obj: Scenario,
                      draw_params: Union[ParamServer, dict, None],
                      call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted
        :param draw_params: parameters for plotting given by a nested dict
        that recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)
        call_stack = tuple(list(call_stack) + ['scenario'])
        obj.lanelet_network.draw(self, draw_params, call_stack)

        # draw only obstacles inside plot limits
        if self.plot_limits is not None:
            time_begin = draw_params.by_callstack(call_stack, ('time_begin',))
            # dynamic obstacles
            dyn_obs = obj.obstacles_by_position_intervals(
                    [Interval(self.plot_limits[0], self.plot_limits[1]),
                     Interval(self.plot_limits[2], self.plot_limits[3])],
                    tuple([ObstacleRole.DYNAMIC]), time_begin)

            # static obstacles
            static_obs = obj.obstacles_by_position_intervals(
                    [Interval(self.plot_limits[0], self.plot_limits[1]),
                     Interval(self.plot_limits[2], self.plot_limits[3])],
                    tuple([ObstacleRole.STATIC]))
        else:
            dyn_obs = obj.dynamic_obstacles
            static_obs = obj.static_obstacles
        # Draw all objects
        for o in dyn_obs:
            o.draw(self, draw_params, call_stack)
        for o in static_obs:
            o.draw(self, draw_params, call_stack)

    def draw_static_obstacle(self, obj: StaticObstacle,
                             draw_params: Union[ParamServer, dict, None],
                             call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted
        :param draw_params: parameters for plotting given by a nested dict
        that recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)
        time_begin = draw_params.by_callstack(tuple(), ('time_begin',))
        call_stack = tuple(list(call_stack) + ['static_obstacle'])
        obj.occupancy_at_time(time_begin).draw(self, draw_params, call_stack)

    def draw_dynamic_obstacle(self, obj: DynamicObstacle,
                              draw_params: Union[ParamServer, dict, None],
                              call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted
        :param draw_params: parameters for plotting given by a nested dict
        that recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)

        time_begin = draw_params.by_callstack(call_stack, ('time_begin',))
        time_end = draw_params.by_callstack(call_stack, ('time_end',))

        call_stack = tuple(list(call_stack) + ['dynamic_obstacle'])
        draw_icon = draw_params.by_callstack(call_stack, 'draw_icon')
        show_label = draw_params.by_callstack(call_stack, 'show_label')
        draw_shape = draw_params.by_callstack(call_stack, 'draw_shape')
        draw_initial_state = draw_params.by_callstack(call_stack, (
                'initial_state', 'draw_initial_state'))
        scale_factor = draw_params.by_callstack(call_stack, (
                'initial_state', 'scale_factor'))
        kwargs_init_state = draw_params.by_callstack(call_stack, (
                'initial_state', 'kwargs'))
        draw_occupancies = draw_params.by_callstack(call_stack, (
                'occupancy', 'draw_occupancies'))
        draw_signals = draw_params.by_callstack(call_stack, 'draw_signals')
        draw_trajectory = draw_params.by_callstack(call_stack, (
                'trajectory', 'draw_trajectory'))

        draw_history = draw_params.by_callstack(call_stack,
                                                ('history', 'draw_history'))
        if draw_history and isinstance(obj.prediction,
                                       commonroad.prediction.prediction.TrajectoryPrediction):
            self._draw_history(obj, call_stack, draw_params)

        # draw occupancies
        if (draw_occupancies == 1 or (draw_occupancies == 0 and type(
                obj.prediction) == commonroad.prediction.prediction.SetBasedPrediction)):
            if draw_shape:
                # occupancy already plotted
                time_begin_occ = time_begin + 1
            else:
                time_begin_occ = time_begin

            for time_step in range(time_begin_occ, time_end):
                occ = obj.occupancy_at_time(time_step)
                if occ is not None:
                    occ.draw(self, draw_params, call_stack)

        # draw trajectory
        if draw_trajectory and type(
                obj.prediction) == \
                commonroad.prediction.prediction.TrajectoryPrediction:
            obj.prediction.trajectory.draw(self, draw_params, call_stack)

        # draw shape
        if draw_shape:
            occ = obj.occupancy_at_time(time_begin)
            if occ is not None:
                occ.draw(self, draw_params, call_stack)

                if draw_signals:
                    sig = obj.signal_state_at_time_step(time_begin)
                    if sig is not None:
                        self.draw_signale_state(sig, occ, draw_params,
                                                call_stack)

        # draw car icon
        if draw_icon and type(
                obj.prediction) == \
                commonroad.prediction.prediction.TrajectoryPrediction:
            if time_begin == 0:
                inital_state = obj.initial_state
            else:
                inital_state = obj.prediction.trajectory.state_at_time_step(
                        time_begin)
            if inital_state is not None:
                self.obstacle_patches.extend(
                        get_car_patch(inital_state.position[0],
                                      inital_state.position[1],
                                      inital_state.orientation, 2.5,
                                      zorder=ZOrders.CAR_PATCH))

        # get state
        state = None
        if time_begin == 0:
            state = obj.initial_state
        elif type(
                obj.prediction) == \
                commonroad.prediction.prediction.TrajectoryPrediction:
            state = obj.prediction.trajectory.state_at_time_step(time_begin)

        # draw label
        if show_label:
            if state is not None:
                position = state.position
                self.dynamic_artists.append(
                        self.ax.text(position[0] + 0.5, position[1],
                                     str(obj.obstacle_id), clip_on=True,
                                     zorder=ZOrders.LABELS))

        # draw initial state
        if draw_initial_state:
            state.draw(self, call_stack=call_stack, draw_params=draw_params,
                       scale_factor=scale_factor, arrow_args=kwargs_init_state)

    def _draw_history(self, dyn_obs: DynamicObstacle,
                      call_stack: Tuple[str, ...], draw_params: ParamServer):
        """
        Draws history occupancies of the dynamic obstacle
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of drawing functions
        :param draw_params: parameters for plotting given by a nested dict
        that recreates the structure of an object,
        :param dyn_obs: the dynamic obstacle
        :return:
        """
        time_begin = draw_params['time_begin']
        history_callstack = call_stack + ('history',)
        history_steps = draw_params.by_callstack(call_stack, 'steps')
        history_fade_factor = draw_params.by_callstack(call_stack,
                                                       'fade_factor')
        history_base_color = draw_params.by_callstack(history_callstack, (
                'occupancy', 'shape', 'rectangle', 'facecolor'))
        history_base_color = rgb_to_hsv(to_rgb(history_base_color))
        for time_step in range(1, history_steps):
            occ = dyn_obs.occupancy_at_time(time_begin - time_step)
            if occ is not None:
                color_hsv_new = history_base_color.copy()
                color_hsv_new[2] = max(0, color_hsv_new[
                    2] - history_fade_factor * time_step)
                color_hex_new = to_hex(hsv_to_rgb(color_hsv_new))
                draw_params[call_stack + ('occupancy', 'shape', 'rectangle',
                                          'facecolor')] = color_hex_new
                draw_params[call_stack + ('occupancy', 'shape', 'circle',
                                          'facecolor')] = color_hex_new
                draw_params[call_stack + ('occupancy', 'shape', 'polygon',
                                          'facecolor')] = color_hex_new
                occ.draw(self, draw_params, history_callstack)

    def draw_trajectory(self, obj: Trajectory,
                        draw_params: Union[ParamServer, dict, None],
                        call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted
        :param draw_params: parameters for plotting given by a nested dict
        that recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)

        time_begin = draw_params.by_callstack(call_stack, 'time_begin')
        time_end = draw_params.by_callstack(call_stack, 'time_end')

        call_stack = call_stack + ('trajectory',)
        line_color = draw_params.by_callstack(call_stack, 'facecolor')

        line_width = draw_params.by_callstack(call_stack, 'line_width')
        draw_continuous = draw_params.by_callstack(call_stack,
                                                   'draw_continuous')
        z_order = draw_params.by_callstack(call_stack, 'z_order')

        if time_begin == time_end:
            return

        traj_points = list()
        for time_step in range(time_begin, time_end):
            tmp = obj.state_at_time_step(time_step)
            if tmp is not None:
                traj_points.append(tmp.position)
            else:
                if time_begin > obj.initial_time_step:
                    break
        traj_points = np.array(traj_points)
        if draw_continuous:
            path = mpl.path.Path(traj_points, closed=False)
            self.obstacle_patches.append(
                    mpl.patches.PathPatch(path, color=line_color, lw=line_width,
                                          zorder=z_order, fill=False))
        else:
            self.dynamic_collections.append(collections.EllipseCollection(
                    np.ones([traj_points.shape[0], 1]) * line_width,
                    np.ones([traj_points.shape[0], 1]) * line_width,
                    np.zeros([traj_points.shape[0], 1]), offsets=traj_points,
                    units='xy', linewidths=0, zorder=z_order,
                    transOffset=self.ax.transData, facecolor=line_color))

    def draw_trajectories(self, obj: List[Trajectory],
                          draw_params: Union[ParamServer, dict, None],
                          call_stack: Tuple[str, ...]) -> None:
        draw_params = self._get_draw_params(draw_params)
        unique_colors = draw_params.by_callstack(call_stack, (
                'trajectory', 'unique_colors'))
        if unique_colors:
            cmap = colormap_idx(len(obj))
            for i, traj in enumerate(obj):
                draw_params['trajectory', 'facecolor'] = mpl.colors.to_hex(
                        cmap(i))
                traj.draw(self, draw_params, call_stack)
        else:
            self.draw_list(obj, draw_params)

    def draw_polygon(self, obj: Polygon,
                     draw_params: Union[ParamServer, dict, None],
                     call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted

        :param draw_params: parameters for plotting given by a nested dict
        that recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)
        call_stack = tuple(list(call_stack) + ['shape', 'polygon'])
        facecolor = draw_params.by_callstack(call_stack, 'facecolor')
        edgecolor = draw_params.by_callstack(call_stack, 'edgecolor')
        zorder = draw_params.by_callstack(call_stack, 'zorder')
        opacity = draw_params.by_callstack(call_stack, 'opacity')
        linewidth = draw_params.by_callstack(call_stack, 'linewidth')
        antialiased = draw_params.by_callstack(call_stack, 'antialiased')
        self.obstacle_patches.append(
                mpl.patches.Polygon(obj.vertices, closed=True,
                                    facecolor=facecolor, edgecolor=edgecolor,
                                    zorder=zorder, alpha=opacity,
                                    linewidth=linewidth,
                                    antialiased=antialiased))

    def draw_rectangle(self, obj: Rectangle,
                       draw_params: Union[ParamServer, dict, None],
                       call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted
        :param draw_params: parameters for plotting given by a nested dict that
        recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)
        call_stack = tuple(list(call_stack) + ['shape', 'rectangle'])

        facecolor = draw_params.by_callstack(call_stack, 'facecolor')
        edgecolor = draw_params.by_callstack(call_stack, 'edgecolor')
        zorder = draw_params.by_callstack(call_stack, 'zorder')
        opacity = draw_params.by_callstack(call_stack, 'opacity')
        linewidth = draw_params.by_callstack(call_stack, 'linewidth')
        antialiased = draw_params.by_callstack(call_stack, 'antialiased')

        self.obstacle_patches.append(
                mpl.patches.Polygon(obj.vertices, closed=True, zorder=zorder,
                                    facecolor=facecolor, edgecolor=edgecolor,
                                    alpha=opacity, antialiased=antialiased,
                                    linewidth=linewidth))

    def draw_circle(self, obj: Circle,
                    draw_params: Union[ParamServer, dict, None],
                    call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted

        :param draw_params: parameters for plotting given by a nested dict that
        recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)
        call_stack = tuple(list(call_stack) + ['shape', 'circle'])
        facecolor = draw_params.by_callstack(call_stack, 'facecolor')
        edgecolor = draw_params.by_callstack(call_stack, 'edgecolor')
        zorder = draw_params.by_callstack(call_stack, 'zorder')
        opacity = draw_params.by_callstack(call_stack, 'opacity')
        linewidth = draw_params.by_callstack(call_stack, 'linewidth')

        self.obstacle_patches.append(
                mpl.patches.Circle(obj.center, obj.radius, facecolor=facecolor,
                                   edgecolor=edgecolor, zorder=zorder,
                                   linewidth=linewidth, alpha=opacity))

    def draw_state(self, state: State,
                   draw_params: Union[ParamServer, dict, None],
                   call_stack: Tuple[str, ...] = None,
                   scale_factor: Optional[float] = None,
                   arrow_args: Optional[dict] = None) -> None:
        """
        :param state: state to be plotted
        :param draw_params: parameters for plotting given by a nested dict
        that recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :param scale_factor scaling factor for arrow
        :param arrow_args arguments for arrow drawing
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)

        scale_factor = scale_factor or draw_params.by_callstack(call_stack, (
                'initial_state', 'scale_factor'))
        arrow_args = arrow_args or draw_params.by_callstack(call_stack, (
                'initial_state', 'kwargs'))

        cos = math.cos(state.orientation)
        sin = math.sin(state.orientation)
        x = state.position[0]
        y = state.position[1]
        self.obstacle_patches.append(
            mpl.patches.Arrow(x=x, y=y, dx=state.velocity * cos * scale_factor,
                              dy=state.velocity * sin * scale_factor,
                              zorder=ZOrders.STATE, **arrow_args))

    def draw_lanelet_network(self, obj: LaneletNetwork,
                             draw_params: Union[ParamServer, dict, None],
                             call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted
        :param draw_params: parameters for plotting given by a nested dict that
        recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)
        call_stack = tuple(list(call_stack) + ['lanelet_network'])

        traffic_lights = obj.traffic_lights
        traffic_signs = obj.traffic_signs
        intersections = obj.intersections
        lanelets = obj.lanelets

        time_begin = draw_params.by_callstack(call_stack, ('time_begin',))
        if traffic_lights is not None:
            draw_traffic_lights = draw_params.by_callstack(call_stack, (
                    'traffic_light', 'draw_traffic_lights'))

            traffic_light_colors = draw_params.by_callstack(call_stack,
                                                            ('traffic_light'))
        else:
            draw_traffic_lights = False

        if traffic_signs is not None:
            draw_traffic_signs = draw_params.by_callstack(call_stack, (
                    'traffic_sign', 'draw_traffic_signs'))
            show_traffic_sign_label = draw_params.by_callstack(call_stack, (
                    'traffic_sign', 'show_label'))
        else:
            draw_traffic_signs = show_traffic_sign_label = False

        if intersections is not None and len(intersections) > 0:
            draw_intersections = draw_params.by_callstack(call_stack, (
                    'intersection', 'draw_intersections'))
        else:
            draw_intersections = False

        if draw_intersections is True:
            draw_incoming_lanelets = draw_params.by_callstack(call_stack, (
                    'intersection', 'draw_incoming_lanelets'))
            incoming_lanelets_color = draw_params.by_callstack(call_stack, (
                    'intersection', 'incoming_lanelets_color'))
            draw_crossings = draw_params.by_callstack(call_stack, (
                    'intersection', 'draw_crossings'))
            crossings_color = draw_params.by_callstack(call_stack, (
                    'intersection', 'crossings_color'))
            draw_successors = draw_params.by_callstack(call_stack, (
                    'intersection', 'draw_successors'))
            successors_left_color = draw_params.by_callstack(call_stack, (
                    'intersection', 'successors_left_color'))
            successors_straight_color = draw_params.by_callstack(call_stack, (
                    'intersection', 'successors_straight_color'))
            successors_right_color = draw_params.by_callstack(call_stack, (
                    'intersection', 'successors_right_color'))
            show_intersection_labels = draw_params.by_callstack(call_stack, (
                    'intersection', 'show_label'))
        else:
            draw_incoming_lanelets = draw_crossings = draw_successors = \
                show_intersection_labels = False

        left_bound_color = draw_params.by_callstack(call_stack, (
                'lanelet', 'left_bound_color'))
        right_bound_color = draw_params.by_callstack(call_stack, (
                'lanelet', 'right_bound_color'))
        center_bound_color = draw_params.by_callstack(call_stack, (
                'lanelet', 'center_bound_color'))
        unique_colors = draw_params.by_callstack(call_stack,
                                                 ('lanelet', 'unique_colors'))
        draw_stop_line = draw_params.by_callstack(call_stack,
                                                  ('lanelet', 'draw_stop_line'))
        stop_line_color = draw_params.by_callstack(call_stack, (
                'lanelet', 'stop_line_color'))
        draw_line_markings = draw_params.by_callstack(call_stack, (
                'lanelet', 'draw_line_markings'))
        show_label = draw_params.by_callstack(call_stack,
                                              ('lanelet', 'show_label'))
        draw_border_vertices = draw_params.by_callstack(call_stack, (
                'lanelet', 'draw_border_vertices'))
        draw_left_bound = draw_params.by_callstack(call_stack, (
                'lanelet', 'draw_left_bound'))
        draw_right_bound = draw_params.by_callstack(call_stack, (
                'lanelet', 'draw_right_bound'))
        draw_center_bound = draw_params.by_callstack(call_stack, (
                'lanelet', 'draw_center_bound'))
        draw_start_and_direction = draw_params.by_callstack(call_stack, (
                'lanelet', 'draw_start_and_direction'))
        draw_linewidth = draw_params.by_callstack(call_stack,
                                                  ('lanelet', 'draw_linewidth'))
        fill_lanelet = draw_params.by_callstack(call_stack,
                                                ('lanelet', 'fill_lanelet'))
        facecolor = draw_params.by_callstack(call_stack,
                                             ('lanelet', 'facecolor'))
        antialiased = draw_params.by_callstack(call_stack, 'antialiased')

        draw_lanlet_ids = draw_params.by_callstack(call_stack, 'draw_ids')

        # Collect lanelets
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
                incomings: List[set] = []
                for intersection in intersections:
                    for incoming in intersection.incomings:
                        incomings.append(incoming.incoming_lanelets)
                        for l_id in incoming.incoming_lanelets:
                            incomings_left[l_id] = incoming.left_of
                            incomings_id[l_id] = incoming.incoming_id
                incoming_lanelets: Set[int] = set.union(*incomings)

            if draw_crossings:
                tmp_list: List[set] = [intersection.crossings for intersection
                                       in intersections]
                crossings: Set[int] = set.union(*tmp_list)

            if draw_successors:
                tmp_list: List[set] = [incoming.successors_left for intersection
                                       in intersections for incoming in
                                       intersection.incomings]
                successors_left: Set[int] = set.union(*tmp_list)
                tmp_list: List[set] = [incoming.successors_straight for
                                       intersection in intersections for
                                       incoming in intersection.incomings]
                successors_straight: Set[int] = set.union(*tmp_list)
                tmp_list: List[set] = [incoming.successors_right for
                                       intersection in intersections for
                                       incoming in intersection.incomings]
                successors_right: Set[int] = set.union(*tmp_list)
                all_successors = set.union(successors_straight,
                                           successors_right, successors_left)

        # select unique colors from colormap for each lanelet's center_line

        incoming_vertices_fill = list()
        crossing_vertices_fill = list()
        succ_left_paths = list()
        succ_straight_paths = list()
        succ_right_paths = list()

        vertices_fill = list()
        coordinates_left_border_vertices = np.empty((0, 2))
        coordinates_right_border_vertices = np.empty((0, 2))
        direction_list = list()
        center_paths = list()
        left_paths = list()
        right_paths = list()

        if draw_traffic_lights:
            center_line_color_dict = collect_center_line_colors(obj,
                                                                traffic_lights,
                                                                time_begin)

        cmap_lanelet = colormap_idx(len(lanelets))

        # collect paths for drawing
        for i_lanelet, lanelet in enumerate(lanelets):
            if isinstance(draw_lanlet_ids,
                          list) and lanelet.lanelet_id not in draw_lanlet_ids:
                continue

            # left bound
            if draw_border_vertices or draw_left_bound:
                if draw_border_vertices:
                    coordinates_left_border_vertices = np.vstack((
                            coordinates_left_border_vertices,
                            lanelet.left_vertices))

                if draw_line_markings and lanelet.line_marking_left_vertices \
                        is not LineMarking.UNKNOWN and \
                        lanelet.line_marking_left_vertices is not \
                        LineMarking.NO_MARKING:
                    linestyle, dashes, linewidth_metres = \
                        line_marking_to_linestyle(
                            lanelet.line_marking_left_vertices)
                    tmp_left = lanelet.left_vertices.copy()
                    tmp_left[0, :] = \
                        lanelet.interpolate_position(linewidth_metres / 2)[2]
                    tmp_left[-1, :] = lanelet.interpolate_position(
                            lanelet.distance[-1] - linewidth_metres / 2)[2]
                    line = LineDataUnits(tmp_left[:, 0], tmp_left[:, 1],
                                         zorder=ZOrders.LEFT_BOUND,
                                         linewidth=linewidth_metres, alpha=1.0,
                                         color=left_bound_color,
                                         linestyle=linestyle, dashes=dashes)
                    # TODO: Convert to path
                    self.ax.add_line(line)
                else:
                    left_paths.append(Path(lanelet.left_vertices, closed=False))

            # right bound
            if draw_border_vertices or draw_right_bound:
                if draw_border_vertices:
                    coordinates_right_border_vertices = np.vstack((
                            coordinates_right_border_vertices,
                            lanelet.right_vertices))

                if draw_line_markings and lanelet.line_marking_right_vertices\
                        is not LineMarking.UNKNOWN and \
                        lanelet.line_marking_right_vertices is not \
                        LineMarking.NO_MARKING:
                    linestyle, dashes, linewidth_metres = \
                        line_marking_to_linestyle(
                            lanelet.line_marking_right_vertices)
                    tmp_right = lanelet.right_vertices.copy()
                    tmp_right[0, :] = \
                        lanelet.interpolate_position(linewidth_metres / 2)[1]
                    tmp_right[-1, :] = lanelet.interpolate_position(
                            lanelet.distance[-1] - linewidth_metres / 2)[1]
                    line = LineDataUnits(tmp_right[:, 0], tmp_right[:, 1],
                                         zorder=ZOrders.RIGHT_BOUND,
                                         linewidth=linewidth_metres, alpha=1.0,
                                         color=right_bound_color,
                                         linestyle=linestyle, dashes=dashes)
                    # TODO: Convert to path
                    self.ax.add_line(line)
                else:
                    right_paths.append(
                            Path(lanelet.right_vertices, closed=False))

            # stop line
            if draw_stop_line and lanelet.stop_line:
                stop_line = np.vstack(
                        [lanelet.stop_line.start, lanelet.stop_line.end])
                linestyle, dashes, linewidth_metres = line_marking_to_linestyle(
                        lanelet.stop_line.line_marking)
                # cut off in the beginning, because linewidth_metres is added
                # later
                vec = stop_line[1, :] - stop_line[0, :]
                tangent = vec / np.linalg.norm(vec)
                stop_line[0, :] += linewidth_metres * tangent / 2
                stop_line[1, :] -= linewidth_metres * tangent / 2
                line = LineDataUnits(stop_line[:, 0], stop_line[:, 1],
                                     zorder=ZOrders.STOP_LINE,
                                     linewidth=linewidth_metres, alpha=1.0,
                                     color=stop_line_color, linestyle=linestyle,
                                     dashes=dashes)
                # TODO: Convert to path
                self.ax.add_line(line)

            if unique_colors:
                # set center bound color to unique value
                center_bound_color = cmap_lanelet(i_lanelet)

            # direction arrow
            if draw_start_and_direction:
                center = lanelet.center_vertices[0]
                tan_vec = np.array(lanelet.right_vertices[0]) - np.array(
                        lanelet.left_vertices[0])
                path = get_arrow_path_at(center[0], center[1],
                                         np.arctan2(tan_vec[1],
                                                    tan_vec[0]) + 0.5 * np.pi)
                if unique_colors:
                    direction_list.append(mpl.patches.PathPatch(path,
                                                                color=center_bound_color,
                                                                lw=0.5,
                                                                zorder=ZOrders.DIRECTION_ARROW,
                                                                antialiased=antialiased))
                else:
                    direction_list.append(path)

            # visualize traffic light state through colored center bound
            has_traffic_light = draw_traffic_lights and lanelet.lanelet_id in\
                                center_line_color_dict
            if has_traffic_light:
                light_state = center_line_color_dict[lanelet.lanelet_id]

                if light_state is not TrafficLightState.INACTIVE:
                    linewidth_metres = 0.75
                    # dashed line for red_yellow
                    linestyle = '--' if light_state == \
                                        TrafficLightState.RED_YELLOW else '-'
                    dashes = (5, 5) if linestyle == '--' else (None, None)

                    # cut off in the beginning, because linewidth_metres is
                    # added
                    # later
                    tmp_center = lanelet.center_vertices.copy()
                    if lanelet.distance[-1] > linewidth_metres:
                        tmp_center[0, :] = \
                            lanelet.interpolate_position(linewidth_metres)[0]
                    zorder = ZOrders.LIGHT_STATE_GREEN if light_state == \
                                                          TrafficLightState.GREEN else ZOrders.LIGHT_STATE_OTHER
                    line = LineDataUnits(tmp_center[:, 0], tmp_center[:, 1],
                                         zorder=zorder,
                                         linewidth=linewidth_metres, alpha=0.7,
                                         color=traffic_light_color_dict(
                                                 light_state,
                                                 traffic_light_colors),
                                         linestyle=linestyle, dashes=dashes)
                    # TODO: Convert to path
                    self.ax.add_line(line)

            # draw colored center bound. Hierarchy or colors: successors > usual
            # center bound
            is_successor = draw_intersections and draw_successors and \
                           lanelet.lanelet_id in all_successors
            if is_successor:
                if lanelet.lanelet_id in successors_left:
                    succ_left_paths.append(
                            Path(lanelet.center_vertices, closed=False))
                elif lanelet.lanelet_id in successors_straight:
                    succ_straight_paths.append(
                            Path(lanelet.center_vertices, closed=False))
                else:
                    succ_right_paths.append(
                            Path(lanelet.center_vertices, closed=False))

            elif draw_center_bound:
                if unique_colors:
                    center_paths.append(mpl.patches.PathPatch(
                            Path(lanelet.center_vertices, closed=False),
                            edgecolor=center_bound_color, facecolor='none',
                            lw=draw_linewidth, zorder=ZOrders.CENTER_BOUND,
                            antialiased=antialiased))
                else:
                    center_paths.append(
                            Path(lanelet.center_vertices, closed=False))

            is_incoming_lanelet = draw_intersections and \
                                  draw_incoming_lanelets and (
                    lanelet.lanelet_id in incoming_lanelets)
            is_crossing = draw_intersections and draw_crossings and (
                    lanelet.lanelet_id in crossings)

            # Draw lanelet area
            if fill_lanelet:
                if not is_incoming_lanelet and not is_crossing:
                    vertices_fill.append(np.concatenate((lanelet.right_vertices,
                                                         np.flip(
                                                                 lanelet.left_vertices,
                                                                 0))))

            # collect incoming lanelets in separate list for plotting in
            # different color
            if is_incoming_lanelet:
                incoming_vertices_fill.append(np.concatenate((
                        lanelet.right_vertices,
                        np.flip(lanelet.left_vertices, 0))))
            elif is_crossing:
                crossing_vertices_fill.append(np.concatenate((
                        lanelet.right_vertices,
                        np.flip(lanelet.left_vertices, 0))))

            # Draw labels
            if show_label or show_intersection_labels or draw_traffic_signs:
                strings = []
                if show_label:
                    strings.append(str(lanelet.lanelet_id))
                if is_incoming_lanelet and show_intersection_labels:
                    strings.append(
                            'inc_id: ' + str(incomings_id[lanelet.lanelet_id]))
                    strings.append('inc_left: ' + str(
                            incomings_left[lanelet.lanelet_id]))
                if draw_traffic_signs and show_traffic_sign_label is True:
                    traffic_signs_tmp = [traffic_signs[id] for id in
                                         lanelet.traffic_signs]
                    if traffic_signs_tmp:
                        # add as text to label
                        str_tmp = 'sign: '
                        add_str = ''
                        for sign in traffic_signs_tmp:
                            for el in sign.traffic_sign_elements:
                                # TrafficSignIDGermany(
                                # el.traffic_sign_element_id).name would give
                                # the
                                # name
                                str_tmp += add_str + \
                                           el.traffic_sign_element_id.value
                                add_str = ', '

                        strings.append(str_tmp)

                label_string = ', '.join(strings)
                if len(label_string) > 0:
                    # compute normal angle of label box
                    clr_positions = lanelet.interpolate_position(
                            0.5 * lanelet.distance[-1])
                    normal_vector = np.array(clr_positions[1]) - np.array(
                            clr_positions[2])
                    angle = np.rad2deg(
                            np.arctan2(normal_vector[1], normal_vector[0])) - 90
                    angle = angle if Interval(-90, 90).contains(
                            angle) else angle - 180

                    self.ax.text(clr_positions[0][0], clr_positions[0][1],
                                 label_string, bbox={
                                'facecolor': center_bound_color, 'pad': 2
                        }, horizontalalignment='center',
                                 verticalalignment='center', rotation=angle,
                                 zorder=ZOrders.LANELET_LABEL)

        # draw paths and collect axis handles
        if draw_right_bound:
            self.static_collections.append(
                    collections.PathCollection(right_paths,
                                               edgecolor=right_bound_color,
                                               facecolor='none',
                                               lw=draw_linewidth,
                                               zorder=ZOrders.RIGHT_BOUND,
                                               antialiased=antialiased))
        if draw_left_bound:
            self.static_collections.append(
                    collections.PathCollection(left_paths,
                                               edgecolor=left_bound_color,
                                               facecolor='none',
                                               lw=draw_linewidth,
                                               zorder=ZOrders.LEFT_BOUND,
                                               antialiased=antialiased))
        if unique_colors:
            if draw_center_bound:
                if draw_center_bound:
                    self.static_collections.append(
                            collections.PatchCollection(center_paths,
                                                        match_original=True,
                                                        zorder=ZOrders.CENTER_BOUND,
                                                        antialiased=antialiased))
                if draw_start_and_direction:
                    self.static_collections.append(
                            collections.PatchCollection(direction_list,
                                                        match_original=True,
                                                        zorder=ZOrders.DIRECTION_ARROW,
                                                        antialiased=antialiased))

        else:
            if draw_center_bound:
                self.static_collections.append(
                        collections.PathCollection(center_paths,
                                                   edgecolor=center_bound_color,
                                                   facecolor='none',
                                                   lw=draw_linewidth,
                                                   zorder=ZOrders.CENTER_BOUND,
                                                   antialiased=antialiased))
            if draw_start_and_direction:
                self.static_collections.append(
                        collections.PathCollection(direction_list,
                                                   color=center_bound_color,
                                                   lw=0.5,
                                                   zorder=ZOrders.DIRECTION_ARROW,
                                                   antialiased=antialiased))

        if successors_left:
            self.static_collections.append(
                    collections.PathCollection(succ_left_paths,
                                               edgecolor=successors_left_color,
                                               facecolor='none',
                                               lw=draw_linewidth * 3.0,
                                               zorder=ZOrders.SUCCESSORS,
                                               antialiased=antialiased))
        if successors_straight:
            self.static_collections.append(
                    collections.PathCollection(succ_straight_paths,
                                               edgecolor=successors_straight_color,
                                               facecolor='none',
                                               lw=draw_linewidth * 3.0,
                                               zorder=ZOrders.SUCCESSORS,
                                               antialiased=antialiased))
        if successors_right:
            self.static_collections.append(
                    collections.PathCollection(succ_right_paths,
                                               edgecolor=successors_right_color,
                                               facecolor='none',
                                               lw=draw_linewidth * 3.0,
                                               zorder=ZOrders.SUCCESSORS,
                                               antialiased=antialiased))

        # fill lanelets with facecolor
        self.static_collections.append(collections.PolyCollection(vertices_fill,
                                                                  transOffset=self.ax.transData,
                                                                  zorder=ZOrders.LANELET_POLY,
                                                                  facecolor=facecolor,
                                                                  edgecolor='none',
                                                                  antialiased=antialiased))
        if incoming_vertices_fill:
            self.static_collections.append(
                    collections.PolyCollection(incoming_vertices_fill,
                                               transOffset=self.ax.transData,
                                               facecolor=incoming_lanelets_color,
                                               edgecolor='none',
                                               zorder=ZOrders.INCOMING_POLY,
                                               antialiased=antialiased))
        if crossing_vertices_fill:
            self.static_collections.append(
                    collections.PolyCollection(crossing_vertices_fill,
                                               transOffset=self.ax.transData,
                                               facecolor=crossings_color,
                                               edgecolor='none',
                                               zorder=ZOrders.CROSSING_POLY,
                                               antialiased=antialiased))

        # draw_border_vertices
        if draw_border_vertices:
            # left vertices
            self.static_collections.append(collections.EllipseCollection(
                    np.ones([coordinates_left_border_vertices.shape[0], 1]) * 1,
                    np.ones([coordinates_left_border_vertices.shape[0], 1]) * 1,
                    np.zeros([coordinates_left_border_vertices.shape[0], 1]),
                    offsets=coordinates_left_border_vertices,
                    color=left_bound_color, transOffset=self.ax.transData))

            # right_vertices
            self.static_collections.append(collections.EllipseCollection(
                    np.ones([coordinates_right_border_vertices.shape[0],
                             1]) * 1, np.ones(
                            [coordinates_right_border_vertices.shape[0],
                             1]) * 1,
                    np.zeros([coordinates_right_border_vertices.shape[0], 1]),
                    offsets=coordinates_right_border_vertices,
                    color=right_bound_color, transOffset=self.ax.transData))

        if draw_traffic_signs:
            # draw actual traffic sign
            for sign in traffic_signs:
                sign.draw(self, draw_params, call_stack)

        if draw_traffic_lights:
            # draw actual traffic light
            for light in traffic_lights:
                light.draw(self, draw_params, call_stack)

    def draw_planning_problem_set(self, obj: PlanningProblemSet,
                                  draw_params: Union[ParamServer, dict, None],
                                  call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted
        :param draw_params: parameters for plotting given by a nested dict
        that recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)
        call_stack = tuple(list(call_stack) + ['planning_problem_set'])
        draw_ids = draw_params.by_callstack(call_stack, 'draw_ids')

        for pp_id, problem in obj.planning_problem_dict.items():
            if draw_ids == 'all' or pp_id in draw_ids:
                self.draw_planning_problem(problem, draw_params, call_stack)

    def draw_planning_problem(self, obj: PlanningProblem,
                              draw_params: Union[ParamServer, dict, None],
                              call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted
        :param draw_params: parameters for plotting given by a nested dict
        that recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)
        call_stack = tuple(list(call_stack) + ['planning_problem'])
        if 'initial_state' not in draw_params:
            draw_params['initial_state'] = {}
        draw_params['initial_state', 'label'] = 'initial position'
        self.draw_initital_state(obj.initial_state, draw_params, call_stack)
        self.draw_goal_region(obj.goal, draw_params, call_stack)

    def draw_initital_state(self, obj: State,
                            draw_params: Union[ParamServer, dict, None],
                            call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted
        :param draw_params: parameters for plotting given by a nested dict
        that recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)
        facecolor = draw_params.by_callstack(call_stack,
                                             ('initial_state', 'facecolor'))
        zorder = draw_params.by_callstack(call_stack,
                                          ('initial_state', 'zorder'))
        label = draw_params.by_callstack(call_stack, ('initial_state', 'label'))

        self.ax.plot(obj.position[0], obj.position[1], 'o', color=facecolor,
                     zorder=zorder, markersize=3)
        self.ax.annotate(label, xy=(obj.position[0] + 1, obj.position[1]),
                         textcoords='data', zorder=zorder + 10)

    def draw_goal_region(self, obj: GoalRegion,
                         draw_params: Union[ParamServer, dict, None],
                         call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted
        :param draw_params: parameters for plotting given by a nested dict
        that recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)
        if call_stack is ():
            call_stack = tuple(['planning_problem_set'])
        call_stack = tuple(list(call_stack) + ['goal_region'])
        for goal_state in obj.state_list:
            self.draw_goal_state(goal_state, draw_params, call_stack)

    def draw_goal_state(self, obj: State,
                        draw_params: Union[ParamServer, dict, None],
                        call_stack: Tuple[str, ...]) -> None:
        """
        :param obj: object to be plotted
        :param draw_params: parameters for plotting given by a nested dict
        that recreates the structure of an object,
        :param call_stack: tuple of string containing the call stack,
        which allows for differentiation of plotting styles
               depending on the call stack of draw_object
        :return: None
        """
        draw_params = self._get_draw_params(draw_params)
        if hasattr(obj, 'position'):
            if type(obj.position) == list:
                for pos in obj.position:
                    pos.draw(self, draw_params, call_stack)
            else:
                obj.position.draw(self, draw_params, call_stack)

    def draw_traffic_light_sign(self, obj: Union[TrafficLight, TrafficSign],
                                draw_params: Union[ParamServer, dict, None],
                                call_stack: Tuple[str, ...]):
        draw_params = self._get_draw_params(draw_params)
        # traffic signs and lights have to be collected and drawn only right
        # before rendering to allow correct grouping
        self.traffic_sign_call_stack = call_stack
        self.traffic_sign_draw_params = draw_params
        self.traffic_signs.append(obj)

    def draw_signale_state(self, sig: SignalState, occ: Occupancy,
                           draw_params: Union[ParamServer, dict, None],
                           call_stack: Tuple[str, ...]):
        draw_params = self._get_draw_params(draw_params)

        zorder = draw_params.by_callstack(call_stack, 'zorder')
        signal_radius = draw_params.by_callstack(call_stack, 'signal_radius')
        indicator_color = draw_params.by_callstack(call_stack,
                                                   'indicator_color')
        braking_color = draw_params.by_callstack(call_stack, 'braking_color')
        horn_color = draw_params.by_callstack(call_stack, 'horn_color')
        blue_lights_color = draw_params.by_callstack(call_stack,
                                                     'blue_lights_color')
        indicators = []
        braking = []
        horns = []
        bluelights = []
        if isinstance(occ.shape, Rectangle):
            if hasattr(sig,
                       'hazard_warning_lights') and sig.hazard_warning_lights\
                    is True:
                indicators.extend([occ.shape.vertices[0], occ.shape.vertices[1],
                                   occ.shape.vertices[2],
                                   occ.shape.vertices[3]])
            else:
                if hasattr(sig,
                           'indicator_left') and sig.indicator_left is True:
                    indicators.extend(
                            [occ.shape.vertices[1], occ.shape.vertices[2]])
                if hasattr(sig,
                           'indicator_right') and sig.indicator_right is True:
                    indicators.extend(
                            [occ.shape.vertices[0], occ.shape.vertices[3]])
            if hasattr(sig, 'braking_lights') and sig.braking_lights is True:
                braking.extend([occ.shape.vertices[0], occ.shape.vertices[1]])
            if hasattr(sig,
                       'flashing_blue_lights') and sig.flashing_blue_lights \
                    is True:
                bluelights.append(occ.shape.center)
            if hasattr(sig, 'horn') and sig.horn is True:
                horns.append(occ.shape.center)
        else:
            warnings.warn('Plotting signal states only implemented for '
                          'obstacle_shapes Rectangle.')

        # draw signals
        if len(indicators) > 0:
            indicator_params = {
                    'facecolor': indicator_color,
                    'edgecolor': indicator_color,
                    'zorder':    zorder + ZOrders.INDICATOR_ADD,
                    'linewidth': 0
            }
            for e in indicators:
                self.draw_circle(Circle(signal_radius, e), indicator_params,
                                 call_stack)
        if len(braking) > 0:
            braking_params = {
                    'facecolor': braking_color,
                    'edgecolor': braking_color,
                    'zorder':    zorder + ZOrders.BRAKING_ADD,
                    'linewidth': 0
            }
            for e in indicators:
                self.draw_circle(Circle(signal_radius * 1.5, e), braking_params,
                                 call_stack)
        if len(horns) > 0:
            horn_params = {
                    'facecolor': horn_color,
                    'edgecolor': braking_color,
                    'zorder':    zorder + ZOrders.HORN_ADD,
                    'linewidth': 0
            }
            for e in indicators:
                self.draw_circle(Circle(signal_radius * 1.5, e), horn_params,
                                 call_stack)
        if len(bluelights) > 0:
            bluelight_params = {
                    'facecolor': blue_lights_color,
                    'edgecolor': braking_color,
                    'zorder':    zorder + ZOrders.BLUELIGHT_ADD,
                    'linewidth': 0
            }
            for e in indicators:
                self.draw_circle(Circle(signal_radius, e), bluelight_params,
                                 call_stack)
