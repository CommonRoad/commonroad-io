import warnings
import numpy as np
from commonroad.scenario.scenario import Scenario
from matplotlib.lines import Line2D
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.collections as collections
import matplotlib.pyplot as plt
from typing import List, Dict, Tuple, Union

from commonroad.scenario.lanelet import LaneletNetwork, LineMarking
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.traffic_sign import TrafficLightState, \
    TrafficLight, \
    TrafficLightDirection

import warnings
from typing import List, Dict, Tuple, Union

import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.collections as collections
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from commonroad.scenario.lanelet import LaneletNetwork, LineMarking
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_sign import TrafficLightState, \
    TrafficLight, \
    TrafficLightDirection
from matplotlib.lines import Line2D
from matplotlib.path import Path

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = [""]
__version__ = "2020.3"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


class LineDataUnits(Line2D):
    def __init__(self, *args, **kwargs):
        _lw_data = kwargs.pop("linewidth", 1)
        # _dashes_data = kwargs.pop("dashes", 1)
        super().__init__(*args, **kwargs)
        self._lw_data = _lw_data
        # self._dashes_data = _dashes_data

    def _get_lw(self):
        if self.axes is not None:
            ppd = 72./self.axes.figure.dpi
            trans = self.axes.transData.transform
            return ((trans((1, self._lw_data))-trans((0, 0)))*ppd)[1]
        else:
            return 1

    def _set_lw(self, lw):
        self._lw_data = lw

    # def _get_dashes(self):
    #     if self.axes is not None:
    #         ppd = 72. / self.axes.figure.dpi
    #         trans = self.axes.transData.transform
    #         if not None in self._dashes_data:
    #             scale0 = ((trans((1, self._dashes_data[0])) - trans((0, 0))) * ppd)[1]
    #             scale1 = ((trans((1, self._dashes_data[1])) - trans((0, 0))) * ppd)[1]
    #             return (scale0, scale1)
    #         else:
    #             return self._dashes_data
    #     else:
    #         return (1,1)
    #
    # def _set_dashes(self, dashes):
    #     self._dashes_data = dashes


    _linewidth = property(_get_lw, _set_lw)
    # _dashes = property(_get_dashes, _set_dashes)


def draw_polygon_as_patch(vertices, ax, zorder=5, facecolor='#ffffff',
                          edgecolor='#000000', lw=0.5, alpha=1.0) -> mpl.patches.Patch:
    """
    vertices are no closed polygon (first element != last element)
    """
    verts = []
    codes = [Path.MOVETO]
    for p in vertices:
        verts.append(p)
        codes.append(Path.LINETO)
    del codes[-1]
    codes.append(Path.CLOSEPOLY)
    verts.append((0, 0))

    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=facecolor, edgecolor=edgecolor,
                              lw=lw, zorder=zorder, alpha=alpha)
    ax.add_patch(patch)

    return patch


def draw_polygon_collection_as_patch(vertices: List[list], ax, zorder=5, facecolor='#ffffff',
                          edgecolor='#000000', lw=0.5, alpha=1.0, antialiased=True) -> mpl.collections.Collection:
    """
    vertices are no closed polygon (first element != last element)
    """
    path_list = list()
    for v in vertices:
        verts = []
        codes = [Path.MOVETO]
        for p in v:
            verts.append(p)
            codes.append(Path.LINETO)
        del codes[-1]
        codes.append(Path.CLOSEPOLY)
        verts.append((0, 0))

        path_list.append(Path(verts, codes))
        collection_tmp = collections.PathCollection(path_list, facecolor=facecolor, edgecolor=edgecolor,
                                  lw=lw, zorder=zorder, alpha=alpha, antialiaseds=antialiased)
        collection = ax.add_collection(collection_tmp)

    return collection


def collect_center_line_colors(lanelet_network:LaneletNetwork, traffic_lights: List[TrafficLight], time_step)\
        -> Dict[int,TrafficLightState]:
    """Collects traffic light states that each lanelet is affected by."""
    def update_state_dict(new_dict: Dict[int, TrafficLightState]):
        """ Updates state in dict. If new_state is inactive, an existing state is not overwritten."""
        for lanelet_id, new_state in new_dict.items():
            if lanelet_id in l2state:
                if new_state == TrafficLightState.INACTIVE or \
                   (new_state == TrafficLightState.RED and l2state[lanelet_id] == TrafficLightState.GREEN):
                    continue

            l2state[lanelet_id] = new_state

    l2int = lanelet_network.map_inc_lanelets_to_intersections
    l2state = {}
    for lanelet in lanelet_network.lanelets:
        intersection = l2int[lanelet.lanelet_id] if lanelet.lanelet_id in l2int else None
        for tl_id in lanelet.traffic_lights:
            tl = lanelet_network.find_traffic_light_by_id(tl_id)
            direction = tl.direction
            state = tl.get_state_at_time_step(time_step)
            if direction == TrafficLightDirection.ALL:
                update_state_dict({succ_id: state for succ_id in lanelet.successor})
            elif intersection is not None:
                inc_ele = intersection.map_incoming_lanelets[lanelet.lanelet_id]
                if direction in (TrafficLightDirection.RIGHT, TrafficLightDirection.LEFT_RIGHT,
                                 TrafficLightDirection.STRAIGHT_RIGHT):
                    update_state_dict({l: state for l in inc_ele.successors_right})
                if direction in (TrafficLightDirection.LEFT, TrafficLightDirection.LEFT_RIGHT,
                                 TrafficLightDirection.LEFT_STRAIGHT):
                    update_state_dict({l: state for l in inc_ele.successors_left})
                if direction in (TrafficLightDirection.STRAIGHT, TrafficLightDirection.STRAIGHT_RIGHT,
                                 TrafficLightDirection.LEFT_STRAIGHT):
                    update_state_dict({l: state for l in inc_ele.successors_straight})
            elif len(lanelet.successor) == 1:
                update_state_dict({lanelet.successor[0]: state})
            else:
                warnings.warn('Direction of traffic light cannot be visualized.')

    return l2state


def approximate_bounding_box_dyn_obstacles(obj: list, time_step=0) -> Union[Tuple[list], None]:
    """
    Compute bounding box of dynamic obstacles at time step
    :param obj: All possible objects. DynamicObstacles are filtered.
    :return:
    """
    def update_bounds(new_point: np.ndarray, bounds:List[list]):
        """Update bounds with new point"""
        if new_point[0] < bounds[0][0]:
            bounds[0][0] = new_point[0]
        if new_point[1] < bounds[1][0]:
            bounds[1][0] = new_point[1]
        if new_point[0] > bounds[0][1]:
            bounds[0][1] = new_point[0]
        if new_point[1] > bounds[1][1]:
            bounds[1][1] = new_point[1]

        return bounds

    dynamic_obstacles_filtered = []
    for o in obj:
        if type(o) == DynamicObstacle:
            dynamic_obstacles_filtered.append(o)
        elif type(o) == Scenario:
            dynamic_obstacles_filtered.extend(o.dynamic_obstacles)

    x_int = [np.inf, -np.inf]
    y_int = [np.inf, -np.inf]
    bounds = [x_int, y_int]

    for obs in dynamic_obstacles_filtered:
        occ = obs.occupancy_at_time(time_step)
        if occ is None: continue
        shape = occ.shape
        if hasattr(shape, 'center'):  # Rectangle, Circle
            bounds = update_bounds(shape.center, bounds=bounds)
        elif hasattr(shape, 'vertices'):  # Polygon, Triangle
            v = shape.vertices
            bounds = update_bounds(np.min(v, axis=0), bounds=bounds)
            bounds = update_bounds(np.max(v, axis=0), bounds=bounds)

    if np.inf in bounds[0] or -np.inf in bounds[0] or np.inf in bounds[
        1] or -np.inf in bounds[1]:
        return None
    else:
        return tuple(bounds)


def get_arrow_path_at(x, y, angle):
    """Returns path of arrow shape"""
    # direction arrow
    codes_direction = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]

    scale_direction = 1.5
    pts = np.array([[0.0, -0.5, 1.0], [1.0, 0.0, 1.0], [0.0, 0.5, 1.0],
                    [0.0, -0.5, 1.0]])
    scale_m = np.array(
            [[scale_direction, 0, 0], [0, scale_direction, 0], [0, 0, 1]])
    transform = np.array([[np.cos(angle), -np.sin(angle), x],
                          [np.sin(angle), np.cos(angle), y], [0, 0, 1]])
    ptr_trans = transform.dot(scale_m.dot(pts.transpose()))
    ptr_trans = ptr_trans[0:2, :]
    ptr_trans = ptr_trans.transpose()

    path = Path(ptr_trans, codes_direction)
    return path


def colormap_idx(max_x):
    norm = mpl.colors.Normalize(vmin=0, vmax=max_x)
    colormap = cm.ScalarMappable(norm=norm, cmap=cm.jet)
    # Closure
    return lambda x: colormap.to_rgba(x)


def get_car_patch(pos_x: Union[int, float], pos_y: Union[int, float],
                  rotate: Union[int, float], scale: Union[int, float],
                  zorder: float = 5, carcolor: str = '#ffffff', lw=0.5):
    rotate = rotate + np.pi

    def reshape_and_addup(verts):
        verts = verts.reshape((int(len(verts) / 2), 2))
        for i in range(1, len(verts)):
            verts[i] = verts[i] + verts[i - 1]
        return verts

    def transform(vertices, pos_x, pos_y, rotate, scale):
        vertices = np.asarray(np.bmat([vertices, np.ones((len(vertices), 1))]))
        tmat = np.array([[np.cos(rotate), -np.sin(rotate), pos_x],
                         [np.sin(rotate), np.cos(rotate), pos_y], [0, 0, 1]])
        scalemat = np.array(
                [[scale * 1 / 356.3211, 0, 0], [0, scale * 1 / 356.3211, 0],
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

    verts1 = transform(verts1, pos_x, pos_y, rotate, scale)
    verts2 = transform(verts2, pos_x, pos_y, rotate, scale)
    verts3 = transform(verts3, pos_x, pos_y, rotate, scale)
    verts4 = transform(verts4, pos_x, pos_y, rotate, scale)
    verts5 = transform(verts5, pos_x, pos_y, rotate, scale)
    verts6 = transform(verts6, pos_x, pos_y, rotate, scale)
    verts7 = transform(verts7, pos_x, pos_y, rotate, scale)
    verts8 = transform(verts8, pos_x, pos_y, rotate, scale)

    windowcolor = '#555555'
    patch_list = [
            mpl.patches.Polygon(verts1, closed=True, facecolor=windowcolor,
                                zorder=zorder + 1, lw=lw),
            mpl.patches.Polygon(verts2, closed=True, facecolor=windowcolor,
                                zorder=zorder + 1, lw=lw),
            mpl.patches.Polygon(verts3, closed=True, facecolor=windowcolor,
                                zorder=zorder + 1, lw=lw),
            mpl.patches.Polygon(verts4, closed=True, facecolor=windowcolor,
                                zorder=zorder + 1, lw=lw),
            # mpl.patches.PathPatch(Path(verts5, closed=True), fill=False,
            # zorder=zorder + 1,
            #                       color='#000000', lw=lw),
            mpl.patches.Polygon(verts6, closed=True, facecolor=windowcolor,
                                zorder=zorder + 1, lw=lw),
            mpl.patches.Polygon(verts7, closed=True, facecolor=windowcolor,
                                zorder=zorder + 1, lw=lw),
            mpl.patches.Polygon(verts8, closed=True, edgecolor='#000000',
                                zorder=zorder, lw=lw)]
    return patch_list


def set_non_blocking() -> None:
    """
    Ensures that interactive plotting is enabled for non-blocking plotting.

    :return: None
    """

    plt.ion()
    if not mpl.is_interactive():
        warnings.warn(
            'The current backend of matplotlib does not support interactive '
            'mode: ' + str(
                mpl.get_backend()) + '. Select another backend with: '
                                     '\"matplotlib.use(\'TkAgg\')\"',
            UserWarning, stacklevel=3)


def line_marking_to_linestyle(line_marking: LineMarking) -> Tuple:
    """:returns: Tuple[line_style, dashes, line_width] for matplotlib
    plotting options."""
    return {
            LineMarking.DASHED:       ('--', (10, 10), 0.25,),
            LineMarking.SOLID:        ('-', (None, None), 0.25),
            LineMarking.BROAD_DASHED: ('--', (10, 10), 0.5),
            LineMarking.BROAD_SOLID:  ('-', (None, None), 0.5)
    }[line_marking]


def traffic_light_color_dict(traffic_light_state: TrafficLightState,
                             params: dict):
    """Retrieve color code for traffic light state."""
    return {
            TrafficLightState.RED:        params['red_color'],
            TrafficLightState.YELLOW:     params['yellow_color'],
            TrafficLightState.GREEN:      params['green_color'],
            TrafficLightState.RED_YELLOW: params['red_yellow_color']
    }[traffic_light_state]
