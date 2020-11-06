import warnings
import numpy as np
from commonroad.scenario.scenario import Scenario
from matplotlib.lines import Line2D
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib as mpl
import matplotlib.collections as collections
from typing import List, Dict, Tuple, Union

from commonroad.common.util import Interval
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.traffic_sign import TrafficLightState, TrafficLight, TrafficLightDirection

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

    if np.inf in bounds[0] or -np.inf in bounds[0] or np.inf in bounds[1] or -np.inf in bounds[1]:
        return None
    else:
        return tuple(bounds)