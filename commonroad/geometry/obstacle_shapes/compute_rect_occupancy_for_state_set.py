import numpy as np
import shapely

from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.scenario.state import TraceState


def compute_rect_occupancy_for_state_set(state: TraceState, w_v: float, l_v: float) -> Occupancy:
    # From M. Althoff and J. M. Dolan, “Online Verification of Automated Road Vehicles Using Reachability Analysis,”
    # IEEE Transactions on Robotics, vol. 30, no. 4, pp. 903–918, Aug. 2014, doi: 10.1109/TRO.2014.2312453.
    # Section IV.C
    if state.is_uncertain_orientation:
        # Using middle of orientation interval as reference orientation
        psi_d = state.orientation.start + 0.5 * state.orientation.length
        delta_psi = 0.5 * state.orientation.length
    else:
        psi_d = state.orientation
        delta_psi = 0.0

    if state.is_uncertain_position:
        center = state.position.center
        position_area_at_origin = state.position.translate_rotate(-center.x, -center.y, -psi_d)
        enclosing_rect = position_area_at_origin.enclosing_axis_aligned_rect()
        l_s = enclosing_rect.length
        w_s = enclosing_rect.width
    else:
        l_s = 0.0
        w_s = 0.0
        center = shapely.Point(state.position)

    # Maximum enlargement at these angles
    delta_psi_l = min(delta_psi, np.arctan(w_v / l_v))
    delta_psi_w = min(delta_psi, np.arctan(l_v / w_v))

    l_psi = np.abs((1.0 - np.cos(delta_psi_l)) * l_v - np.sin(delta_psi_l) * w_v)
    w_psi = np.abs((1.0 - np.cos(delta_psi_w)) * w_v - np.sin(delta_psi_w) * l_v)

    l_enclosing = l_s + l_v + l_psi
    w_enclosing = w_s + w_v + w_psi

    return RectOccupancy(
        rect_center=center, width=w_enclosing, length=l_enclosing, orientation=psi_d
    )
