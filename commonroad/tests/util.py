from typing import List

import numpy as np

from commonroad.scenario.lanelet import Lanelet


def parallel_lanelets(num_lanes, lanelet_width=4.0, lanelet_length=90.0) -> List[Lanelet]:
    """
    Defines parallel lanelets along the x-axis.
    Lanelet IDs are increasing from right to left. Right boundary of lanelet 0 is at x=0
    :return: List of lanelets
    """
    lon_step = 10
    lanelets = []
    for i in range(num_lanes):
        # Lanes from right to left
        right_y = i * lanelet_width
        left_y = (i + 1) * lanelet_width
        center_y = (i + 0.5) * lanelet_width
        x_points = np.arange(start=0, stop=lanelet_length + lon_step, step=lon_step)
        ones = np.ones((x_points.shape[0]))
        right_vertices_lane = np.stack((x_points, ones * right_y), axis=1)
        left_vertices_lane = np.stack((x_points, ones * left_y), axis=1)
        center_vertices_lane = np.stack((x_points, ones * center_y), axis=1)
        if i == 0:
            adjacent_right = None
        else:
            adjacent_right = i - 1
        if i == num_lanes - 1:
            adjacent_left = None
        else:
            adjacent_left = i + 1
        lanelets.append(Lanelet(left_vertices_lane, center_vertices_lane, right_vertices_lane, lanelet_id=i,
                adjacent_left=adjacent_left, adjacent_right=adjacent_right, adjacent_right_same_direction=True,
                adjacent_left_same_direction=True, ))
    return lanelets
