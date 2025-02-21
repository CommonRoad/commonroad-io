from typing import Union

import numpy as np

from commonroad.geometry.transform import rotate_translate


def transform_to_global(
    vertices: list,
    pos_x: Union[int, float],
    pos_y: Union[int, float],
    orientation: Union[int, float],
    vehicle_length: Union[int, float],
    vehicle_width: Union[int, float],
):
    """Transform absolute coordinate to car-relative coordinate.

    Args:
        vertices: Shape: (N,2)
        pos_x: -
        pos_y: -
        orientation: -
        vehicle_length: -
        vehicle_width: -

    Returns:
        np_array: transformed absolute coordinate in the form (x,y) (shape: (N,2))
    """
    # Norm the array
    vertices = np.array(vertices)
    vertices = vertices * 0.01
    # Scale it to vehicle dim
    vertices[:, 0] = vertices[:, 0] * vehicle_length
    vertices[:, 1] = vertices[:, 1] * vehicle_width
    # Preprocess current pos
    curr_pos = np.array([pos_x, pos_y])
    vertices = rotate_translate(vertices, curr_pos, orientation)
    return vertices
