from typing import List

import matplotlib as mpl
import numpy as np
from matplotlib.path import Path

from commonroad.geometry.shape import Rectangle, SemiTrailerTruck, Shape
from commonroad.geometry.transform import rotate_translate
from commonroad.scenario.state import TraceState


def _create_trailer_vertices(shape: SemiTrailerTruck) -> List[np.ndarray]:
    trailer_dist_from_front_to_origin = (
        shape.trailer_dist_from_front_to_hitch
        + shape.truck_dist_from_rear_to_hitch
        - shape.truck_dist_from_rear_to_rear_axle
    )
    trailer_dist_from_rear_to_origin = shape.trailer.length - trailer_dist_from_front_to_origin
    x_min = -trailer_dist_from_rear_to_origin
    x_max = trailer_dist_from_front_to_origin
    y_min_max = shape.trailer.width / 2
    v_trailer = np.array(
        [[x_min, -y_min_max], [x_max, -y_min_max], [x_max, y_min_max], [x_min, y_min_max]]
    )
    return [v_trailer]


def _create_truck_vertices(shape: SemiTrailerTruck) -> List[np.ndarray]:
    x_min = -shape.truck_dist_from_rear_to_rear_axle
    truck_dist_from_origin_to_front = shape.truck.length - shape.truck_dist_from_rear_to_rear_axle
    y_min_max = shape.truck.width / 2
    v_truck = np.array(
        [
            [x_min, -y_min_max],
            [truck_dist_from_origin_to_front, -y_min_max],
            [truck_dist_from_origin_to_front, y_min_max],
            [x_min, y_min_max],
        ]
    )
    cabin_x_min = truck_dist_from_origin_to_front - shape.cabin_length
    v_cabin = np.array(
        [
            [cabin_x_min, -y_min_max],
            [truck_dist_from_origin_to_front, -y_min_max],
            [truck_dist_from_origin_to_front, y_min_max],
            [cabin_x_min, y_min_max],
        ]
    )
    v_roof = v_cabin.copy()
    v_roof[[1, 2], 0] = 0.9 * v_roof[[1, 2], 0]  # scale x max
    v_roof[:, 1] = 0.8 * v_roof[:, 1]  # scale y
    v_a_col_l = np.array([v_roof[2], v_cabin[2]])
    v_a_col_r = np.array([v_roof[1], v_cabin[1]])
    mirror_x = truck_dist_from_origin_to_front - 0.15 * shape.cabin_length
    mirror_width = 0.1
    mirror_x_min = mirror_x - 0.5 * mirror_width
    mirror_x_max = mirror_x + 0.5 * mirror_width
    mirror_y_min = 0.5 * shape.truck.width
    mirror_y_max = 0.5 * shape.truck.width + 0.2
    v_mirror_l = np.array(
        [
            [mirror_x_max, mirror_y_min],
            [mirror_x_min, mirror_y_min],
            [mirror_x_min, mirror_y_max],
            [mirror_x_max, mirror_y_max],
        ]
    )
    v_mirror_r = np.array(
        [
            [mirror_x_max, -mirror_y_min],
            [mirror_x_min, -mirror_y_min],
            [mirror_x_min, -mirror_y_max],
            [mirror_x_max, -mirror_y_max],
        ]
    )
    return [v_truck, v_cabin, v_roof, v_a_col_l, v_a_col_r, v_mirror_l, v_mirror_r]


def rotate_by_hitch_angle(
    vertices: np.ndarray, shape: SemiTrailerTruck, hitch_angle: float
) -> np.ndarray:
    hitch_point = np.array(
        [shape.truck_dist_from_rear_to_hitch - shape.truck_dist_from_rear_to_rear_axle, 0]
    )
    translated_to_hitch_position = rotate_translate(vertices, -hitch_point, 0.0)
    rotated_by_hitch_angle = rotate_translate(
        translated_to_hitch_position, np.zeros(2), hitch_angle
    )
    translated_back = rotate_translate(rotated_by_hitch_angle, hitch_point, 0.0)
    return translated_back


def create_origin_patch(origin: np.ndarray):
    vertices = origin + 0.1 * np.array([(-0.5, -0.5), (0.5, 0.5), (0, 0), (0.5, -0.5), (-0.5, 0.5)])
    codes = [Path.MOVETO, Path.LINETO, Path.MOVETO, Path.LINETO, Path.LINETO]
    path = Path(vertices, codes)
    return mpl.patches.PathPatch(path, facecolor="none", edgecolor="black", lw=1, zorder=200)


def draw_truck_icon(
    state: TraceState,
    shape: Shape = SemiTrailerTruck.create_default(),
    zorder: float = 5,
    vehicle_color: str = "#ffffff",
    edgecolor="black",
    lw=0.5,
    opacity=1,
):
    """Return the patches of the truck icon.

    Define vertices in a normed rectangle.
    -50 <= x <= 50 and -50 <= y <= 50

    Credits to Tobias Geißenberger for defining the vertices.
    """
    # region Define your points in the norm square (-50<=x<=50, -50<=y<=50)
    # x -> length |  y -> width
    # endregion
    assert isinstance(shape, SemiTrailerTruck) or isinstance(shape, Rectangle)
    if isinstance(shape, Rectangle):
        shape = SemiTrailerTruck.from_rectangle(shape)

    trailer_components = _create_trailer_vertices(shape)
    truck_components = _create_truck_vertices(shape)
    truck_components_transformed = [
        rotate_translate(part, state.position, state.orientation) for part in truck_components
    ]

    hitch_angle = hasattr(state, "hitch_angle") and state.hitch_angle or 0.0

    trailer_components_transformed = [
        rotate_translate(
            rotate_by_hitch_angle(part, shape, hitch_angle), state.position, state.orientation
        )
        for part in trailer_components
    ]
    patch_list = [
        mpl.patches.Polygon(
            part, fc=vehicle_color, ec=edgecolor, lw=lw, zorder=zorder, alpha=opacity, closed=True
        )
        for part in truck_components_transformed + trailer_components_transformed
    ]

    patch_list += [create_origin_patch(state.position)]

    hitch_point = np.array(
        [[shape.truck_dist_from_rear_to_hitch - shape.truck_dist_from_rear_to_rear_axle, 0]]
    )
    hitch_point = rotate_translate(hitch_point, state.position, state.orientation)[0]
    patch_list += [mpl.patches.Circle(hitch_point, 0.1, color="black", alpha=0.5, zorder=200)]

    return patch_list
