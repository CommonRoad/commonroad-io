import typing
from typing import List, Union

import matplotlib as mpl
import numpy as np
import shapely
from matplotlib.path import Path

from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.obstacle_shapes.semi_trailer_truck_shape import SemiTrailerTruckShape
from commonroad.geometry.obstacle_shapes.truck_shape import TruckShape
from commonroad.geometry.occupancy.occupancy_group import OccupancyGroup
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.geometry.transform import rotate_translate
from commonroad.geometry.transform_shapely_shape import rotate_and_translate
from commonroad.scenario.state import TraceState


def _create_additional_truck_components(
    truck_shape: TruckShape, state: TraceState
) -> List[np.ndarray]:
    truck_dist_from_origin_to_front = (
        truck_shape.truck_dims.length / 2.0 - truck_shape.origin_x_shift
    )
    y_max = truck_shape.truck_dims.width / 2

    cabin_x_min = truck_dist_from_origin_to_front - truck_shape.truck_dims.cabin_length
    v_cabin = np.array(
        [
            [cabin_x_min, -y_max],
            [truck_dist_from_origin_to_front, -y_max],
            [truck_dist_from_origin_to_front, y_max],
            [cabin_x_min, y_max],
        ]
    )

    v_roof = v_cabin.copy()
    v_roof[[1, 2], 0] = cabin_x_min + 0.9 * truck_shape.truck_dims.cabin_length  # scale x max
    v_roof[:, 1] = 0.8 * v_roof[:, 1]  # scale y
    v_a_col_l = np.array([v_roof[2], v_cabin[2]])
    v_a_col_r = np.array([v_roof[1], v_cabin[1]])
    mirror_x = truck_dist_from_origin_to_front - 0.15 * truck_shape.truck_dims.cabin_length
    mirror_width = 0.1
    mirror_x_min = mirror_x - 0.5 * mirror_width
    mirror_x_max = mirror_x + 0.5 * mirror_width
    mirror_y_min = 0.5 * truck_shape.truck_dims.width
    mirror_y_max = 0.5 * truck_shape.truck_dims.width + 0.2
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

    additional_truck_components = [v_cabin, v_roof, v_a_col_l, v_a_col_r, v_mirror_l, v_mirror_r]

    return [
        rotate_translate(part, state.position, state.orientation)
        for part in additional_truck_components
    ]


def create_origin_patch(origin: np.ndarray):
    vertices = origin + 0.1 * np.array([(-0.5, -0.5), (0.5, 0.5), (0, 0), (0.5, -0.5), (-0.5, 0.5)])
    codes = [Path.MOVETO, Path.LINETO, Path.MOVETO, Path.LINETO, Path.LINETO]
    path = Path(vertices, codes)
    return mpl.patches.PathPatch(path, facecolor="none", edgecolor="black", lw=1, zorder=200)


def draw_semi_trailer_truck_icon(
    state: TraceState,
    shape: Union[SemiTrailerTruckShape, RectObstacleShape] = SemiTrailerTruckShape.create_default(),
    zorder: float = 5,
    vehicle_color: str = "#ffffff",
    edgecolor="black",
    lw=0.5,
    opacity=1,
    show_ref_position: bool = False,
):
    """Return the patches of the truck icon.

    Define vertices in a normed rectangle.
    -50 <= x <= 50 and -50 <= y <= 50

    Credits to Tobias Geißenberger for defining the vertices.
    """
    # region Define your points in the norm square (-50<=x<=50, -50<=y<=50)
    # x -> length |  y -> width
    # endregion
    assert isinstance(shape, SemiTrailerTruckShape) or isinstance(shape, RectObstacleShape)
    if isinstance(shape, RectObstacleShape):
        shape = SemiTrailerTruckShape.with_length(shape.length)

    occ = typing.cast(OccupancyGroup, shape.compute_occupancy_for_state(state))
    truck_shape = typing.cast(RectOccupancy, occ.occupancies[0])
    trailer_shape = typing.cast(RectOccupancy, occ.occupancies[1])
    # exclude closing vertex:
    truck_vertices = truck_shape.vertices[:-1]
    trailer_vertices = trailer_shape.vertices[:-1]

    additional_truck_components = _create_additional_truck_components(shape.truck_shape, state)

    patch_list = [
        mpl.patches.Polygon(
            part, fc=vehicle_color, ec=edgecolor, lw=lw, zorder=zorder, alpha=opacity, closed=True
        )
        for part in [trailer_vertices, truck_vertices] + additional_truck_components
    ]

    if show_ref_position:
        patch_list += [create_origin_patch(state.position)]

    hitch_point = shapely.Point(shape.hitch_shift_from_origin, 0)
    hitch_point = rotate_and_translate(
        hitch_point, state.orientation, state.position[0], state.position[1]
    )
    patch_list += [mpl.patches.Circle(hitch_point.xy, 0.1, color="black", alpha=0.5, zorder=200)]

    return patch_list


def draw_truck_only_icon(
    state: TraceState,
    shape: Union[TruckShape, RectObstacleShape] = TruckShape.create_default(),
    zorder: float = 5,
    vehicle_color: str = "#ffffff",
    edgecolor="black",
    lw=0.5,
    opacity=1,
    show_ref_position: bool = False,
):
    """Return the patches of the truck icon.

    Define vertices in a normed rectangle.
    -50 <= x <= 50 and -50 <= y <= 50

    Credits to Tobias Geißenberger for defining the vertices.
    """
    # region Define your points in the norm square (-50<=x<=50, -50<=y<=50)
    # x -> length |  y -> width
    # endregion
    assert isinstance(shape, TruckShape) or isinstance(shape, RectObstacleShape)
    if isinstance(shape, RectObstacleShape):
        shape = TruckShape.with_length(shape.length)

    occ = typing.cast(RectOccupancy, shape.compute_occupancy_for_state(state))
    # exclude closing vertex:
    truck_vertices = occ.vertices[:-1]

    additional_truck_components = _create_additional_truck_components(shape, state)

    patch_list = [
        mpl.patches.Polygon(
            part, fc=vehicle_color, ec=edgecolor, lw=lw, zorder=zorder, alpha=opacity, closed=True
        )
        for part in [truck_vertices] + additional_truck_components
    ]

    if show_ref_position:
        patch_list += [create_origin_patch(state.position)]

    return patch_list


def draw_truck_icon(
    state: TraceState,
    shape: Union[
        SemiTrailerTruckShape, RectObstacleShape, TruckShape
    ] = SemiTrailerTruckShape.create_default(),
    zorder: float = 5,
    vehicle_color: str = "#ffffff",
    edgecolor="black",
    lw=0.5,
    opacity=1,
    show_ref_position: bool = False,
):
    if isinstance(shape, SemiTrailerTruckShape) or (
        isinstance(shape, RectObstacleShape) and shape.length > 7.0
    ):
        return draw_semi_trailer_truck_icon(
            state,
            shape=shape,
            zorder=zorder,
            vehicle_color=vehicle_color,
            edgecolor=edgecolor,
            lw=lw,
            opacity=opacity,
            show_ref_position=show_ref_position,
        )
    else:
        return draw_truck_only_icon(
            state,
            shape=shape,
            zorder=zorder,
            vehicle_color=vehicle_color,
            edgecolor=edgecolor,
            lw=lw,
            opacity=opacity,
            show_ref_position=show_ref_position,
        )
