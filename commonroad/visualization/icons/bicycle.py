import math

import matplotlib as mpl
import numpy as np

from commonroad.geometry.shape import Rectangle, Shape
from commonroad.scenario.state import TraceState
from commonroad.visualization.icons.transform_to_global import transform_to_global


def draw_bicycle_icon(
    state: TraceState,
    shape: Shape = Rectangle(width=0.8, length=2.5),
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

    def elliptic_arc(center, major, minor, start_angle, end_angle):
        """Create the vertices of an elliptic arc."""
        arc = []
        angle_list = np.linspace(start_angle, end_angle, 50)
        for angle in angle_list:
            arc.append([center[0] + major * math.cos(angle), center[1] + minor * math.sin(angle)])

        return np.array(arc)

    # region Define your points in the norm square (-50<=x<=50, -50<=y<=50)
    # x -> length |  y -> width
    v_front_wheel = elliptic_arc((30, 0), 20, 6, 0, 2 * np.pi)
    v_rear_wheel = elliptic_arc((-30, 0), 20, 6, 0, 2 * np.pi)
    v_handlebar = np.array([[18, 50], [16, 50], [16, -50], [18, -50]])
    v_frame = np.array([[18, 3], [18, -3], [-30, -3], [-30, 3]])
    v_body = elliptic_arc((5, 0), 20, 40, np.pi / 2 + 0.2, np.pi * 3 / 2 - 0.2)
    v_arm_r = np.array(
        [
            v_body[-1],
            v_handlebar[3],
            [v_handlebar[3][0], v_handlebar[3][1] + 7.5],
            [v_body[-1][0], v_body[-1][1] + 15],
        ]
    )
    v_arm_l = np.array(
        [
            [v_body[0][0], v_body[0][1] - 15],
            [v_handlebar[0][0], v_handlebar[0][1] - 7.5],
            v_handlebar[0],
            v_body[0],
        ]
    )
    v_body = np.concatenate([v_body, v_arm_r, v_arm_l])
    v_head = elliptic_arc((3, 0), 6, 15, 0, 2 * np.pi)
    # endregion

    # Transform your coords
    list_bicycle = [v_front_wheel, v_frame, v_rear_wheel, v_handlebar, v_body, v_head]

    list_bicycle = [
        transform_to_global(
            vertices=part,
            pos_x=state.position[0],
            pos_y=state.position[1],
            orientation=state.orientation,
            vehicle_length=shape.length,
            vehicle_width=shape.width,
        )
        for part in list_bicycle
    ]
    patch_list = [
        mpl.patches.Polygon(
            part, fc=vehicle_color, ec=edgecolor, lw=lw, zorder=zorder, alpha=opacity, closed=True
        )
        for part in list_bicycle
    ]

    # Return this patch collection
    return patch_list
