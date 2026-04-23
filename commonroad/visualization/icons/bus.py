import matplotlib as mpl
import numpy as np

from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.scenario.state import TraceState
from commonroad.visualization.icons.transform_to_global import transform_to_global


def draw_bus_icon(
    state: TraceState,
    shape: RectObstacleShape = RectObstacleShape(width=2.5, length=12.0),
    zorder: float = 5,
    vehicle_color: str = "#ffffff",
    edgecolor="black",
    lw=0.5,
    opacity=1,
    **kwargs,
):
    """Return the patches of the truck icon.

    Define vertices in a normed rectangle.
    -50 <= x <= 50 and -50 <= y <= 50
    """
    window_color = edgecolor

    outline = np.array([[-50, -50], [50, -50], [50, 50], [-50, 50]])
    front_window = np.array([[47, -42], [50, -46], [50, 46], [47, 42]])
    right_window = np.array([[-20, -50], [-15, -42], [40, -42], [45, -50]])
    left_window = np.array([[-20, 50], [-15, 42], [40, 42], [45, 50]])
    roof_hatch = np.array([[-40, -27], [-15, -27], [-15, 27], [-40, 27]])
    hatch_circles = [[-35, 0], [-27.5, 0], [-20, 0]]
    roof_line = np.array([[-7, -27], [-7, 27]])
    bus_list = [outline, roof_hatch, roof_line]
    window_list = [front_window, right_window, left_window]

    bus_list = [
        transform_to_global(
            vertices=part,
            pos_x=state.position[0],
            pos_y=state.position[1],
            orientation=state.orientation,
            vehicle_length=shape.length,
            vehicle_width=shape.width,
        )
        for part in bus_list
    ]
    window_list = [
        transform_to_global(
            vertices=window,
            pos_x=state.position[0],
            pos_y=state.position[1],
            orientation=state.orientation,
            vehicle_length=shape.length,
            vehicle_width=shape.width,
        )
        for window in window_list
    ]
    hatch_circles = transform_to_global(
        vertices=hatch_circles,
        pos_x=state.position[0],
        pos_y=state.position[1],
        orientation=state.orientation,
        vehicle_length=shape.length,
        vehicle_width=shape.width,
    )

    bus_list_patches = [
        mpl.patches.Polygon(
            part,
            fc=vehicle_color,
            ec=edgecolor,
            lw=lw,
            zorder=zorder,
            alpha=opacity,
            closed=True,
        )
        for part in bus_list
    ]
    window_list_patches = [
        mpl.patches.Polygon(
            window,
            fc=window_color,
            ec=edgecolor,
            lw=lw,
            zorder=zorder,
            alpha=opacity,
            closed=True,
        )
        for window in window_list
    ]
    hatch_circle_patches = [
        mpl.patches.Circle(
            point,
            radius=shape.length * 2.5 / 100,
            facecolor=vehicle_color,
            zorder=zorder,
            alpha=opacity,
            linewidth=lw,
            edgecolor=edgecolor,
        )
        for point in hatch_circles
    ]

    return bus_list_patches + window_list_patches + hatch_circle_patches
