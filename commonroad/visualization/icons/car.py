import matplotlib as mpl
import numpy as np

from commonroad.geometry.shape import Rectangle, Shape
from commonroad.scenario.state import TraceState
from commonroad.visualization.icons.transform_to_global import transform_to_global


def draw_car_icon(
    state: TraceState,
    shape: Shape = Rectangle(length=5.0, width=2.0),
    zorder: float = 5,
    vehicle_color: str = "#ffffff",
    edgecolor="black",
    lw=0.5,
    opacity=1,
    show_ref_position: bool = False,
):
    """Return the patches of the car icon.

    Define vertices in a normed rectangle.
    -50 <= x <= 50 and -50 <= y <= 50
    """
    window_color = edgecolor

    front_window = np.array(
        [
            [-21.36, -38.33],
            [-23.93, -27.66],
            [-24.98, -12.88],
            [-25.28, -0.3],
            [-25.29, -0.3],
            [-25.28, -0.04],
            [-25.29, 0.22],
            [-25.28, 0.22],
            [-24.98, 12.8],
            [-23.93, 27.58],
            [-21.36, 38.24],
            [-14.65, 36.18],
            [-7.64, 33.19],
            [-8.32, 19.16],
            [-8.62, -0.04],
            [-8.32, -19.24],
            [-7.64, -33.27],
            [-14.65, -36.27],
        ]
    )

    rear_window = np.array(
        [
            [37.68, -34.02],
            [26.22, -32.15],
            [27.43, -14.56],
            [27.8, -0.41],
            [27.43, 13.74],
            [26.22, 31.32],
            [37.68, 33.19],
            [40.17, 21.22],
            [41.3, -0.34],
            [40.17, -21.91],
            [40.17, -21.91],
        ]
    )

    left_window = np.array(
        [
            [4.32, -38.7],
            [25.84, -37.76],
            [27.35, -36.27],
            [15.06, -32.71],
            [-0.1, -32.71],
            [-13.6, -37.95],
            [0.84, -38.78],
        ]
    )

    left_mirror = np.array(
        [
            [-12.62, -49.78],
            [-13.3, -50.0],
            [-15.11, -46.63],
            [-16.78, -41.24],
            [-17.23, -39.56],
            [-14.92, -39.45],
            [-14.52, -40.68],
            [-13.97, -41.47],
        ]
    )

    engine_hood = np.array(
        [
            [-21.67, -38.04],
            [-32.98, -34.96],
            [-40.1, -29.77],
            [-46.78, -18.96],
            [-49.04, 2.65],
            [-46.78, 19.35],
            [-40.33, 29.6],
            [-32.98, 35.35],
            [-21.67, 38.44],
        ]
    )

    right_window = np.array(
        [
            [4.32, 38.7],
            [25.84, 37.76],
            [27.35, 36.27],
            [15.06, 32.71],
            [-0.1, 32.71],
            [-13.6, 37.95],
            [0.84, 38.78],
        ]
    )

    right_mirror = np.array(
        [
            [-12.62, 49.78],
            [-13.3, 50.0],
            [-15.11, 46.63],
            [-16.78, 41.24],
            [-17.23, 39.56],
            [-14.92, 39.45],
            [-14.52, 40.68],
            [-13.97, 41.47],
        ]
    )

    outline = np.array(
        [
            [0.78, -45.23],
            [-38.09, -42.38],
            [-45.85, -36.08],
            [-49.16, -15.15],
            [-49.99, 1.79],
            [-50.0, 1.79],
            [-50.0, 2.0],
            [-50.0, 2.22],
            [-49.99, 2.22],
            [-49.16, 14.1],
            [-45.85, 35.03],
            [-38.09, 41.33],
            [0.78, 44.18],
            [30.15, 42.88],
            [44.88, 37.96],
            [47.6, 32.77],
            [49.58, 14.36],
            [50.0, 3.86],
            [50.0, 0.14],
            [49.58, -15.41],
            [47.6, -33.82],
            [44.88, -39.01],
            [30.15, -43.93],
        ]
    )

    windows = [-front_window, -rear_window, -left_window, -right_window]
    car = [-outline, -left_mirror, -right_mirror, -engine_hood]

    windows = [
        transform_to_global(
            vertices=window,
            pos_x=state.position[0],
            pos_y=state.position[1],
            orientation=state.orientation,
            vehicle_length=shape.length,
            vehicle_width=shape.width,
        )
        for window in windows
    ]
    car = [
        transform_to_global(
            vertices=part,
            pos_x=state.position[0],
            pos_y=state.position[1],
            orientation=state.orientation,
            vehicle_length=shape.length,
            vehicle_width=shape.width,
        )
        for part in car
    ]

    window_patches = [
        mpl.patches.Polygon(
            window,
            fc=window_color,
            ec=edgecolor,
            lw=lw,
            alpha=opacity,
            zorder=zorder,
            closed=True,
        )
        for window in windows
    ]
    car_patches = [
        mpl.patches.Polygon(
            part, fc=vehicle_color, ec=edgecolor, lw=lw, alpha=opacity, zorder=zorder, closed=True
        )
        for part in car
    ]

    return car_patches + window_patches
