"""Module for drawing obstacle icons."""

from commonroad.geometry.shape import Shape
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.state import TraceState
from commonroad.visualization.icons.bicycle import draw_bicycle_icon
from commonroad.visualization.icons.bus import draw_bus_icon
from commonroad.visualization.icons.car import draw_car_icon
from commonroad.visualization.icons.truck import draw_truck_icon


def _obstacle_icon_assignment():
    """Assign obstacle type to icon."""
    assign_dict = {
        ObstacleType.CAR: draw_car_icon,
        ObstacleType.PARKED_VEHICLE: draw_car_icon,
        ObstacleType.TAXI: draw_car_icon,
        ObstacleType.TRUCK: draw_truck_icon,
        ObstacleType.BUS: draw_bus_icon,
        ObstacleType.BICYCLE: draw_bicycle_icon,
    }

    return assign_dict


def supported_icons():
    """Return a list of obstacle types, that have a icon."""
    return list(_obstacle_icon_assignment().keys())


def get_obstacle_icon_patch(
    obstacle_type: ObstacleType,
    state: TraceState,
    shape: Shape,
    zorder: float = 5,
    vehicle_color: str = "#ffffff",
    edgecolor="black",
    lw=0.5,
    opacity=1,
    show_ref_position: bool = False,
):
    """Get a list of mpl.patches to draw a obstacle specific icon."""
    if obstacle_type not in supported_icons():
        error_string = (
            f"There is no icon available for vehicle type: {str(obstacle_type)}\n\nEnsure to call the "
            f"get_obstacle_icon_patch(...) function\nonly for vehicle types supported.\nThese can be "
            f"retrieved by "
            f"calling commonroad.visualization.icons.supported_icons()"
        )
        raise TypeError(error_string)
    draw_func = _obstacle_icon_assignment()[obstacle_type]
    patch = draw_func(
        state=state,
        shape=shape,
        zorder=zorder,
        vehicle_color=vehicle_color,
        edgecolor=edgecolor,
        lw=lw,
        opacity=opacity,
        show_ref_position=show_ref_position,
    )
    return patch
