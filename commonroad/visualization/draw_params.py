import dataclasses
import inspect
from dataclasses import dataclass, field
import pathlib
from typing import Any, Optional, Dict, TypeVar, Union, List

from omegaconf import OmegaConf

Color = str


def _dict_to_params(dict_params, cls):
    fields = dataclasses.fields(cls)
    cls_map = {f.name: f.type for f in fields}
    kwargs = {}
    for k, v in cls_map.items():
        if k not in dict_params:
            continue
        if inspect.isclass(v) and issubclass(v, BaseParam):
            kwargs[k] = _dict_to_params(dict_params[k], cls_map[k])
        else:
            kwargs[k] = dict_params[k]
    return cls(**kwargs)


@dataclass
class BaseParam:
    #: First time step of the visualized time interval.
    time_begin: int = 0
    #: Last time step of the visualized time interval.
    time_end: int = 200
    #: Use anti-aliasing.
    antialiased: bool = True
    __initialized: bool = field(init=False, default=False, repr=False)

    def __post_init__(self):
        self.__initialized = True
        # Make sure that the base parameters are propagated to all sub-parameters
        # This cannot be done in the init method, because the sub-parameters are not yet initialized.
        # This is not a noop, as it calls the __setattr__ method.
        # Do not remove!
        self.time_begin = self.time_begin
        self.time_end = self.time_end
        self.antialiased = self.antialiased

    def __setattr__(self, name: str, value: Any) -> None:
        if name in {f.name for f in dataclasses.fields(self)}:
            super().__setattr__(name, value)
        if self.__initialized:
            for k, v in self.__dict__.items():
                if isinstance(v, BaseParam):
                    v.__setattr__(name, value)

    def __getitem__(self, item):
        try:
            value = self.__getattribute__(item)
        except AttributeError:
            raise KeyError(f"{item} is not a parameter of {self.__class__.__name__}")
        return value

    def __setitem__(self, key, value):
        try:
            self.__setattr__(key, value)
        except AttributeError:
            raise KeyError(f"{key} is not a parameter of {self.__class__.__name__}")

    @classmethod
    def load(cls, file_path: Union[pathlib.Path, str], validate_types: bool = True):
        file_path = pathlib.Path(file_path)
        assert file_path.suffix == ".yaml", f"File type {file_path.suffix} is unsupported! Please use .yaml!"
        loaded_yaml = OmegaConf.load(file_path)
        if validate_types:
            OmegaConf.merge(OmegaConf.structured(MPDrawParams), loaded_yaml)
        params = _dict_to_params(OmegaConf.to_object(loaded_yaml), cls)
        return params

    def save(self, file_path: Union[pathlib.Path, str]):
        # Avoid saving private attributes
        dict_cfg = dataclasses.asdict(self, dict_factory=lambda items: {key: val for key, val in items if
                                                                        not key.startswith("_")})
        OmegaConf.save(OmegaConf.create(dict_cfg), file_path, resolve=True)


@dataclass
class ShapeParams(BaseParam):
    opacity: float = 1.0
    facecolor: Color = "#1d7eea"
    edgecolor: Color = "#00478f"
    linewidth: float = 0.5
    zorder: float = 20
    # draw mesh of Polygon
    # NOTE: This parameter is currently only valid for Collision Polygons created by the CommonRoad-Drivability-Checker
    # and has no effect for the Polygon class defined in commonroad-io.geometry.shape
    draw_mesh: bool = False


@dataclass
class VehicleSignalParams(BaseParam):
    signal_radius: float = 0.5
    indicator: ShapeParams = field(default_factory=lambda: ShapeParams(facecolor="#ebc200", zorder=20.2, linewidth=0.0))
    braking: ShapeParams = field(default_factory=lambda: ShapeParams(facecolor="red", zorder=20.2, linewidth=0.0))
    horn: ShapeParams = field(default_factory=lambda: ShapeParams(facecolor="red", zorder=20.1, linewidth=0.0))
    bluelight: ShapeParams = field(default_factory=lambda: ShapeParams(facecolor="blue", zorder=20.1, linewidth=0.0))


@dataclass
class ArrowParams(BaseParam):
    linewidth: float = 1.5
    edgecolor: Color = "black"
    facecolor: Color = "black"
    width: float = 0.8


@dataclass
class StateParams(BaseParam):
    draw_arrow: bool = False
    radius: float = 0.5
    scale_factor: float = 0.3
    linewidth: Optional[float] = None
    edgecolor: Optional[Color] = "black"
    facecolor: Optional[Color] = "black"
    zorder: float = 25
    arrow: ArrowParams = field(default_factory=ArrowParams)


@dataclass
class OccupancyParams(BaseParam):
    draw_occupancies: bool = True
    shape: ShapeParams = field(default_factory=lambda: ShapeParams(zorder=18, opacity=0.2))
    uncertain_position: ShapeParams = field(
            default_factory=lambda: ShapeParams(opacity=0.6, facecolor="k", edgecolor="r"))


@dataclass
class HistoryParams(BaseParam):
    """Draw the history of an object with fading colors."""
    draw_history: bool = False
    steps: int = 5
    step_size: int = 1
    fade_color: float = 0.1
    basecolor: Color = "#ffe119"
    occupancy: ShapeParams = field(
            default_factory=lambda: ShapeParams(opacity=0.2, edgecolor="k", linewidth=0.0, zorder=17))


@dataclass
class StaticObstacleParams(BaseParam):
    occupancy: OccupancyParams = field(
            default_factory=lambda: OccupancyParams(shape=ShapeParams(facecolor="#d95558", edgecolor="#831d20")))


@dataclass
class EnvironmentObstacleParams(BaseParam):
    occupancy: OccupancyParams = field(
            default_factory=lambda: OccupancyParams(shape=ShapeParams(facecolor="#0e6000", edgecolor="#831d20")))


@dataclass
class TrafficLightParams(BaseParam):
    draw_traffic_lights: bool = True
    #: Color of the red phase
    red_color: Color = "red"
    #: Color of the yellow phase
    yellow_color: Color = "#feb609"
    #: Color of the green phase
    green_color: Color = "#00aa16"
    #: Color of the red-yellow phase
    red_yellow_color: Color = "#fe4009ff"
    #: Show id of the traffic light
    show_label: bool = False
    scale_factor: float = 1.0
    zorder: float = 30


@dataclass
class TrafficSignParams(BaseParam):
    #: Enable drawing of traffic signs
    draw_traffic_signs: bool = False
    #: Limit traffic signs to a list of IDs, or None to include all IDs.
    show_traffic_signs: Optional[List[int]] = None
    #: Select mph or kmh as unit of the speed limit. Choose "auto" to determine based on the country.
    speed_limit_unit: str = "auto"
    #: Show id of the traffic sign
    show_label: bool = False
    scale_factor: float = 1.0
    zorder: float = 30


@dataclass
class IntersectionParams(BaseParam):
    draw_intersections: bool = False
    draw_incoming_lanelets: bool = True
    incoming_lanelets_color: Color = "#3ecbcf"
    draw_crossings: bool = True
    crossings_color: Color = "#b62a55"
    draw_successors: bool = True
    successors_left_color: Color = "#ff00ff"
    successors_straight_color: Color = "blue"
    successors_right_color: Color = "#ccff00"
    #: Show the ID of the intersection
    show_label: bool = False


@dataclass
class LaneletParams(BaseParam):
    left_bound_color: Color = "#555555"
    right_bound_color: Color = "#555555"
    center_bound_color: Color = "#dddddd"
    unique_colors: bool = False
    draw_stop_line: bool = True
    stop_line_color: Color = "#ffffff"
    draw_line_markings: bool = True
    draw_left_bound: bool = True
    draw_right_bound: bool = True
    draw_center_bound: bool = True
    draw_border_vertices: bool = False
    #: Show a triangle at the beginning of the lanelet, pointing in the driving direction.
    draw_start_and_direction: bool = True
    #: Apply a colormap to the center line of the lanelet, according the tangent angle.
    colormap_tangent: bool = False
    #: Show the ID of the lanelet.
    show_label: bool = False
    draw_linewidth: float = 0.5
    #: Fill the lanelet with a solid color.
    fill_lanelet: bool = True
    #: Fill color of the lanelet.
    facecolor: Color = "#c7c7c7"


@dataclass
class LaneletNetworkParams(BaseParam):
    kwargs_traffic_light_signs: Dict[str, Any] = field(default_factory=dict)
    #: Limit lanelets to a list of IDs, or None to include all IDs.
    draw_ids: Optional[List[int]] = None
    #: Options for lanelet drawing.
    lanelet: LaneletParams = field(default_factory=LaneletParams)
    #: Options for intersection drawing.
    intersection: IntersectionParams = field(default_factory=IntersectionParams)
    #: Offset angle for tangent colormap. See LaneletParams.colormap_tangent
    relative_angle: float = 0.0
    #: Options for traffic sign drawing.
    traffic_sign: TrafficSignParams = field(default_factory=TrafficSignParams)
    #: Options for traffic light drawing.
    traffic_light: TrafficLightParams = field(default_factory=TrafficLightParams)


@dataclass
class TrajectoryParams(BaseParam):
    draw_trajectory: bool = True
    facecolor: Color = "k"
    #: Draw trajectories as a continuous line instead of dots.
    draw_continuous: bool = False
    line_width: float = 0.17
    #: Use unique colors for each of the trajectories points.
    unique_colors: bool = False
    #: Parameters for shapes of uncertain position.
    shape: ShapeParams = field(default_factory=lambda: ShapeParams(facecolor="k", edgecolor="r", zorder=24))
    zorder: float = 24


@dataclass
class InitialStateParams(BaseParam):
    label_zorder: float = 35
    label: str = ""
    state: StateParams = field(
            default_factory=lambda: StateParams(draw_arrow=True, radius=1.0, scale_factor=0.5, facecolor="g", zorder=11,
                                                arrow=ArrowParams(facecolor="g", edgecolor="g")))


@dataclass
class PlanningProblemParams(BaseParam):
    initial_state: InitialStateParams = field(default_factory=InitialStateParams)
    goal_region: OccupancyParams = field(
        default_factory=lambda: OccupancyParams(shape=ShapeParams(facecolor="#f1b514", edgecolor="#302404", zorder=15)))
    #: Goal lanelet parameters
    lanelet: LaneletParams = field(default_factory=LaneletParams)


@dataclass
class PlanningProblemSetParams(BaseParam):
    #: Limit planning problems to a list of IDs, or None to include all IDs.
    draw_ids: Optional[List[int]] = None
    planning_problem: PlanningProblemParams = field(default_factory=PlanningProblemParams)


@dataclass
class VehicleShapeParams(BaseParam):
    #: Options for visualizing the direction indicator of the vehicle shape.
    direction: ShapeParams = field(default_factory=lambda: ShapeParams(facecolor="#00478f"))
    #: Options for visualizing the vehicle's occupancy in the current time step.
    occupancy: OccupancyParams = field(
            default_factory=lambda: OccupancyParams(shape=ShapeParams(opacity=1.0, zorder=20)))


@dataclass
class DynamicObstacleParams(BaseParam):
    draw_shape: bool = True
    #: Draw a type-dependent icon (if available) instead of the primitive geometric shape.
    draw_icon: bool = False
    #: Draw the direction indicator of the dynamic obstacle.
    draw_direction: bool = False
    draw_bounding_box: bool = True
    #: Show the ID of the dynamic obstacle.
    show_label: bool = False
    #: Visualize the dynamic obstacle signals like indicator or braking lights.
    draw_signals: bool = True
    #: Draw the initial state of the dynamic obstacle.
    draw_initial_state: bool = False
    #: Options for visualizing the vehicle in the current time step.
    vehicle_shape: VehicleShapeParams = field(default_factory=VehicleShapeParams)
    #: Options for visualizing the vehicle signals.
    signals: VehicleSignalParams = field(default_factory=VehicleSignalParams)
    #: Options for visualizing the vehicle states within [time_begin, time_end].
    state: StateParams = field(default_factory=StateParams)
    #: Options for visualizing the vehicle occupancy history with fading colors.
    history: HistoryParams = field(default_factory=HistoryParams)
    #: Options for visualizing the vehicle occupancy in future time steps.
    occupancy: OccupancyParams = field(default_factory=OccupancyParams)
    #: Options for visualizing the vehicle trajectory in future time steps.
    trajectory: TrajectoryParams = field(default_factory=TrajectoryParams)


@dataclass
class PhantomObstacleParams(BaseParam):
    draw_shape: bool = True
    draw_icon: bool = False
    draw_direction: bool = False
    draw_bounding_box: bool = True
    show_label: bool = False
    zorder: float = 20
    draw_signals: bool = True
    vehicle_shape: VehicleShapeParams = field(
        default_factory=lambda: VehicleShapeParams(occupancy=OccupancyParams(shape=ShapeParams(facecolor="#e59dff"))))
    signals: VehicleSignalParams = field(default_factory=VehicleSignalParams)
    draw_initial_state: bool = False
    state: StateParams = field(default_factory=StateParams)
    history: HistoryParams = field(default_factory=HistoryParams)
    occupancy: OccupancyParams = field(default_factory=lambda: OccupancyParams(shape=ShapeParams(facecolor="#c641f6")))
    trajectory: TrajectoryParams = field(default_factory=TrajectoryParams)


@dataclass
class MPDrawParams(BaseParam):
    #: Enable axes for matplotlib.
    axis_visible: bool = True
    shape: ShapeParams = field(default_factory=ShapeParams)
    dynamic_obstacle: DynamicObstacleParams = field(default_factory=DynamicObstacleParams)
    static_obstacle: StaticObstacleParams = field(default_factory=StaticObstacleParams)
    phantom_obstacle: PhantomObstacleParams = field(default_factory=PhantomObstacleParams)
    environment_obstacle: EnvironmentObstacleParams = field(default_factory=EnvironmentObstacleParams)
    trajectory: TrajectoryParams = field(default_factory=TrajectoryParams)
    lanelet_network: LaneletNetworkParams = field(default_factory=LaneletNetworkParams)
    traffic_light: TrafficLightParams = field(default_factory=TrafficLightParams)
    traffic_sign: TrafficSignParams = field(default_factory=TrafficSignParams)
    occupancy: OccupancyParams = field(default_factory=OccupancyParams)
    state: StateParams = field(default_factory=StateParams)
    planning_problem: PlanningProblemParams = field(default_factory=PlanningProblemParams)
    planning_problem_set: PlanningProblemSetParams = field(default_factory=PlanningProblemSetParams)
    initial_state: InitialStateParams = field(default_factory=InitialStateParams)
    goal_region: OccupancyParams = field(default_factory=OccupancyParams)


T = TypeVar("T")
OptionalSpecificOrAllDrawParams = Optional[Union[T, MPDrawParams]]
