import dataclasses
from dataclasses import dataclass, field
from typing import Any


@dataclass
class BaseParam:

    def __setattr__(self, name: str, value: Any) -> None:
        if name in {f.name for f in dataclasses.fields(self)}:
            super().__setattr__(name, value)
        else:
            for k, v in self.__dict__.items():
                if isinstance(v, BaseParam):
                    v.__setattr__(name, value)

    def __getitem__(self, item):
        # Todo: Tuple style lookup
        try:
            value = self.__getattribute__(item)
        except AttributeError:
            raise KeyError()
        return value

    def __setitem__(self, key, value):
        try:
            self.__setattr__(key, value)
        except AttributeError:
            raise KeyError()


@dataclass
class ShapeParams(BaseParam):
    """Opacity doc string"""
    opacity: float = 1.0
    facecolor: str = "#1d7eea"
    edgecolor: str = "#00478f"
    linewidth: float = 0.5
    zorder: int = 20


@dataclass
class ShapesParams(BaseParam):
    rectangle: ShapeParams = field(default_factory=ShapeParams)
    circle: ShapeParams = field(default_factory=ShapeParams)
    polygon: ShapeParams = field(default_factory=ShapeParams)


@dataclass
class DynamicObstacleParams(BaseParam):
    draw_shape: bool = True
    draw_icon: bool = False
    draw_direction: bool = False
    draw_bounding_box: bool = True
    show_label: bool = False
    zorder: int = 20
    draw_signals: bool = True
    shapes: ShapesParams = field(default_factory=ShapesParams)


@dataclass
class MPDrawParams(BaseParam):
    time_begin: int = 0
    time_end: int = 200
    axis_visible: bool = True
    focus_obstacle_id: bool = False
    relative_angle: float = 0.0
    antialiased: bool = True
    dynamic_obstacle: DynamicObstacleParams = field(default_factory=DynamicObstacleParams)


def dict_to_params(dict_params: dict, cls=MPDrawParams):
    fields = dataclasses.fields(cls)
    cls_map = {f.name: f.type for f in fields}
    kwargs = {k: v if not isinstance(v, dict) else dict_to_params(v, cls_map[k]) for k, v in dict_params.items()}
    return cls(**kwargs)


if __name__ == '__main__':
    # Easy instantiation with default params
    p = MPDrawParams()
    # Or custom
    p = MPDrawParams(time_begin=10)
    # Type hints
    p = MPDrawParams(time_begin="test")
    # Manipulate values with IDE context support
    p.dynamic_obstacle.shapes.circle.opacity = 0
    # Dictionary style value setting
    p["time_begin"] = 5
    # Dictionary style lookup
    assert p["time_begin"] == 5
    # General setting of parameters for all sub-params
    p.facecolor = "red"
    assert p.dynamic_obstacle.shapes.circle.facecolor == "red"
    assert p.dynamic_obstacle.shapes.rectangle.facecolor == "red"

    # Create nested dicts from params objects
    dictionary = dataclasses.asdict(p)
    # Or param objects from dictionaries (backward compatibility)
    p2 = dict_to_params({"antialiased": False, "dynamic_obstacle": {"draw_shape": False}})

    # Currently not so nice: Instantiation with custom parameters
    p = MPDrawParams(dynamic_obstacle=DynamicObstacleParams(shapes=ShapesParams(circle=ShapeParams(edgecolor="blue"))))
