from abc import ABC, abstractmethod
from typing import Union, Dict, Tuple

from commonroad.visualization.param_server import ParamServer


class IDrawable(ABC):
    @abstractmethod
    def draw(self, renderer, draw_params: Union[ParamServer, Dict[str, ...]],
             call_stack: Tuple[str, ...]):
        pass
