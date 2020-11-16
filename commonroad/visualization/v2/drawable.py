from abc import ABC, abstractmethod
from typing import Union, Tuple, Optional

from commonroad.visualization.v2.param_server import ParamServer


class IDrawable(ABC):
    @abstractmethod
    def draw(self, renderer, draw_params: Union[ParamServer, dict, None],
             call_stack: Optional[Tuple[str, ...]]):
        pass
