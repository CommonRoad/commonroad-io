from abc import ABC, abstractmethod
from typing import Union, Dict, Tuple, Optional

from commonroad.visualization.param_server import ParamServer


class IDrawable(ABC):
    @abstractmethod
    def draw(self, renderer, draw_params: Union[ParamServer, dict, None],
             call_stack: Optional[Tuple[str, ...]]):
        pass
