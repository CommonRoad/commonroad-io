from abc import ABC, abstractmethod
from typing import Optional

from commonroad.visualization.renderer import IRenderer
from commonroad.visualization.draw_params import BaseParam


class IDrawable(ABC):
    """
    Interface for drawable types
    """

    @abstractmethod
    def draw(self, renderer: IRenderer, draw_params: Optional[BaseParam]) -> None:
        """
        Draw the object

        :param renderer: Renderer to use for drawing
        :param draw_params: optional parameters for plotting, overriding the parameters of the renderer
        :return: None
        """
        pass
