from abc import ABC

from commonroad.scenario.scenario import Scenario


class PredictorInterface(ABC):
    """Base class for prediction."""

    def predict(self, sc: Scenario, initial_time_step: int = 0) -> Scenario:
        """
        Abstract method for performing predictions.

        :param sc: Scenario containing no predictions for obstacles.
        :param initial_time_step: Time step to start prediction.
        :return: CommonRoad scenario containing predictions.
        """
