import unittest

from commonroad.prediction.prediction_interface import PredictorInterface
from commonroad.scenario.scenario import Scenario


class TestPredictor(PredictorInterface):
    def predict(self, sc: Scenario, initial_time_step: int = 0) -> Scenario:
        return Scenario(dt=initial_time_step * 0.1)


class TestMotionPlannerInterface(unittest.TestCase):
    def test_prediction(self):
        predictor = TestPredictor()
        prediction = predictor.predict(Scenario(0.1), initial_time_step=31)

        self.assertEqual(prediction.dt, 3.1)
