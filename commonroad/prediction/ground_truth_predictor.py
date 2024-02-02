from commonroad.prediction.prediction_interface import PredictorInterface
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory


class GroundTruthPredictor(PredictorInterface):
    """Applies prediction stored in scenario."""

    def predict(self, sc: Scenario, initial_time_step: int = 0) -> Scenario:
        """Applies ground truth prediction.

        :param sc: CommonRoad scenario.
        :param initial_time_step: Time step at which prediction should start.
        :return: CommonRoad scenario containing prediction.
        """
        for obstacle in sc.dynamic_obstacles:
            state_list = [
                state for state in obstacle.prediction.trajectory.state_list if state.time_step >= initial_time_step
            ]
            traj = Trajectory(state_list[0].time_step, state_list)
            obstacle.prediction.trajectory = traj

        return sc
