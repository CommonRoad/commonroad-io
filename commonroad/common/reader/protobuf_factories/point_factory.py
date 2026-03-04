import numpy as np

from commonroad.scenario_definition.protobuf_format.generated_scripts import util_pb2


class PointFactory:
    @classmethod
    def create_from_message(cls, point_msg: util_pb2.Point) -> np.ndarray:
        return np.array([point_msg.x, point_msg.y])
