import numpy as np

from commonroad.scenario_definition.protobuf_format.generated_scripts import util_pb2


class PointMessage:
    @classmethod
    def create_message(cls, point: np.ndarray) -> util_pb2.Point:
        point_msg = util_pb2.Point()

        point_msg.x = point[0]
        point_msg.y = point[1]

        return point_msg
