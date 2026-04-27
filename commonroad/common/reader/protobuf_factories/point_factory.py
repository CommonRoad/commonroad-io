import numpy as np

from commonroad.common.protobuf.common import util_pb2


class PointFactory:
    @classmethod
    def create_from_message(cls, point_msg: util_pb2.Point) -> np.ndarray:
        return np.array([point_msg.x, point_msg.y])
