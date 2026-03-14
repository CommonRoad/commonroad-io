import numpy as np

from commonroad.common.protobuf.common import util_pb2


class PointMessage:
    @classmethod
    def create_message(cls, point: np.ndarray) -> util_pb2.Point:
        point_msg = util_pb2.Point()

        point_msg.x = point[0]
        point_msg.y = point[1]

        return point_msg
