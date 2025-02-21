import math

import numpy as np
from matplotlib import pyplot as plt

from commonroad.geometry.shape import Rectangle, SemiTrailerTruck
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import InitialState, KSTState
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.draw_params import DynamicObstacleParams
from commonroad.visualization.mp_renderer import MPRenderer

fig = plt.figure()
rnd = MPRenderer()

shape = SemiTrailerTruck.create_default()
# You can also use a normal rectangle as vehicle shape:
shape = Rectangle(width=2.0, length=10.0)

initial_state = InitialState(position=np.array([0, 0]), orientation=0, velocity=0, time_step=0)
ego_veh = DynamicObstacle(0, ObstacleType.TRUCK, shape, initial_state)
state_0 = KSTState(
    time_step=0,
    position=initial_state.position,
    steering_angle=0,
    velocity=initial_state.velocity,
    orientation=initial_state.orientation,
    hitch_angle=0,
)
state_1 = KSTState(
    time_step=1,
    position=initial_state.position,
    steering_angle=0,
    velocity=initial_state.velocity,
    orientation=-math.pi / 10,
    hitch_angle=math.pi / 5,
)
ego_veh.prediction = TrajectoryPrediction(
    trajectory=Trajectory(initial_time_step=0, state_list=[state_0, state_1]), shape=shape
)

ego_params = DynamicObstacleParams()
ego_params.time_begin = 1
ego_params.time_end = 1
ego_params.draw_icon = True
ego_params.trajectory.draw_trajectory = False
ego_params.vehicle_shape.occupancy.shape.facecolor = "orange"
ego_params.vehicle_shape.occupancy.shape.zorder = 105
ego_veh.draw(rnd, draw_params=ego_params)

rnd.render()
plt.axis("equal")
# rnd.ax.axis('off')

plt.show()

# plt.savefig("scenario.svg")
