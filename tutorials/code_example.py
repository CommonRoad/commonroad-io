# import functions to read xml file and visualize commonroad objects
import numpy as np
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.geometry.shape import Rectangle, Circle, Polygon, ShapeGroup
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.trajectory import State
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.trajectory import Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction

# generate path of the file to be opened
file_path = "ZAM_Tutorial-1_1_T-1.xml"

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

# generate the static obstacle according to the specification, refer to API for details of input parameters
static_obstacle_id = scenario.generate_object_id()
static_obstacle_type = ObstacleType.PARKED_VEHICLE
static_obstacle_shape = Rectangle(width=2.0, length=4.5)
static_obstacle_initial_state = State(position=np.array([30.0, 3.5]), orientation=0.02, time_step=0)

static_obstacle = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape,
                                 static_obstacle_initial_state)

scenario.add_objects(static_obstacle)

# initial state has a time step of 0
dynamic_obstacle_initial_state = State(position=np.array([50.0, 0.0]), velocity=22, orientation=0.02, time_step=0)

# generate the states for the obstacle for time steps 1 to 40 by assuming constant velocity
state_list = []
for i in range(1, 100):
    # compute new position
    new_position = np.array([dynamic_obstacle_initial_state.position[0] + scenario.dt * i * 22, scenario.dt * i])
    # create new state
    new_state = State(position=new_position, velocity=22, orientation=1 / 22, time_step=i, hitch=-1 / 44)
    # add new state to state_list
    state_list.append(new_state)

# create the trajectory of the obstacle, starting at time step 1
dynamic_obstacle_trajectory = Trajectory(1, state_list)

# create the prediction using the trajectory and the shape of the obstacle
dynamic_obstacle_shape = ShapeGroup([Rectangle(5.1, 2.55), Rectangle(13.6, 2.55)])
dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, dynamic_obstacle_shape,
                                                   wheelbase_lengths=[3.6, 8.1])

# generate the dynamic obstacle according to the specification
dynamic_obstacle_id = scenario.generate_object_id()
dynamic_obstacle_type = ObstacleType.TRUCK
dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id, dynamic_obstacle_type, dynamic_obstacle_shape,
                                   dynamic_obstacle_initial_state, dynamic_obstacle_prediction)

# add dynamic obstacle to the scenario
scenario.add_objects(dynamic_obstacle)

for i in range(5):
    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    scenario.draw(rnd, draw_params={'time_begin': i})
    planning_problem_set.draw(rnd)
    rnd.render()
    plt.show()

fig = plt.figure(figsize=(25, 10))
