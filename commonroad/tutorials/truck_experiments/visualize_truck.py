from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from IPython import display
import numpy as np
import os

import commonroad_dc.pycrcc as pycrcc
# import functions to read xml file and visualize commonroad objects
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader, Solution
from commonroad.scenario.scenario import ScenarioID
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_dc.feasibility.solution_checker import obstacle_collision
from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_object, \
    create_collision_checker
from commonroad_dc.collision.trajectory_queries.trajectory_queries import trajectories_collision_static_obstacles, \
    trajectories_collision_dynamic_obstacles

# generate path of the file to be opened
file_path = "input/ZAM_Loading_Bay-1_1_T-1.xml"
file_path_without_truck = "input/ZAM_Loading_Bay-1_1_T-no-truck.xml"

# TODO: check why feasibility checker fails
#       -> because the y coordinate difference is way too big already for the first step
#       -> plot values (how big is the difference?) - save the differences/values in an array
# TODO: check classes and whether they make sense at this point
# TODO: create_collision_checker (tutorial 2) - additional param for continuous/discrete collision checking
#       -> discrete: each time step - default
#       -> continuous: use bounding box around pos in t1 and t2 to check for collisions between the two states
#       -> trajectory_queries.py - trajectory_preprocess_obb_sum - add this for continuous case
#       -> added it in create_collision_checker_scenario, what happens if preprocessing fails?
#           collide_fcl returns true if the object is null, but doesn't return timestamp
#       -> test: try with/without and check if with, we have 1 timestamp less
# TODO: collision checker object be more dynamic (can change the scenario)
#       -> remove/modify collision object - C++

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path_without_truck).open()
collision_checker = create_collision_checker(scenario)

tvo_static = pycrcc.TimeVariantCollisionObject(1)
tvo_dynamic = pycrcc.TimeVariantCollisionObject(1)

static_obstacles = pycrcc.ShapeGroup()
for s in scenario.static_obstacles:
    collision_object = create_collision_object(s)
    static_obstacles.add_shape(collision_object)
    tvo_static.append_obstacle(collision_object)

scenario_truck, planning_problem_set_truck = CommonRoadFileReader(file_path).open()

for do in scenario_truck.dynamic_obstacles:
    collision_object = create_collision_object(do.prediction)
    tvo_dynamic.append_obstacle(collision_object)
    if collision_checker.collide(collision_object):
        print("There was a collision for dynamic obstacle with id " + str(do.obstacle_id))
    x = trajectories_collision_static_obstacles([collision_object], static_obstacles=static_obstacles)
    y = trajectories_collision_dynamic_obstacles([collision_object], [collision_object])
fig = plt.figure(figsize=(25, 10))

def animation_frame(i):
    # zoom in the animation (plot_limits can be removed if not needed)
    rnd = MPRenderer(plot_limits=[0, 200, 400, 600], figsize=(25, 10))
    # plot the scenario at different time step
    scenario.draw(rnd, draw_params={'time_begin': i, 'time_end': 1573})
    # plot the planning problem set
    planning_problem_set.draw(rnd)
    return rnd.render()

# animate
animation = FuncAnimation(fig, func=animation_frame, frames=range(0, 1573, 5), interval=1)
plt.show()

# Check if both objects collide
print('Collision between time-varying obstacle tvo_1 and tvo_2: ', tvo_static.collide(tvo_dynamic))

rnd = MPRenderer(figsize=(10, 10))
tvo_static.draw(rnd, draw_params={'time_variant_obstacle': {'collision': {'polygon': {'draw_mesh': False, 'facecolor': 'red'}}}})
tvo_dynamic.draw(rnd, draw_params={'facecolor': 'green'})
rnd.render(show=True)
plt.show()