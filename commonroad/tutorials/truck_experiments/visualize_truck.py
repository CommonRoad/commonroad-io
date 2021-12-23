from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from IPython import display
import numpy as np
import os

# import functions to read xml file and visualize commonroad objects
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

# generate path of the file to be opened
file_path = "input/ZAM_Loading_Bay-1_1_T-1.xml"

# TODO: visualize video with the actual truck size, not just wheelbase
#       -> do we want to leave it like this, or do it like in the paper?
#       -> like in the paper
# TODO: check if collision checker works for truck - obstacle_collision in solution_checker.py
# TODO: add wheelbase parameters to the truck, not initial state
# TODO: check why feasibility checker fails
# TODO: check classes and whether they make sense at this point
# TODO: create_collision_checker (tutorial 2) - additional param for continuous/discrete collision checking
#       -> discrete: each time step - default
#       -> continuous: use bounding box around pos in t1 and t2 to check for collisions between the two states
#       -> trajectory_queries.py - trajectory_preprocess_obb_sum - add this for continuous case
# TODO: collision checker object be more dynamic (can change the scenario)
#       -> remove/modify collision object - C++

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
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