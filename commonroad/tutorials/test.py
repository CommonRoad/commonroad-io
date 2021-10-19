import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython import display
import numpy as np

# import functions to read xml file and visualize commonroad objects
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

# generate path of the file to be opened
file_path = "ZAM_Tutorial_truck-1_2_T-3.xml"

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
fig = plt.figure(figsize=(25, 10))

def animation_frame(i):
    rnd = MPRenderer()
    # plot the scenario at different time step
    scenario.draw(rnd, draw_params={'time_begin': i})
    # plot the planning problem set
    planning_problem_set.draw(rnd)
    return rnd.render()

animation = FuncAnimation(fig, func=animation_frame, frames=range(0, 80, 1), interval=80)
plt.show()

# # plot the scenario for 40 time step, here each time step corresponds to 0.1 second
# for i in range(0, 40):
#     plt.figure(figsize=(25, 10))
#     rnd = MPRenderer()
#     # plot the scenario at different time step
#     scenario.draw(rnd, draw_params={'time_begin': i})
#     # plot the planning problem set
#     planning_problem_set.draw(rnd)
#     rnd.render()