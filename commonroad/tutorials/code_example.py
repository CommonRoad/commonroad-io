# import functions to read xml file and visualize commonroad objects
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from visual3d import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

# generate path of the file to be opened
#file_path = "USA_Lanker-1_1_T-1.xml"
#file_path = "ZAM_Tutorial-1_1_T-1.xml"
#file_path="ZAM_Tutorial-1_2_T-1.xml"
#file_path="BEL_Beringen-3_5_I-1-1.cr.xml"


# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

v=visual(scenario)
v.show()

print(planning_problem_set.planning_problem_dict[1].initial_state.__getattribute__("position"))

"""

print (scenario.environment_obstacle)

# plot the planning problem and the scenario for the fifth time step
plt.figure(figsize=(25, 10))
rnd = MPRenderer()
scenario.draw(rnd, draw_params={'time_begin': 9})
planning_problem_set.draw(rnd)
rnd.render()
plt.show()
"""
