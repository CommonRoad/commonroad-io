# import functions to read xml file and visualize commonroad objects
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from visual3d import *
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import random
import sys
from threading import Thread
import time
import os
import sys
import select

# generate path of the file to be opened
#file_path = "USA_Lanker-1_1_T-1.xml"
# file_path = "ZAM_Tutorial-1_1_T-1.xml"
file_path = "ZAM_Tutorial-1_2_T-1.xml"
#file_path="BEL_Beringen-3_5_I-1-1.cr.xml"
#file_path="DEU_Gar-2_1_T-1.xml"

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

v = visual(scenario, planning_problem_set)
v.init_show()
#v.show()




i = 0


while True:
    input = select.select([sys.stdin], [], [], 0.1)[0]
    if input:
        value = sys.stdin.readline().rstrip()

        if (value == "f"):
            i += 1
        elif (value == "p"):
            if i != 0:
                i -= 1
        elif (value == "l"):
            v.switch_ligth()
        elif (value == "i"):
            v.zoom_in()
        elif (value == "o"):
            v.zoom_out()
        elif (value == "z"):
            v.drone.go()
        elif (value == "s"):
            v.drone.returne()
        elif (value == "q"):
            v.drone.left()
        elif (value == "d"):
            v.drone.right()
        elif (value == "+"):
            v.drone.rotate_plus()
        elif (value == "-"):
            v.drone.rotate_minus()
        elif (value == "t"):
            v.drone.follow += 1
        elif (value == "*"):
            v.drone.up()
        elif (value == "/"):
            v.drone.down()
        elif (value == "w"):
            v.drone.go_powerline()
            for j in range(len(v.drone.path)):
                for i in range(len(v.drone.position)):
                    v.drone.position[i] = v.drone.path[-j][i]
                v.auto_set_lim()
                v.show_at_time(j)
                v.drone.new_drone()
    else:
        v.auto_set_lim()
        v.show_at_time(i)
        v.drone.new_drone()



"""
j=0
while True:

    j+=1
    if j<len(v.drone.path):
        


"""
"""

# plot the planning problem and the scenario for the fifth time step
plt.figure(figsize=(25, 10))
rnd = MPRenderer()
scenario.draw(rnd, draw_params={'time_begin': 9})
planning_problem_set.draw(rnd)
rnd.render()
plt.show()
"""

