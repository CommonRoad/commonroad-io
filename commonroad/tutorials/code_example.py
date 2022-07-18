# import functions to read xml file and visualize commonroad objects
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from visual3d import *
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario import scenario
from commonroad.scenario.traffic_sign import TrafficSign, TrafficLight
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
from traffic_lights import trafic_light

# generate path of the file to be opened
#file_path = "USA_Lanker-1_1_T-1.xml"#big
#file_path="ARG_Carcarana-10_4_T-1.xml"
#file_path = "ZAM_Tutorial-1_1_T-1.xml"
file_path = "ZAM_Tutorial-1_2_T-1.xml"#tuto
#file_path="BEL_Beringen-3_5_I-1-1.cr.xml"
#file_path="DEU_Gar-2_1_T-1.xml"
#file_path="ARG_Carcarana-10_2_T-1.xml"
#file_path="/home/thomas/Downloads/RUS_Bicycle-5_1_T-1.xml"#bike
file_path="/home/thomas/Downloads/USA_Lanker-1_3_T-1.xml"#trafic light
#file_path="/home/thomas/Downloads/ZAM_Urban-4_1_S-1.xml"#PEDESTRIAN


# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()





v = visual(scenario)
v.init_show()

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
        elif (value == "a"):
            v.show()
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
        elif (value == "1"):
            v.drone.elev+=10
        elif (value == "0"):
            v.drone.elev-=10
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
        for traffic in v.list_traffic_lights:
            traffic.new_light()

        
        v.drone.new_drone()




"""


# plot the planning problem and the scenario for the fifth time step

plt.figure(figsize=(25, 10))
rnd = MPRenderer()
scenario.draw(rnd, draw_params={'time_begin': 0})
planning_problem_set.draw(rnd)
rnd.render()
plt.ion()
plt.show()

for i in range (500):

    rnd = MPRenderer()
    scenario.draw(rnd, draw_params={'time_begin': i})
    planning_problem_set.draw(rnd)
    rnd.render()
    plt.pause(1)

"""

