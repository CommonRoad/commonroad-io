# import functions to read xml file and visualize commonroad objects
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from obstacles import *
from commonroad.visualization.mp_renderer import MPRenderer

import matplotlib.pyplot as plt
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from lanelet import *
from car_time import car_time
from graphe import Graphe


file_path="ZAM_Tjunction-1_129_T-1.xml"
#file_path="DEU_Flensburg-6_1_T-1.xml"#don't work
# generate path of the file to be opened
#file_path = "USA_Lanker-1_1_T-1.xml"
#file_path="ARG_Carcarana-10_4_T-1.xml"
#file_path = "ZAM_Tutorial-1_1_T-1.xml"
#file_path = "ZAM_Tutorial-1_2_T-1.xml"
#file_path="BEL_Beringen-3_5_I-1-1.cr.xml"
#file_path="DEU_Gar-2_1_T-1.xml"
#file_path="ARG_Carcarana-10_2_T-1.xml"
file_path="ZAM_ahhh-1_1_T-1.xml"
#file_path="ZAM_d-1_1_T-1.xml"
file_path="ZAM_finalscenariobridge-1_1_T-1.xml"
i=0

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
scenario.lanelet_network.lanelets[0].polygon.vertices
car_time=car_time(scenario).car_time
"""
Graphe(scenario)


# plot the planning problem and the scenario for the fifth time step
plt.figure(figsize=(25, 10))
rnd = MPRenderer()
scenario.draw(rnd, draw_params={'time_begin': 5})
planning_problem_set.draw(rnd)
rnd.render()
plt.show()

"""

fig = plt.figure(figsize=plt.figaspect(1), constrained_layout=False)
ax = fig.gca(projection='3d')


lan=lanelet(ax,scenario)
o=obstacles(ax,scenario,lan.information,car_time)

ax.set_xlim([0, 800])
ax.set_ylim([-400, 400])
ax.set_zlim([-2, 20])

if file_path=="/home/thomas/Downloads/ZAM_ahhh-1_1_T-1.xml":
    ax.set_xlim([0, 100])
    ax.set_ylim([-50, 50])
    ax.set_zlim([-10, 10])
    

ax.view_init(elev=40, azim=150)
plt.axis('on')
plt.savefig("test.pdf")

#o.show()


plt.show()
