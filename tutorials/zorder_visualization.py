# load the CommonRoad scenario that has been created in the CommonRoad tutorial
import os

from matplotlib import pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer

plt.rcParams["figure.max_open_warning"] = 50

file_path = os.path.join(os.getcwd(), "ZAM_Tutorial-1_2_T-1.xml")

scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

draw_params = MPDrawParams()

for i in range(5, 6):
    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    rnd.draw_params.dynamic_obstacle.zorder = 20
    rnd.draw_params.dynamic_obstacle.opacity = 0.3
    rnd.draw_params.time_begin = i
    scenario.draw(rnd)
    planning_problem_set.draw(rnd)
    rnd.render()
    plt.show()
