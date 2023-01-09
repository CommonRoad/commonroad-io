import os
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.draw_params import DynamicObstacleParams
filename = os.getcwd() + '/../../../tests/test_scenarios/USA_US101-4_1_T-1.xml'
scenario, planning_problem_set = CommonRoadFileReader(filename).open()
rnd = MPRenderer(figsize=(8,4.5))
rnd.draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "yellow"
draw_params = DynamicObstacleParams()
draw_params.vehicle_shape.occupancy.shape.facecolor = "green"
scenario.draw(rnd)
scenario.dynamic_obstacles[0].draw(rnd, draw_params)
planning_problem_set.draw(rnd)
rnd.render()