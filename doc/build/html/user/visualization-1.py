import os
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
filename = os.getcwd() + '/../../../tests/test_scenarios/USA_US101-4_1_T-1.xml'
scenario, planning_problem_set = CommonRoadFileReader(filename).open()
rnd = MPRenderer(figsize=(8,4.5))
scenario.draw(rnd)
planning_problem_set.draw(rnd)
rnd.render()