#import os
#import matplotlib.pyplot as plt
#from commonroad.common.file_reader import CommonRoadFileReader
#from commonroad.visualization.draw_dispatch_cr import draw_object
#filename = os.getcwd() + '/../../../../../../scenarios/hand-crafted/DEU_Muc-1_1_T-1.xml'
#scenario, planning_problem_set = CommonRoadFileReader(filename).open()

#plt.style.use('classic')
#inch_in_cm = 2.54
#figsize = [20, 8]
#plot_limits = [-80, 80, -60, 30]
#plt.figure(figsize=(8,4.5))
#plt.gca().axis('equal')

#draw_object(scenario, plot_limits=plot_limits)
#draw_object(planning_problem_set, plot_limits=plot_limits)
#plt.tight_layout()
#plt.show()

import os
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
filename = os.getcwd() + '/../../../../../../../scenarios/NGSIM/US101/USA_US101-2_1_T-1.xml'
scenario, planning_problem_set = CommonRoadFileReader(filename).open()

plt.style.use('classic')
inch_in_cm = 2.54
figsize = [20, 8]
plot_limits = [-30, 120, -140, 20]
plt.figure(figsize=(8,4.5))
plt.gca().axis('equal')

draw_object(scenario, draw_params={'time_end':20},plot_limits=plot_limits)
draw_object(planning_problem_set, plot_limits=plot_limits)
plt.tight_layout()
plt.show()