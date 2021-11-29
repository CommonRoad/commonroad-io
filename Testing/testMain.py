import os
import matplotlib.pyplot as plt
from IPython import display

import numpy as np

# import functions to read xml file and visualize commonroad objects
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route

# generate path of the file to be opened
file_path = "sixthScenario.xml"

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

"""
planning_problem = planning_problem_set.planning_problem_dict[123]
route_planner = RoutePlanner(scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)
candidate_holder = route_planner.plan_routes()
route = candidate_holder.retrieve_best_route_by_orientation()
reference_path = route.reference_path
planning_problem_set.extend_planning_problem_with_reference_path(123, reference_path)

reference_path_list = reference_path.tolist()
open("filename", "w").close()
f = open("refPath.txt", "w")
f.write("<referencePath>\n")
for line in reference_path_list:
    f.write("\t<position>\n")
    f.write("\t\t<x>" + str(line[0]) + "</x>\n")
    f.write("\t\t<y>" + str(line[1]) + "</y>\n")
    f.write("\t</position>\n")
f.write("</referencePath>")
f.close()

ref_path = planning_problem.reference_path.tolist()
list_interval_reference_path = []
for coords in range(len(ref_path) - 1):
    diff = tuple()
    diff = abs(abs(ref_path[coords+1][0]) - abs(ref_path[coords][0])),\
           abs(abs(ref_path[coords+1][1]) - abs(ref_path[coords][1]))
    list_interval_reference_path.append(diff)
ndArrInterval = np.array(list_interval_reference_path)
someBreakPoint = True

planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
#dictProblemSet = extended_planning_problem_set.extended_planning_problem_dict
dictProblemSet = extended_planning_problem_set.planning_problem_dict()
ndArrayCoordinates = list(dictProblemSet.values())[0].reference_path
#listCoordinates = ndarrayCoordinates.toList()
#plt.plot(ndArrayCoordinates, zorder=200)

route_planner = RoutePlanner(scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)

# plan routes, and save the routes in a route candidate holder
candidate_holder = route_planner.plan_routes()

# option 1: retrieve all routes
list_routes, num_route_candidates = candidate_holder.retrieve_all_routes()
print(f"Number of route candidates: {num_route_candidates}")
# here we retrieve the first route in the list, this is equivalent to: route = list_routes[0]
route = candidate_holder.retrieve_best_route_by_orientation()
# option 2: retrieve the best route by orientation metric
# route = candidate_holder.retrieve_best_route_by_orientation()

# print coordinates of the vertices of the reference path
print("\nCoordinates [x, y]:")
print(route.reference_path)

visualize_route(route, draw_route_lanelets=True, draw_reference_path=False, size_x=15)

visualize_route(route, draw_route_lanelets=True, draw_reference_path=True, size_x=15)
"""
# plot the scenario for 40 time step, here each time step corresponds to 0.1 second
for i in range(0, 40):
    plt.figure(figsize=(40, 25))
    rnd = MPRenderer()
    # plot the scenario at different time step
    scenario.draw(rnd, draw_params={'time_begin': i})
    # plot the planning problem set
    planning_problem_set.draw(rnd)
    rnd.render()
    #plt.plot(ndArrayCoordinates, zorder=200)

#plt.plot([420,500],[-400,-250], zorder=200)