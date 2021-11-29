#%matplotlib inline
#%load_ext autoreload
#%autoreload 2

import os
import sys

# add the root folder to python path
import route as route

path_notebook = os.getcwd()
sys.path.append(os.path.join(path_notebook, "../"))

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad-route-planner.commonroad_route_planner.route_planner import RoutePlanner
from commonroad-route-planner.commonroad_route_planner.utils_visualization import visualize_route

# load scenario
path_scenario = os.path.join(path_notebook, "../scenarios/")
id_scenario = 'USA_Peach-2_1_T-1'

# read in scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(path_scenario + id_scenario + '.xml').open()
# retrieve the first planning problem in the problem set
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

# plot the scenario and the planning problem set
renderer = MPRenderer(figsize=(12, 12))

scenario.draw(renderer)
planning_problem.draw(renderer)

renderer.render()
plt.margins(0, 0)

route_planner = RoutePlanner(scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)

# plan routes, and save the routes in a route candidate holder
candidate_holder = route_planner.plan_routes()

# option 1: retrieve all routes
list_routes, num_route_candidates = candidate_holder.retrieve_all_routes()
print(f"Number of route candidates: {num_route_candidates}")
# here we retrieve the first route in the list, this is equivalent to: route = list_routes[0]
route = candidate_holder.retrieve_first_route()

# option 2: retrieve the best route by orientation metric
# route = candidate_holder.retrieve_best_route_by_orientation()

# print coordinates of the vertices of the reference path
print("\nCoordinates [x, y]:")
print(route.reference_path)

visualize_route(route, draw_route_lanelets=True, draw_reference_path=False, size_x=6)