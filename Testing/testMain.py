import os
import matplotlib.pyplot as plt
from IPython import display

import numpy as np

import math
from typing import List

# import functions to read xml file and visualize commonroad objects
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_route_planner.route_planner import RoutePlanner
#from commonroad_route_planner.utility.visualization import visualize_route

from commonroad_route_planner.utility.visualization import visualize_route
from commonroad_dc.costs.evaluation import CostFunctionEvaluator
from commonroad_dc.geometry.util import chaikins_corner_cutting, resample_polyline
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad.scenario.trajectory import Trajectory,State
from commonroad.common.solution import CommonRoadSolutionReader



path = os.path.abspath("")

# generate path of the file to be opened
file_path = "sixthScenario.xml"
solution_file_path = "solutionScenario.xml"
solution = CommonRoadSolutionReader.open(os.path.join(path, solution_file_path))

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

planning_problem = planning_problem_set.find_planning_problem_by_id(123)
reference_path = planning_problem.reference_path

# get the initial state of the ego vehicle from the planning problem set
planning_problem = planning_problem_set.find_planning_problem_by_id(123)
initial_state = planning_problem.initial_state

curvilinear_cosy = CurvilinearCoordinateSystem(reference_path)

curvilinear_cosy.compute_and_set_curvature()

init_position = planning_problem.initial_state.position

try:
    import matplotlib.pyplot as plt
except ImportError:
    print('Matplotlib not installed. Please use pip(3) to install required package!')

try:
    import numpy as npy
except ImportError:
    print('Numpy not installed. Please use pip(3) to install required package!')

try:
    import vehiclemodels
except ImportError:
    print('commonroad-vehicle-models not installed. Please use pip install to install required package!')

try:
    import pkg_resources
    pkg_resources.require("scipy>=1.1.0")
    pkg_resources.require("cvxpy>=1.0.0")
    from cvxpy import *
except ImportError:
    print('CVXPy not installed or wrong version. Please use pip(3) to install required package!')

class TIConstraints:
    a_min = -8
    a_max = 15
    s_min = 0
    s_max = 150
    v_min = 0
    v_max = 35
    j_min = -30
    j_max = 30


def plot_state_vector(x : Variable, c : TIConstraints, s_obj = None):
    plt.figure(figsize=(10,10))
    N = x.shape[1]-1
    s_max = npy.maximum(150,100+0*npy.ceil(npy.amax((x.value)[0,:].flatten())*1.1/10)*10)

    # Plot (x_t)_1.
    plt.subplot(4,1,1)
    x1 = (x.value)[0,:].flatten()
    plt.plot(npy.array(range(N+1)),x1,'g')
    if s_obj is not None:
        plt.plot(npy.array(range(1,N+1)),s_obj[0],'b')
        plt.plot(npy.array(range(1,N+1)),s_obj[1],'r')
    plt.ylabel(r"$s$", fontsize=16)
    plt.yticks(npy.linspace(c.s_min, s_max, 3))
    plt.ylim([c.s_min, s_max])
    plt.xticks([])

    # Plot (x_t)_2.
    plt.subplot(4,1,2)
    x2 = (x.value)[1,:].flatten()
    plt.plot(npy.array(range(N+1)),x2,'g')
    plt.yticks(npy.linspace(c.v_min,c.v_max,3))
    plt.ylim([c.v_min, c.v_max+2])
    plt.ylabel(r"$v$", fontsize=16)
    plt.xticks([])

    # Plot (x_t)_3.
    plt.subplot(4,1,3)
    x2 = (x.value)[2,:].flatten()
    plt.plot(npy.array(range(N+1)),x2,'g')
    plt.yticks(npy.linspace(c.a_min,c.a_max,3))
    plt.ylim([c.a_min, c.a_max+2])
    plt.ylabel(r"$a$", fontsize=16)
    plt.xticks([])

    # Plot (x_t)_4.
    plt.subplot(4,1,4)
    x2 = (x.value)[3,:].flatten()
    plt.plot(npy.array(range(N+1)), x2,'g')
    plt.yticks(npy.linspace(c.j_min,c.j_max,3))
    plt.ylim([c.j_min-1, c.j_max+1])
    plt.ylabel(r"$j$", fontsize=16)
    plt.xticks(npy.arange(0,N+1,5))
    plt.xlabel(r"$k$", fontsize=16)
    plt.tight_layout()
    plt.show()

# problem data
N  = 100  # number of time steps
n  = 4   # length of state vector
m  = 1   # length of input vector
dT = scenario.dt # time step


# set up variables
x = Variable(shape=(n,N+1)) # optimization vector x contains n states per time step
u = Variable(shape=(N)) # optimization vector u contains 1 state

# set up constraints
c = TIConstraints()
c.a_min = -6 # Minimum feasible acceleration of vehicle
c.a_max = 6 # Maximum feasible acceleration of vehicle
c.s_min = 0 # Minimum allowed position
c.s_max = 59 # Maximum allowed position
c.v_min = 0 # Minimum allowed velocity (no driving backwards!)
c.v_max = 35 # Maximum allowed velocity (speed limit)
c.j_min = -15 # Minimum allowed jerk
c.j_max = 15 # Maximum allowed jerk

# weights for cost function
w_s = 0
w_v = 8
w_a = 2
w_j = 2
Q = npy.eye(n)*npy.transpose(npy.array([w_s,w_v,w_a,w_j]))
w_u = 1
R = w_u

A = npy.array([[1,dT,(dT**2)/2,(dT**3)/6],
               [0,1,dT,(dT**2)/2],
               [0,0,1,dT],
               [0,0,0,1]])
B = npy.array([(dT**4)/24,
               (dT**3)/6,
               (dT**2)/2,
               dT]).reshape([n,])


# initial state of vehicle for the optimization problem (longitudinal position, velocity, acceleration, jerk)
x_0 = npy.array([curvilinear_cosy.convert_to_curvilinear_coords(init_position[0], init_position[1])[0],
                 initial_state.velocity,
                 0.0,
                 0.0]).reshape([n,]) # initial state

# reference velocity
v_ref = 30.0
# Set up optimization problem
states = []

cost = 0
# initial state constraint
constr = [x[:,0] == x_0]

for k in range(N):
    # cost function
    cost += quad_form(x[:,k+1] - npy.array([0,v_ref,0,0],), Q)\
           + R * u[k] ** 2
    # time variant state and input constraints
    constr.append(x[:,k+1] == A @ x[:,k] + B * u[k])

# sums problem objectives and concatenates constraints.
# create optimization problem
prob = Problem(Minimize(cost), constr)

# Solve optimization problem
prob.solve(verbose=True)
print("Problem is convex: ",prob.is_dcp())
print("Problem solution is "+prob.status)

# plot results
plot_state_vector(x, TIConstraints())

tiConstraints = [x[0,:] <= c.s_max, x[0,:] >= c.s_min]
tiConstraints += [x[1,:] <= c.v_max, x[1,:] >= c.v_min] # velocity
tiConstraints += [x[2,:] <= c.a_max, x[2,:] >= c.a_min] # acceleration
tiConstraints += [x[3,:] <= c.j_max, x[3,:] >= c.j_min] # jerk

# Adjust problem
prob = Problem(Minimize(cost), constr + tiConstraints)

# Solve optimization problem
prob.solve()
print("Problem is convex: ",prob.is_dcp())
print("Problem solution is "+prob.status)

# plot results
plot_state_vector(x, c)

# extract obstacle from scenario
dyn_obstacles = scenario.dynamic_obstacles

# create constraints for minimum and maximum position
s_min = [] # minimum position constraint
s_max = [] # maximum position constraint

# go through obstacle list and distinguish between following and leading vehicle
temp_boom = 0
collision_dict = {}
for o in dyn_obstacles:
    states_o = o.prediction.trajectory.state_list
    for t in range(len(states_o)):
        if o.obstacle_id == 320 and t == 47:
            if planning_problem.is_point_part_of_reference_path(states_o[t].position):
                coordinates_state_o = states_o[t].position
                found_collision = curvilinear_cosy.convert_to_curvilinear_coords(coordinates_state_o[0], coordinates_state_o[1])
                #constraint on maximum l before collision
                collision_dict[t] = found_collision[0]



"""
for o in dyn_obstacles:
    if o.initial_state.position[0] < x_0[0]:
        print('Following vehicle id={}'.format(o.obstacle_id))
        prediction = o.prediction.trajectory.state_list
        for p in prediction:
            s_min.append(o.obstacle_shape.length/2.+2.5)
    else:
        print('Leading vehicle id={}'.format(o.obstacle_id))
        prediction = o.prediction.trajectory.state_list
        for p in prediction:
            s_max.append(p.position[0]-o.obstacle_shape.length/2.-2.5)
"""
# plot vehicle motions
plt.plot(range(1,len(s_min)+1),s_min,'b')
plt.plot(range(1,len(s_max)+1),s_max,'r')



# initial state of ego vehicle
x_0 = npy.array([curvilinear_cosy.convert_to_curvilinear_coords(init_position[0], init_position[1])[0],
                 initial_state.velocity,
                 0,
                 0]).reshape([n,]) # initial state

# reference velocity
v_ref = 30.0

# Set up optimization problem
cost = 0
constr = [x[:,0] == x_0]
for k in range(N):
    # cost function
    cost += quad_form(x[:,k+1] - npy.array([0,v_ref,0,0],), Q)\
           + R * u[k] ** 2
    # single state and input constraints
    #constr.append(x[0,k+1] <=  x[0,k] + 9)

    constr.append(x[:,k+1] == A @ x[:,k] + B * u[k])
    # add obstacle constraint

    if collision_dict.get(k):
        constr.append(x[0,k+1] <= collision_dict[k] - 1)


# sums problem objectives and concatenates constraints.
prob = sum(states)
# add constraints for all states & inputs
prob = Problem(Minimize(cost), constr + tiConstraints)

#verbose=True
# Solve optimization problem
prob.solve()

print("Problem is convex:",prob.is_dcp())
print("Problem solution is "+prob.status)


x_result = x.value
s_ego = x_result[0,:].flatten()
v_ego = x_result[1,:].flatten()

# generate state list of the ego vehicle's trajectory
state_list = [initial_state]
for i in range(1, N):
    # compute new position
    # add new state to state_list
    state_list.append(State(**{'position': curvilinear_cosy.convert_to_cartesian_coords(s_ego[i],0.0),
                               'time_step': i, 'velocity': v_ego[i],
                               'velocity_y': 0}))

# create the planned trajectory starting at time step 1
ego_vehicle_trajectory = Trajectory(initial_time_step=1, state_list=state_list[1:])

from commonroad.common.solution import CommonRoadSolutionWriter, Solution, PlanningProblemSolution, VehicleModel, VehicleType, CostFunction

pps = PlanningProblemSolution(planning_problem_id=123,
                              vehicle_type=VehicleType.BMW_320i,
                              vehicle_model=VehicleModel.PM,
                              cost_function=CostFunction.JB1,
                              trajectory=ego_vehicle_trajectory)


# define the object with necessary attributes.
solution = Solution(scenario.scenario_id, [pps], computation_time=prob.solver_stats.solve_time)

# write solution to a xml file
csw = CommonRoadSolutionWriter(solution)
ce = CostFunctionEvaluator.init_from_solution(solution)
cost_result = ce.evaluate_solution(scenario, planning_problem_set, solution)

print(cost_result)

"""
for i in range(0, 1):
    fig = plt.figure(figsize=(40, 25))
    rnd = MPRenderer()
    # plot the scenario at different time step
    scenario.draw(rnd, draw_params={'time_begin': i})
    # plot the planning problem set
    planning_problem_set.draw(rnd)
    rnd.render(show=True)
    fig.savefig('output.svg')
    plt.show()

"""




"""

def distanceBetweenConsecutivePoints(point: List, nextPoint: List) -> float:
    return math.sqrt((nextPoint[0] - point[0])**2 +
                     (nextPoint[1] - point[1])**2)

def cosecutivePointsBetweenGivenDistance(dist: float, ref_path: np.ndarray) -> List:
    sum = 0
    saveIndex = 0
    for i in range(len(ref_path.tolist()) - 1):
        sum += distanceBetweenConsecutivePoints(ref_path.tolist()[i], ref_path[i+1])
        if sum >= dist:
            saveIndex = i
            break
    result = list()
    result.append(ref_path.tolist()[saveIndex])
    result.append(ref_path.tolist()[saveIndex+1])
    return result

def totalDistanceToCoordinatePoint(coordPoint: List, ref_path: np.ndarray) -> float:
    sum = 0
    for i in range(len(ref_path.tolist()) - 1):
        if ref_path.tolist()[i] == coordPoint:
            break
        sum += distanceBetweenConsecutivePoints(ref_path.tolist()[i], ref_path.tolist()[i+1])
    return sum


def coordinatesFromDistance(s: float, ref_path: np.ndarray) -> List:
    s_coord_list = list()
    consecutivePoints = cosecutivePointsBetweenGivenDistance(s, ref_path)
    d = distanceBetweenConsecutivePoints(consecutivePoints[0],consecutivePoints[1])
    A = consecutivePoints[0]
    B = consecutivePoints[1]
    a = s - totalDistanceToCoordinatePoint(A, ref_path)
    s_x = (a*(B[0] - A[0]) + d*A[0]) / d
    s_y = (a*(B[1] - A[1]) + d*A[1]) / d
    s_coord_list.append(s_x)
    s_coord_list.append(s_y)
    return s_coord_list

testArray = np.array([[-7,1],[-2,1],[1,-3], [-23,4], [-21,-8]])
testRes = distanceBetweenConsecutivePoints(testArray.tolist()[0], testArray.tolist()[1])
testPoints = cosecutivePointsBetweenGivenDistance(72.5, testArray)
s = 8
consecutivePoints = cosecutivePointsBetweenGivenDistance(s, testArray)
distanceBetweenFoundPoints = distanceBetweenConsecutivePoints(consecutivePoints[0],consecutivePoints[1])
testCoordiantes = coordinatesFromDistance(s, testArray)

testDistanceToCoord = totalDistanceToCoordinatePoint(testArray.tolist()[1], testArray)


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

# we apply chaikins corner cutting 10 times to smoothen the reference path
#for i in range(0, 10):
#    reference_path = chaikins_corner_cutting(reference_path)
#reference_path = resample_polyline(reference_path, 2.0)

# create curvilinear CoSy

def dist_to_point_ref_path(point_end):
    result = 0
    for i in range(point_end):
        result+=math.sqrt((reference_path[i][0] - reference_path[i+1][0])**2 +
                          (reference_path[i][1]-reference_path[i+1][1])**2)
    return result


curvilinear_cosy = CurvilinearCoordinateSystem(reference_path)

#curvilinear_cosy.compute_and_set_curvature()
#temp = curvilinear_cosy.get_curvature()
# original cartesian point
#p_cartesian = np.array([434.32135, -401.14530])
#temp = dist_to_point_ref_path(1)
p_cartesian = reference_path[22]
# convert to curvilinear coordinates

states_s_list = []
for i in range(len(reference_path)):
    states_s_list.append(curvilinear_cosy.convert_to_curvilinear_coords(reference_path[i][0], reference_path[i][1]))
states_s = np.array(states_s_list)

coords_back_list = []
for i in range(len(reference_path)):
    coords_back_list.append(curvilinear_cosy.convert_to_cartesian_coords(states_s[i][0], states_s[i][1]))
coords_back = np.array(coords_back_list)

p_curvilinear = curvilinear_cosy.convert_to_curvilinear_coords(p_cartesian[0], p_cartesian[1])
print('Converted p_cartesian in curvilinear coordinates: {}'.format(p_curvilinear))


# back-convert to cartesian coordinates
p_cartesian2 = curvilinear_cosy.convert_to_cartesian_coords(p_curvilinear[0], p_curvilinear[1])
print('Back-converted p_curvilinear in cartesian coordinates: {}'.format(p_cartesian))

list_temp = [5.810860051642546, 0.0] 
test_temp = np.array(list_temp)
curv_temp = curvilinear_cosy.convert_to_cartesian_coords(test_temp[0],test_temp[1])




reference_path_list = reference_path.tolist()

def distanceBetweenConsecutivePoints(point: List, nextPoint: List) -> float:
    return math.sqrt((nextPoint[0] - point[0])**2 +
                     (nextPoint[1] - point[1])**2)

open("trajectory", "w").close()
f = open("trajectory.txt", "w")
f.write("<pmTrajectory planningProblem=123>\n")
temp = 0
for it in range(len(reference_path_list)-1):
    f.write("\t<pmState>\n")
    f.write("\t\t<x>" + str(reference_path_list[it][0]) + "</x>\n")
    f.write("\t\t<y>" + str(reference_path_list[it][1]) + "</y>\n")
    d = distanceBetweenConsecutivePoints(reference_path_list[it],reference_path_list[it+1])
    velocity = ((d * math.sqrt(2)) / 2) * 5.11
    velocity_y = velocity
    f.write("\t\t<xVelocity>" + str(velocity) + "</xVelocity>\n")
    f.write("\t\t<yVelocity>" + str(velocity_y) + "</yVelocity>\n")
    if it % 5 == 0 and it < 485:
        temp+=1
    if it == 490:
        temp +=1
    if it == 500:
        temp+=1
    if it == 510:
        temp+=1
    f.write("\t\t<time>" + str(temp) + "</time>\n")
    f.write("\t</pmState>\n")
f.write("</pmTrajectory>")
f.close()
# plot the scenario for 40 time step, here each time step corresponds to 0.1 second

    #plt.plot(ndArrayCoordinates, zorder=200)

#plt.plot([420,500],[-400,-250], zorder=200)

#test_tt = scenario.dynamic_obstacles[7].prediction.trajectory.state_list[47].position
#bbb = curvilinear_cosy.convert_to_curvilinear_coords(test_tt[0], test_tt[1])

# plot results
#seems to crash
#plot_state_vector(x, c, [s_min,s_max])

s_list_diff= []
for i in range(len(s_ego)-1):
    s_list_diff.append(s_ego[i+1] - s_ego[i])
s_ndarr = np.array(s_list_diff)



vehicle3 = parameters_vehicle3.parameters_vehicle3()
ego_vehicle_shape = Rectangle(length=vehicle3.l, width=vehicle3.w)
ego_vehicle_prediction = TrajectoryPrediction(trajectory=ego_vehicle_trajectory,
                                              shape=ego_vehicle_shape)

# the ego vehicle can be visualized by converting it into a DynamicObstacle
ego_vehicle_type = ObstacleType.CAR
ego_vehicle = DynamicObstacle(obstacle_id=100, obstacle_type=ego_vehicle_type,
                              obstacle_shape=ego_vehicle_shape, initial_state=initial_state,
                              prediction=ego_vehicle_prediction)


# plot the scenario and the ego vehicle for each time step
for i in range(0, 40):
    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    scenario.draw(rnd, draw_params={'time_begin': i})
    ego_vehicle.draw(rnd, draw_params={'time_begin': i, 'dynamic_obstacle': {
        'vehicle_shape': {'occupancy': {'shape': {'rectangle': {
            'facecolor': 'g'}}}}}})
    planning_problem_set.draw(rnd)
    rnd.render()

"""
