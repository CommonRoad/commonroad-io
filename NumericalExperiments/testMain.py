import numpy as np

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory,State
from commonroad.prediction.prediction import TrajectoryPrediction

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
try:
    import commonroad_dc
except:
    print('commonroad-drivability-checher not installed. Please use pip install to install required package!')

from commonroad_dc.costs.evaluation import CostFunctionEvaluator
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from vehiclemodels import parameters_vehicle3

# generate path of the file to be opened
file_path = "DEU_Flensburg-6_1_T-1.xml"
#file_path = "USA_Peach-4_2_T-1.xml"
#file_path = "DEU_Muc-28_1_T-1.xml"
# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

planning_problem = next(iter(planning_problem_set.planning_problem_dict.values()))

# get the reference path of the planning problem
reference_path = planning_problem.reference_path

# get the initial state of the ego vehicle from the planning problem set
initial_state = planning_problem.initial_state

# get the initial position of the intial state
init_position = planning_problem.initial_state.position

# get the one-dimensional curvilinear coordinate mapping of the reference path discrete two-dimensional points
curvilinear_cosy = CurvilinearCoordinateSystem(reference_path)


# base constraints for acceleration, travel distance, velocity, jerk
class TIConstraints:
    a_min = -8
    a_max = 15
    s_min = 0
    s_max = 170
    v_min = 0
    v_max = 35
    j_min = -30
    j_max = 30

# plots the travel distance, velocity, acceleration and jerk as one-dimensional functions
def plot_state_vector(x : Variable, c : TIConstraints, s_obj = None):
    fig = plt.figure(figsize=(10,10))
    N = x.shape[1]-1
    s_max = npy.maximum(150,100+0*npy.ceil(npy.amax((x.value)[0,:].flatten())*1.1/10)*10)

    # Plot (x_t)_1.
    plt.subplot(4,1,1)
    x1 = (x.value)[0,:].flatten()
    plt.plot(npy.array(range(N+1)),x1,'g')
    if s_obj is not None:
        plt.plot(npy.array(range(1,N+1)),s_obj[0],'b')
        plt.plot(npy.array(range(1,N+1)),s_obj[1],'r')
    plt.ylabel(r"$s$ [$m$]", fontsize=16)
    plt.grid(visible=True)
    plt.yticks(npy.linspace(c.s_min, s_max, 3))
    plt.ylim([c.s_min, s_max])
    plt.xticks([])
    plt.grid(visible=True)
    # Plot (x_t)_2.
    plt.subplot(4,1,2)
    x2 = (x.value)[1,:].flatten()
    plt.plot(npy.array(range(N+1)),x2,'g')
    plt.yticks(npy.linspace(c.v_min,c.v_max,3))
    plt.ylim([c.v_min, c.v_max+2])
    plt.ylabel(r"$v$ [$m/s$]", fontsize=16)
    plt.grid(visible=True)
    plt.xticks([])
    plt.grid(visible=True)
    # Plot (x_t)_3.
    plt.subplot(4,1,3)
    x2 = (x.value)[2,:].flatten()
    plt.plot(npy.array(range(N+1)),x2,'g')
    plt.yticks(npy.linspace(c.a_min,c.a_max,3))
    plt.ylim([c.a_min, c.a_max+2])
    plt.ylabel(r"$a$ $m/s^{2}$]", fontsize=16)
    plt.grid(visible=True)
    plt.xticks([])
    plt.grid(visible=True)
    # Plot (x_t)_4.
    plt.subplot(4,1,4)
    x2 = (x.value)[3,:].flatten()
    plt.plot(npy.array(range(N+1)), x2,'g')
    plt.yticks(npy.linspace(c.j_min,c.j_max,3))
    plt.ylim([c.j_min-1, c.j_max+1])
    plt.ylabel(r"$j$ $m/s^{3}$]", fontsize=16)
    plt.xticks(npy.arange(0,N+1,5))
    plt.xlabel(r"$k$", fontsize=16)
    plt.tight_layout()
    fig.savefig('svaj3.svg')
    plt.show()

# problem data

# gets the maximum time-steps of the scenario
total_time_steps = 0
for obs in scenario.dynamic_obstacles:
    if obs.prediction is None:
        continue
    if obs.prediction.trajectory.state_list[-1].time_step > total_time_steps:
        total_time_steps = obs.prediction.trajectory.state_list[-1].time_step

N  = total_time_steps  # number of time steps
n  = 4   # length of state vector
m  = 1   # length of input vector
dT = scenario.dt # time step


# set up variables
x = Variable(shape=(n,N+1)) # optimization vector x contains n states per time step
u = Variable(shape=(N)) # optimization vector u contains 1 state

# set up base constraints
c = TIConstraints()
c.a_min = -6 # Minimum feasible acceleration of vehicle
c.a_max = 6 # Maximum feasible acceleration of vehicle
c.s_min = 0 # Minimum allowed travel distance
c.s_max = curvilinear_cosy.length() # Maximum allowed travel distance
c.v_min = 0 # Minimum allowed velocity (no driving backwards!)
c.v_max = 35 # Maximum allowed velocity (speed limit)
c.j_min = -15 # Minimum allowed jerk
c.j_max = 15 # Maximum allowed jerk

# weights for the cost function
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


# initial state of vehicle for the optimization problem (longitudinal position(one-dimensional), velocity, acceleration, jerk)
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

# additional constraints
tiConstraints = [x[0,:] <= c.s_max, x[0,:] >= c.s_min]  # distance
tiConstraints += [x[1,:] <= c.v_max, x[1,:] >= c.v_min] # velocity
tiConstraints += [x[2,:] <= c.a_max, x[2,:] >= c.a_min] # acceleration
tiConstraints += [x[3,:] <= c.j_max, x[3,:] >= c.j_min] # jerk

curvilinear_cosy.compute_and_set_curvature()
curve_rate = curvilinear_cosy.get_curvature()
kamm_circle_constr = []
for k in range(N):
    # cost function
    cost += quad_form(x[:,k+1] - npy.array([0,v_ref,0,0],), Q)\
           + R * u[k] ** 2
    # time variant state and input constraints
    constr.append(x[:,k+1] == A @ x[:,k] + B * u[k])

    #Kamm's circle
    kamm_circle_constr.append(x[2,k+1]**2 + (x[1,k+1] * curve_rate[k])**2 <= 132.25)

# Adjust problem
prob = Problem(Minimize(cost), constr + tiConstraints + kamm_circle_constr)

# Solve optimization problem
prob.solve(verbose=True)

# plot results
plot_state_vector(x, c)


x_result = x.value
# one-dimensional list of distance traveled for each time step of the ego-vehicle
s_ego = x_result[0,:].flatten()
# velocity for each time step of the ego-vehicle
v_ego = x_result[1,:].flatten()


# generate state list of the ego vehicle's trajectory
state_list = [initial_state]
for i in range(1, N):
    # compute new position
    # add new state to state_list
    # add the important parameters for point-mass model + the orientation for visualizing the two-dimensional motion planning
    if i < N - 1:
        point_A = curvilinear_cosy.convert_to_cartesian_coords(s_ego[i],0.0)
        point_B = curvilinear_cosy.convert_to_cartesian_coords(s_ego[i+1],0.0)
        opp = point_B[1] - point_A[1]
        adj = point_B[0] - point_A[0]
        heading_angle = npy.arctan2(opp,adj)
    state_list.append(State(**{'position': curvilinear_cosy.convert_to_cartesian_coords(s_ego[i],0.0),
                               'orientation': heading_angle,
                               'time_step': i, 'velocity': v_ego[i],
                               'velocity_y': 0}))

# create the planned trajectory starting at time step 1
ego_vehicle_trajectory = Trajectory(initial_time_step=1, state_list=state_list[1:])

vehicle3 = parameters_vehicle3.parameters_vehicle3()
ego_vehicle_shape = Rectangle(length=vehicle3.l, width=vehicle3.w)
ego_vehicle_prediction = TrajectoryPrediction(trajectory=ego_vehicle_trajectory,
                                              shape=ego_vehicle_shape)

# the ego vehicle can be visualized by converting it into a DynamicObstacle
ego_vehicle_type = ObstacleType.CAR
ego_vehicle = DynamicObstacle(obstacle_id=100, obstacle_type=ego_vehicle_type,
                              obstacle_shape=ego_vehicle_shape, initial_state=initial_state,
                              prediction=ego_vehicle_prediction)


for i in range(0, N):
    plt.figure(figsize=(40, 25))
    rnd = MPRenderer()
    scenario.draw(rnd, draw_params={'time_begin': i})
    ego_vehicle.draw(rnd, draw_params={'time_begin': i, 'dynamic_obstacle': {
        'vehicle_shape': {'occupancy': {'shape': {'rectangle': {
            'facecolor': 'g'}}}}}})
    planning_problem_set.draw(rnd)
    rnd.render()


from commonroad.common.solution import CommonRoadSolutionWriter, Solution, PlanningProblemSolution, VehicleModel, VehicleType, CostFunction

# generate the final planning problem solution
pps = PlanningProblemSolution(planning_problem_id=planning_problem.planning_problem_id,
                              vehicle_type=VehicleType.BMW_320i,
                              vehicle_model=VehicleModel.PM,
                              cost_function=CostFunction.OD1,
                              trajectory=ego_vehicle_trajectory)

# define the solution with the necessary attributes
solution = Solution(scenario.scenario_id, [pps], computation_time=prob.solver_stats.solve_time)

# write the solution to a xml file
csw = CommonRoadSolutionWriter(solution)
csw.write_to_file(overwrite=True)

# evaluate solution
ce = CostFunctionEvaluator.init_from_solution(solution)
cost_result = ce.evaluate_solution(scenario, planning_problem_set, solution)


# print the total cost of the solution of the planning problem
print(cost_result)

