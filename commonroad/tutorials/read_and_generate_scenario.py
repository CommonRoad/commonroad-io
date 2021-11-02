from vehiclemodels.vehicle_dynamics_kst import vehicle_dynamics_kst
from vehiclemodels.parameters_vehicle4 import parameters_vehicle4
from vehiclemodels.init_kst import init_kst
from scipy.integrate import odeint
import numpy as np
import json

OBSTACLE_FILE = "CTA_Obstacles.json"
POSES_FILE = "poses_Hamburg_CTA.json"
TRAJECTORY_FILE = "04M001.json"
OFFSET_X = 561676.6763867161
OFFSET_Y = 5928014.473294518

class MANTruckSimulator:

    def __init__(self):
        self.load_parameters()
        self.set_simulation_time()
        self.set_initial_state()

    @staticmethod
    def func_KST(x, t, u, p):
        f = vehicle_dynamics_kst(x, u, p)
        return f

    def load_parameters(self):
        self.p = parameters_vehicle4()
        self.g = 9.81  # [m/s^2]

    def set_simulation_time(self):
        self.tStart = 0  # start time
        self.tFinal = 1573  # end time

    def set_initial_state(self):
        self.delta0 = 0
        self.Psi0 = 0
        self.dotPsi0 = 0
        self.beta0 = 0
        self.sx0 = 0
        self.sy0 = 0
        self.alpha0 = 0

    def calculate_all_steps(self, long_vel, steering_angle):
        t = np.arange(self.tStart, self.tFinal, 1)
        states = []
        if len(long_vel) != len(steering_angle):
            return states
        for i in range(len(long_vel)):
            u = [steering_angle[i], 0]
            initialState = [self.sx0, self.sy0, self.delta0, long_vel[i], self.Psi0, self.dotPsi0,
                                 self.beta0]  # initial state for simulation
            x0_KST = init_kst(initialState, [0])  # initial state for kinematic single-track trailer model
            x_left_kst = odeint(self.func_KST, x0_KST, t, args=(u, self.p))
            states.append(x_left_kst)
        return states

def read_json_file(file_path):
    f = open(file_path)
    data = json.load(f)
    f.close()
    return data

def world_to_local_coords(obstacles, tuple_size):
    updated_obstacles = []
    for o in obstacles:
        polygons = o
        length = len(o)
        for i in range(0, length, tuple_size):
            polygons[i] -= OFFSET_X
            polygons[i + 1] -= OFFSET_Y
        updated_obstacles.append(polygons)
    return updated_obstacles

def get_lanes(points_parking):
    left_lane = world_to_local_coords(points_parking['left_lane'], 4)
    right_lane = world_to_local_coords(points_parking['right_lane'], 4)
    side_lane = world_to_local_coords(points_parking['side_lane'], 4)
    return left_lane, right_lane, side_lane

def get_bays(parking_lots):
    bays_list = []
    for key, value in parking_lots.items():
        bays_list.append(world_to_local_coords(value, 4))
    return bays_list

def get_poses(json_data):
    entry_points_parking = json_data['entry_points_parking']
    entry_points_left_lane, entry_points_right_lane, entry_points_side_lane = get_lanes(entry_points_parking)
    parking_slot_points = json_data['parking_slot_points']
    updated_parking_slot_points = get_bays(parking_slot_points)
    exit_points_parking = json_data['exit_points_parking']
    exit_points_left_lane, exit_points_right_lane, exit_points_side_lane = get_lanes(exit_points_parking)

def get_local_coords(s_x, s_y):
    initial_x = s_x[0]
    initial_y = s_y[0]
    for x in s_x:
        x -= initial_x
    for y in s_y:
        y -= initial_y
    return s_x, s_y

def get_trajectory(json_data):
    solution = json_data['final_solution']
    hitch_angles = solution['alpha']
    v_long = solution['direction']
    # delta
    steering_angles = solution['phi']
    orientation = solution['theta_trailer']
    s_x, s_y = get_local_coords(solution['x_trailer'], solution['y_trailer'])
    return hitch_angles, v_long, steering_angles, orientation, s_x, s_y

def write_commonroad_trajectory(json_data):
    time = 1
    hitch, v_long, steering, orient, x, y = get_trajectory(json_data)
    simulator = MANTruckSimulator()
    steps = simulator.calculate_all_steps(v_long, steering)
    with open("trajectory", 'w') as traj:
        traj.write("<trajectory>")
        for i in range(len(steps)):
            traj.write("\n  <state>")
            traj.write("\n    <position>")
            traj.write("\n      <point>")
            traj.write("\n        <x>" + str(x[i]) + "</x>")
            traj.write("\n        <y>" + str(y[i]) + "</y>")
            traj.write("\n      </point>")
            traj.write("\n    </position>")
            traj.write("\n    <orientation>")
            traj.write("\n      <exact>" + str(orient[i]) + "</exact>")
            traj.write("\n    </orientation>")
            traj.write("\n    <time>")
            traj.write("\n      <exact>" + str(time) + "</exact>")
            traj.write("\n    </time>")
            traj.write("\n    <velocity>")
            traj.write("\n      <exact>" + str(v_long[i]) + "</exact>")
            traj.write("\n    </velocity>")
            traj.write("\n    <acceleration>")
            traj.write("\n      <exact>" + str(0) + "</exact>")
            traj.write("\n    </acceleration>")
            traj.write("\n    <hitch>")
            traj.write("\n      <exact>" + str(hitch[i]) + "</exact>")
            traj.write("\n    </hitch>")
            traj.write("\n  </state>")
            time += 1
        traj.write("\n</trajectory>")

obstacle_data = read_json_file(OBSTACLE_FILE)
local_obstacle_coords = world_to_local_coords(obstacle_data, 2)
# There two above have already been done so we might as well just skip them I guess
poses_data = read_json_file(POSES_FILE)
get_poses(poses_data)
trajectory_data = read_json_file(TRAJECTORY_FILE)
write_commonroad_trajectory(trajectory_data)