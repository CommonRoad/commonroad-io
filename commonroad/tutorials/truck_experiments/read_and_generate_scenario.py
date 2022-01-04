import numpy as np
import json

from commonroad.scenario.trajectory import State, Trajectory
from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics
from commonroad.common.solution import Solution, VehicleType, PlanningProblemSolution, VehicleModel, CostFunction, \
    CommonRoadSolutionWriter
from commonroad.scenario.scenario import ScenarioID
import commonroad_dc.feasibility.feasibility_checker as feasibility_checker

# specify obstacle, poses and trajectory file
OBSTACLE_FILE = "input/CTA_Obstacles.json"
POSES_FILE = "input/poses_Hamburg_CTA.json"
TRAJECTORY_FILE = "input/04M001.json"

# offset between world and local (CommonRoad) coordinates
OFFSET_X = 561676.6763867161
OFFSET_Y = 5928014.473294518

# template for a single state in the trajectory list
state_template = "\n  <state>\n    <position>\n      <point>\n        <x>%s</x>\n        <y>%s</y>\n      </point>" \
        "\n    </position>\n    <orientation>\n      <exact>%s</exact>\n    </orientation>\n    <time>" \
        "\n      <exact>%s</exact>\n    </time>\n    <velocity>\n      <exact>%s</exact>\n    </velocity>" \
        "\n    <acceleration>\n      <exact>%s</exact>\n    </acceleration>\n    <hitch>" \
        "\n      <exact>%s</exact>\n    </hitch>\n  </state>"


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


def get_local_coords(x_corr, y_corr):
    local_x = []
    local_y = []
    for i in range(len(x_corr)):
        local_x.append(x_corr[i] - OFFSET_X)
        local_y.append(y_corr[i] - OFFSET_Y)
    return local_x, local_y


def check_angles(solution):
    alpha = solution['alpha']
    theta = solution['theta']
    theta_trailer = solution['theta_trailer']
    for i in range(len(alpha)):
        alpha_fake = theta[i] - theta_trailer[i]
        alpha_real = alpha[i]
        print("%s =? %s" % (str(alpha_fake), str(alpha_real)))


def check_x(x_list, s_x, s_xt, theta, alpha):
    # x_fake = s_xt + np.cos(theta + alpha) * 8.755 + 0.5 * np.cos(theta) * 3.6
    x_fake = s_x + 0.5 * np.cos(theta) * 3.6
    x_list.append(x_fake)
    x_trailer = s_xt
    length = -(x_trailer + s_x) / np.cos(theta + alpha)
    print("length = " + str(length))
    print("x: %s ?= %s, diff = %s" % (str(x_fake), str(s_x), str(x_fake - s_x)))


def check_y(y_list, s_y, s_yt, theta, alpha):
    # y_fake = s_yt + np.sin(theta + alpha) * 8.755 + 0.5 * np.sin(theta) * 3.6
    y_fake = s_y + 0.5 * np.sin(theta) * 3.6
    y_list.append(y_fake)
    y_trailer = s_yt
    length = -(y_trailer - s_y) / np.sin(theta + alpha)
    print("length = " + str(length))
    print("y: %s ?= %s, diff = %s" % (str(y_fake), str(s_y), str(y_fake - s_y)))


# update coordinates with the corrected central position for truck
# calculated from the real wheel position, angle and wheelbase length
def update_coords(solution):
    s_x, s_y = solution['x'], solution['y']
    theta = solution['theta']
    x_updated, y_updated = [], []
    for i in range(len(theta)):
        x_updated.append(s_x[i] + 0.5 * np.cos(theta[i]) * 3.6)
        y_updated.append(s_y[i] + 0.5 * np.sin(theta[i]) * 3.6)
    return x_updated, y_updated


def get_trajectory(json_data):
    solution = json_data['final_solution']
    # update coordinates to the CommonRoad model
    x_corr, y_corr = update_coords(solution)
    hitch_angles = solution['alpha']
    v_long = solution['direction']
    steering_angles = solution['phi'] # delta
    orientation = solution['theta']
    # calculate CommonRoad coordinates from world coordinates
    s_x, s_y = get_local_coords(x_corr, y_corr)
    return hitch_angles, v_long, steering_angles, orientation, s_x, s_y


# write trajectory entry
def write_commonroad_trajectory(json_data):
    hitch, v_long, steering, orient, x, y = get_trajectory(json_data)
    states = []
    with open("generated_trajectory.txt", 'w') as traj:
        traj.write("<trajectory>")
        for i in range(len(x)):
            s_x, s_y, ori = str(x[i]), str(y[i]), str(orient[i])
            time, vel, hit = str(i + 1), str(v_long[i]), str(hitch[i])
            traj.write(state_template % (s_x, s_y, ori, time, vel, str(0), hit))
            state = State(
                position=np.array([x[i], y[i]]),
                velocity=v_long[i],
                orientation=orient[i],
                steering_angle=steering[i],
                time_step=i,
                hitch=hitch[i]
            )
            states.append(state)
        traj.write("\n</trajectory>")
    return states


# read obstacle data
obstacle_data = read_json_file(OBSTACLE_FILE)
local_obstacle_coords = world_to_local_coords(obstacle_data, tuple_size=2)

# read poses data
poses_data = read_json_file(POSES_FILE)
get_poses(poses_data)

# read trajectory data
trajectory_data = read_json_file(TRAJECTORY_FILE)
states = write_commonroad_trajectory(trajectory_data)
np.seterr(all='print')
dt = 0.1
vehicle = VehicleDynamics.KST(VehicleType.TRUCK_MAN)
trajectory = Trajectory(0, states)
feasible, reconstructed_inputs = feasibility_checker.trajectory_feasibility(trajectory, vehicle, dt)
print('Feasible? {}'.format(feasible))
pp_solution = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.KST,
            vehicle_type=VehicleType.TRUCK_MAN,
            cost_function=CostFunction.TR1,
            trajectory=trajectory
)
solution = Solution(scenario_id=ScenarioID(), planning_problem_solutions=[pp_solution])
csw = CommonRoadSolutionWriter(solution)
csw.write_to_file(overwrite=True)
