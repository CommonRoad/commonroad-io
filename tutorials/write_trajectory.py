import json
import numpy as np
from commonroad.scenario.trajectory import State


def read_json_file(file_path):
    f = open(file_path)
    data = json.load(f)
    f.close()
    return data

def get_local_coords(x_corr, y_corr):
    local_x = []
    local_y = []
    for i in range(len(x_corr)):
        local_x.append(x_corr[i] - OFFSET_X)
        local_y.append(y_corr[i] - OFFSET_Y)
    return local_x, local_y

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
    for i in range(len(x)):
        s_x, s_y, ori = str(x[i]), str(y[i]), str(orient[i])
        time, vel, hit = str(i + 1), str(v_long[i]), str(hitch[i])
        state = State(
            position=np.array([x[i], y[i]]),
            velocity=v_long[i],
            orientation=orient[i],
            steering_angle=steering[i],
            time_step=i,
            hitch=hitch[i]
        )
        # state = [np.array([x[i], y[i]]),v_long[i],orient[i],steering[i],i,hitch[i] ]
        states.append(state)
    return states

if __name__ == "__main__":

    TRAJECTORY_FILE = "04M001.json"
    # offset between world and local (CommonRoad) coordinates
    OFFSET_X = 561676.6763867161
    OFFSET_Y = 5928014.473294518
    # read trajectory data
    trajectory_data = read_json_file(TRAJECTORY_FILE)
    states = write_commonroad_trajectory(trajectory_data)
    data = np.array( states, dtype=object)
    output_npz_file = 'trajectory'
    np.save(output_npz_file, data)

    # to read file
    # np_load_old = np.load

    # # # modify the default parameters of np.load
    # np.load = lambda *a,**k: np_load_old(*a, allow_pickle=True, **k)
    # st = np.load(output_npz_file+".npy")
    # # st = [ State(position=st[i][0], velocity=st[i][1], orientation=st[i][2], steering_angle=st[i][3], time_step=st[i][4], hitch=st[i][5]) for i in range(len(st))]

    # np.load = np_load_old
    # print(st[0])

