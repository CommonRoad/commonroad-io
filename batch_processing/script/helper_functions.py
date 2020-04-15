import os
import pathlib
import multiprocessing
import yaml
import sys
import warnings

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution_writer import CommonRoadSolutionWriter, VehicleModel, VehicleType, CostFunction

# helper functions for parsing
def parse_vehicle_model(model: str):
    if model == 'PM':
        cr_model = VehicleModel.PM
    elif model == 'ST':
        cr_model = VehicleModel.ST
    elif model == 'KS':
        cr_model = VehicleModel.KS
    elif model == 'MB':
        cr_model = VehicleModel.MB
    else:
        raise ValueError('Selected vehicle model is not valid: {}.'.format(model))
    return cr_model


def parse_vehicle_type(type: str):
    if type == 'FORD_ESCORT':
        cr_type = VehicleType.FORD_ESCORT
        cr_type_id = 1
    elif type == 'BMW_320i':
        cr_type = VehicleType.BMW_320i
        cr_type_id = 2
    elif type == 'VW_VANAGON':
        cr_type = VehicleType.VW_VANAGON
        cr_type_id = 3
    else:
        raise ValueError('Selected vehicle type is not valid: {}.'.format(type))
        
    return cr_type, cr_type_id


def parse_cost_function(cost:str):
    if cost == 'JB1':
        cr_cost = CostFunction.JB1
    elif cost == 'SA1':
        cr_cost = CostFunction.SA1
    elif cost == 'WX1':
        cr_cost = CostFunction.WX1
    elif cost == 'SM1':
        cr_cost = CostFunction.SM1
    elif cost == 'SM2':
        cr_cost = CostFunction.SM2
    elif cost == 'SM3':
        cr_cost = CostFunction.SM3
    else:
        raise ValueError('Selected cost function is not valid: {}.'.format(cost))
    return cr_cost

def call_motion_planner(queue, function, scenario, planning_problem_set, vehicle_type_id=2, planning_problem_idx=0, planner_id=1, max_tree_depth=100):
    try:
        solution_trajectory = function(scenario, planning_problem_set, vehicle_type_id, planning_problem_idx, planner_id, max_tree_depth)
    except Exception as error:
        print(error)
    else:
        queue.put(solution_trajectory)

def load_configuration(path_file_config):
    '''
    loads input configuration file
    '''
    with open(path_file_config, 'r') as stream:
        try:
            configuration = yaml.load(stream)  

            return configuration
        except yaml.YAMLError as exc:
            print(exc)

def get_target_function(configuration):
    '''
    get target function
    '''

    # adds both absolute and relative
    path_absolute = os.path.dirname(configuration['motion_planner_path'])
    path_relative = os.getcwd() + os.path.dirname(configuration['motion_planner_path'])
    sys.path.append(path_relative)
    sys.path.append(path_absolute)

    # check the existence of the module and the function
    name_module = configuration['motion_planner_module_name']
    assert os.path.exists(os.path.join(path_relative, name_module + ".py")) or \
           os.path.exists(os.path.join(path_absolute, name_module + ".py")), "<ERROR> Module {} does not exist!".format(name_module)
    module = __import__(configuration['motion_planner_module_name'])

    name_function = configuration['motion_planner_function_name']
    assert hasattr(module, name_function), "<ERROR> Function {} not found in module {}".format(name_function, name_module)
    target_function = getattr(module, name_function)

    return target_function

def get_input_files(configuration):
    '''
    return a list of scenarios
    '''
    list_files_input = os.listdir(configuration['input_path'])
    list_files_input_verified = []

    for file in list_files_input:    
        if not file.endswith('.xml'):
            continue
        else:
            list_files_input_verified.append(file)

    return list_files_input_verified

def parse_scenario_file(configuration, file_scenario):
    '''
    parse the scenario file and return scenario, planning problem set, vehicle model, etc.
    '''
    path_file_full = os.path.join(configuration['input_path'], file_scenario)
    scenario, planning_problem_set = CommonRoadFileReader(path_file_full).open()

    # get configuration for each scenario
    if scenario.benchmark_id in configuration.keys():
        # configuration for specific scenario
        vehicle_model = parse_vehicle_model(configuration[scenario.benchmark_id]['vehicle_model'])
        vehicle_type, vehicle_type_id = parse_vehicle_type(configuration[scenario.benchmark_id]['vehicle_type'])
        cost_function = parse_cost_function(configuration[scenario.benchmark_id]['cost_function'])
        planning_problem_idx = configuration[scenario.benchmark_id]['planning_problem_idx']
        planner_id = configuration[scenario.benchmark_id]['planner_id']
        max_tree_depth = configuration[scenario.benchmark_id]['max_tree_depth']
    else:
        # default configuration
        vehicle_model = parse_vehicle_model(configuration['default']['vehicle_model'])
        vehicle_type, vehicle_type_id = parse_vehicle_type(configuration['default']['vehicle_type'])
        cost_function = parse_cost_function(configuration['default']['cost_function'])
        planning_problem_idx = configuration['default']['planning_problem_idx']
        planner_id = configuration['default']['planner_id']
        max_tree_depth = configuration['default']['max_tree_depth']

    return (scenario, planning_problem_set, vehicle_type_id, vehicle_type, vehicle_model, cost_function, planning_problem_idx, planner_id, max_tree_depth)

def execute_target_function(function_target, result_parse, time_timeout):
    '''
    function to create a process to execute target function
    '''
    queue = multiprocessing.Queue()

    # create process, pass in required arguements
    p = multiprocessing.Process(target=call_motion_planner, name="motion_planner",
                                args=(queue, function_target, result_parse[0], result_parse[1], result_parse[2], 
                                      result_parse[6], result_parse[7], result_parse[8]))
    # start planning
    p.start()
    
    # wait till the process ends or skip if timed out
    p.join(timeout=time_timeout)

    if p.is_alive():
        # process join()ed but still alive -> timeout
        print("<TIMEOUT> Motion planner timeout.")
        p.terminate()
        p.join()
        return None
    else:
        if queue.empty():
            # queue is empty due to error in the process
            return None
        else:
            solution_trajectories = queue.get()

            return solution_trajectories

def save_solution(configuration, solution_trajectories, result_parse):
    '''
    save the solution if it is a valid one
    '''
    
    if solution_trajectories is None:
        print('<FAILURE> Solution not Found')
        return

    # create path for solutions
    pathlib.Path(configuration['output_path']).mkdir(parents=True, exist_ok=True)

    flag_error_solution = False
    cr_solution_writer = CommonRoadSolutionWriter(configuration['output_path'], 
                                                  result_parse[0].benchmark_id, 
                                                  result_parse[0].dt,
                                                  result_parse[3], 
                                                  result_parse[4], 
                                                  result_parse[5])
    
    # inspect whether all planning problems are solved
    planning_problem_set = result_parse[1]
    for planning_problem_id, planning_problem in planning_problem_set.planning_problem_dict.items():
        if planning_problem_id not in solution_trajectories.keys():
            print('<FAILURE> Solution for planning problem with ID={} is not provided. Skipped.'.format(planning_problem_id))
            flag_error_solution = True
            break
        else:
            cr_solution_writer.add_solution_trajectory(solution_trajectories[planning_problem_id], planning_problem_id)

    if not flag_error_solution:
        cr_solution_writer.write_to_file(overwrite=configuration['overwrite'])
        print("<SUCCESS> Solution file written.")