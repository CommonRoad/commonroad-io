import os
import pathlib
import multiprocessing
import yaml
import sys
import warnings

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution_writer import CommonRoadSolutionWriter, VehicleModel, VehicleType, CostFunction


def parse_vehicle_model(model):
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


def parse_vehicle_type(type):
    if type == 'FORD_ESCORT':
        cr_type = VehicleType.FORD_ESCORT
    elif type == 'BMW_320i':
        cr_type = VehicleType.BMW_320i
    elif type == 'VW_VANAGON':
        cr_type = VehicleType.VW_VANAGON
    else:
        raise ValueError('Selected vehicle type is not valid: {}.'.format(type))
    return cr_type


def parse_cost_function(cost):
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


def call_trajectory_planner(queue, f, scenario, planning_problem_set):
    queue.put(f(scenario, planning_problem_set))


if __name__ == '__main__':
    with open('batch_processing_config.yaml', 'r') as stream:
        try:
            settings = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    sys.path.append(os.getcwd() + os.path.dirname(settings['trajectory_planner_path']))
    module = __import__(settings['trajectory_planner_module_name'])
    function = getattr(module, settings['trajectory_planner_function_name'])

    for filename in os.listdir(settings['input_path']):
        if not filename.endswith('.xml'):
            continue
        fullname = os.path.join(settings['input_path'], filename)

        print("Started processing scenario {}".format(filename))
        scenario, planning_problem_set = CommonRoadFileReader(fullname).open()

        queue = multiprocessing.Queue()
        p = multiprocessing.Process(target=call_trajectory_planner, name="trajectory_planner",
                                    args=(queue, function, scenario, planning_problem_set))
        p.start()
        p.join(timeout=settings['timeout'])

        if p.is_alive():
            print("Trajectory planner timeout.")
            p.terminate()
            p.join()
            solution_trajectories = {}
        else:
            solution_trajectories = queue.get()

        # create path for solutions
        pathlib.Path(settings['output_path']).mkdir(parents=True, exist_ok=True)

        if scenario.benchmark_id in settings.keys():
            vehicle_model = parse_vehicle_model(settings[scenario.benchmark_id]['vehicle_model'])
            vehicle_type = parse_vehicle_type(settings[scenario.benchmark_id]['vehicle_type'])
            cost_function = parse_cost_function(settings[scenario.benchmark_id]['cost_function'])
        else:
            vehicle_model = parse_vehicle_model(settings['default']['vehicle_model'])
            vehicle_type = parse_vehicle_type(settings['default']['vehicle_type'])
            cost_function = parse_cost_function(settings['default']['cost_function'])

        error = False
        cr_solution_writer = CommonRoadSolutionWriter(settings['output_path'], scenario.benchmark_id, scenario.dt,
                                                      vehicle_type, vehicle_model, cost_function)
        for planning_problem_id, planning_problem in planning_problem_set.planning_problem_dict.items():
            if planning_problem_id not in solution_trajectories.keys():
                warnings.warn('Solution for planning problem with ID={} is not provided for scenario {}.'.format(
                    planning_problem_id, filename))
                error = True
                break
            else:
                cr_solution_writer.add_solution_trajectory(
                    solution_trajectories[planning_problem_id], planning_problem_id)
        if not error:
            cr_solution_writer.write_to_file(overwrite=settings['overwrite'])
