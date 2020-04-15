def execute_search_batch(scenario, planning_problem_set, veh_type_id, 
						 planning_problem_idx, planner_id, max_tree_depth):

	# append main directory
	import sys
	import os

	path_commonroad_search = "../../"
	sys.path.append(os.path.join(path_commonroad_search, "GSMP/tools/"))
	sys.path.append(os.path.join(path_commonroad_search, "GSMP/tools/commonroad-collision-checker"))
	sys.path.append(os.path.join(path_commonroad_search, "GSMP/tools/commonroad-road-boundary"))
	sys.path.append(os.path.join(path_commonroad_search, "GSMP/motion_automata"))
	sys.path.append(os.path.join(path_commonroad_search, "GSMP/motion_automata/vehicle_model"))

	import time
	from multiprocessing import Manager, Process

	import numpy as np
	from math import atan2

	import matplotlib.pyplot as plt
	from IPython import display
	from ipywidgets import widgets

	# import CommonRoad-io modules
	from commonroad.visualization.draw_dispatch_cr import draw_object
	from commonroad.common.file_reader import CommonRoadFileReader
	from commonroad.scenario.trajectory import Trajectory, State
	from commonroad.geometry.shape import Rectangle
	from commonroad.prediction.prediction import TrajectoryPrediction
	from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_object
	from commonroad_cc.visualization.draw_dispatch import draw_object as draw_it
	from commonroad.common.solution_writer import CommonRoadSolutionWriter, VehicleType, CostFunction

	# import Motion Automata modules
	from automata.MotionAutomata import MotionAutomata
	from automata.MotionPrimitive import MotionPrimitive
	from automata.States import FinalState

	# load necessary modules and functions
	from automata.HelperFunctions import generate_automata, add_initial_state_to_automata

	# 1: Greedy BFS 2: A* 3: Your own algorithm
	if planner_id == 1:
		from automata.MotionPlanner_gbfs import MotionPlanner
	elif planner_id == 2:
		from automata.MotionPlanner_Astar import MotionPlanner
	else:
		from automata.MotionPlanner import MotionPlanner
	    
	automata = generate_automata(veh_type_id)

	# retrieve planning problem with given index (for cooperative scenario:0, 1, 2, ..., otherwise: 0)
	planning_problem = list(planning_problem_set.planning_problem_dict.values())[planning_problem_idx]
	planning_problem_id = list(planning_problem_set.planning_problem_dict.keys())[planning_problem_idx]

	# add initial state of planning problem to automata
	automata, initial_motion_primitive = add_initial_state_to_automata(automata, planning_problem, flag_print_states = False)

	# construct motion planner.
	motion_planner = MotionPlanner(scenario, planning_problem, automata)

	print("Planning..")
	time_start = time.process_time()
	result = motion_planner.search_alg(initial_motion_primitive.Successors, max_tree_depth)
	time_end = time.process_time()
	print("Execution time: {}".format(round(time_end - time_start, 2)))
	dict_result = {}

	# result is in form of (final path, used_primitives)
	if result is not None:
		result_path = result[0]

		list_state = list()

		for state in result_path:
			kwarg = {'position': state.position, 
			'velocity': state.velocity,
			'steering_angle': state.steering_angle, 
			'orientation': state.orientation, 
			'time_step': state.time_step}
			list_state.append(State(**kwarg))

		trajectory = Trajectory(initial_time_step=list_state[0].time_step, state_list=list_state)
		dict_result[planning_problem_id] = trajectory
		return dict_result
	else:
		return None