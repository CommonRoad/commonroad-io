# import helper functions
from helper_functions import *

# specify path to configuration file
path_file_config = "../configuration/batch_processing_config.yaml"

# load configuration file
configuration = load_configuration(path_file_config)

# get target function
function_target = get_target_function(configuration)

# get a list of scenario files
list_files_input = get_input_files(configuration)

# get length of the list and time before timeout
num_files = len(list_files_input)
time_timeout = configuration['timeout']

print("Total number of files to be processed: {}".format(num_files))
print("Timeout setting: {} seconds\n".format(time_timeout))
count_processed = 0

# iterate through scenarios
for file_scenario in list_files_input:
    count_processed += 1
    print("Processing file No. {} / {}. Scenario ID: {}".format(count_processed, num_files, file_scenario[:-4]))
    # parse the scenario file
    result_parse = parse_scenario_file(configuration, file_scenario)
    # execute target function
    solution_trajectories = execute_target_function(function_target, result_parse, time_timeout)
    # save solution file
    save_solution(configuration, solution_trajectories, result_parse)
    print("\n=========================================================\n")