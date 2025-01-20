# Changelog

## [2025.1] -
### Changed
- Update to new template-repository structure

## [2024.3] - 2024-12-20
### Added
- New Australia and US traffic signs
- New lanelet type: `restricted_area`
- `LaneletNetwork.create_from_lanelet_network` now also includes intersections
- Support for Python 3.13
- TR2 cost function enum value

### Fixed
- `DynamicObstacle` cannot be hashed if some optional attributes are missing
- `TrafficLightCycle` does not invalidate cached cycle init timesteps after modification

## [2024.2] - 2024-07-22

### Added
- Github actions for ubuntu, windows, and macOS
- Gitlab runner for arm64 ubuntu
- Support for reading xml and protobuf byte streams
- Support for numpy `>=2.0`
- Adjustable zorder for dynamic obstacle & lanelet visualization

### Fixed
- AreaBorder can have multiple adjacent lanelets
- Performance regression for occupancy_set lookups in TrajectoryPrediction
- Matplotlib `>=3.9.0` support

### Removed
- Support for Python 3.8

## [2024.1] - 2024-03-21

### Added
- Country-independent traffic sign enum
- Missing country-specific max speed sign IDs
- Automatically generated TrafficSignIDCountries enum for importing in other scripts
- GroundTruthPredictor class to use stored trajectories as prediction
- Function to append a state to a trajectory

### Fixed
- Typo: `TrafficSigInterpreter` → `TrafficSignInterpreter`
- Typo EMERGENCY_STOP traffic sign enum name
- Activation condition for drawing occupancies
- Traffic sign with first occurrence set to None can be hashed
- Traffic light can be plotted without a TrafficLightCycle

### Changed
- Optimization-based planner tutorial now uses planner and predictor interfaces
- Simplified traffic sign matching in FileReader
- The occupancy set, initial time step, and final time step are now computed properties of TrajectoryPrediction
- Trajectory now allows direct access to the state list
- Drawing occupancies by default false
- Improved visual appearance of notebooks

### Removed
- Setters for initial and final time step in predictions
- Setter for occupancy_set in TrajectoryPrediction

## [2023.4] - 2023-12-19

### Added
- Function to convert 3D scenarios to 2D scenarios
- Support for Python 3.12
- Function to retrieve all lanelets referencing a given traffic sign/light
- New line marking types
- Traffic light shape

### Fixed
- Conversion to initial state in function create_dynamic_obstacle of solution object

### Changed
- Traffic signs for Zamunda now use images of German traffic signs
- Code formatting (flake8, black, isort)

### Removed
- Images for Zamunda traffic signs
- Function commonorad.common.util.interpolate_angle

## [2023.3] - 2023-09-18

### Added
- Type information for lanelet init function
- Dynamic obstacles can now store a history of their states
- Function to update the initial state of a dynamic obstacle while storing the previous state in the history
- Function to update behavior predictions of dynamic obstacles
- Function to find lanelet predecessors in range to lanelet network
- Function to compute all predecessor lanelets starting from a provided lanelet and merge them to a single lanelet for each route.
- Documentation for renderers (including video creation)
- Abstract interfaces for motion planner and prediction for usage in other tools
- New ExtendedPMState to support states with position, velocity, orientation, and acceleration
- Orientation property to PMState
- Hash and equality functions for area

### Fixed
- Function create_from_lanelet_network deletes references to removed lanelets
- Write environment time to XML in correct format
- Failing visualization of lanelets, stop lines, traffic signs, and traffic lights with z-coordinate
- Traffic lights now correctly change size in interactive matplotlib plots (only affected matplotlib>=3.7)
- Considering state attributes not part of dataclass definition in state to state conversion
- Enforce InitialState class for initial state property of dynamic obstacle
- Hash function of obstacle

### Changed
- Cleanup lanelet, traffic sign, and traffic light references in function create_from_lanelet_list by default
- Equality checks of scenario elements no longer emit a warning on inequality (except if the elements are of different types)

### Removed
- Duplicated initial_state property of dynamic obstacle

## [2023.2] - 2023-06-26

### Added
- Area for modelling drivable areas which cannot be represented by lanelets
- New weather and time of day options
- Allow file reader to determine format based on suffix

### Fixed

- Visualization of all traffic signs by setting `show_traffic_signs = None` in draw parameters
- Validity functions to support z-axis
- Unreferenced traffic signs for lanelet networks filtered by lanelet type

### Changed

- Visualization of direction arrow of narrow lanelets
- Traffic light cycle optional
- Traffic light in separate python file
- Allow file reader to determine format based on suffix
- Broaden types allowed as file names
- Open files safely by using a context manager
- Use correct suffix when inferring filename from scenario id

### Removed

- function get_default_cycle for traffic lights
- support for Python 3.7

## [2023.1] - 2023-03-05

### Added
- Support for shapely>=2.0.0

### Fixed

- Writing scenarios without location to protobuf
- Dashed lanelet boundaries with fixed dash position
- Default plot limits for focused obstacle
- Use dt from scenario as default for video creation
- Apply axis visible-option also for video creation
- Protobuf FileReader marking road network related IDs as used
- State attribute comparison

### Changed

- Name of SIDEWALK and BUSLANE traffic signs to PEDESTRIAN_SIDEWALK and BUS_LANE
- Packaging and dependency management using poetry

## [2022.3] - 2022-11-16

### Added

- Drawing parameters as dataclasses (support for context help)
- Documentation for drawing parameters
- Support for Python 3.11
- Function to convert a state into a 1D-numpy array
- Progress bar for video creation
- Callback for modifying axes during plotting a video

### Changed

- Name of position in lateral and longitudinal state

### Removed

- ParamServer and dictionary specification of drawing parameters

### Fixed

- Failing visualization of a trajectory at it's initial time step
- Broken lanelet visualization with matplotlib >3.5

## [2022.2] - 2022-09-03

### Added

- Function for getting lanelet orientation closest to a given position
- Function for getting most likely lanelet given an obstacle state
- Function for erasing lanelet network from scenario
- Function for replacing lanelet network of a scenario with new one
- Support for Protobuf format
- Predefined classes for specific states, point-mass model, kinematic single-track model, etc.
- Function for computing shape group occupancy from state
- Support for kinematic single-track model with one on-axle trailer
- Three new lanelet types: parking, border, and restricted

### Changed

- Move tests, tutorial, and documentation folder to root directory
- State classes in separate Python file

### Removed
- setter method for lanelet network in scenario class

### Fixed

- Default constructor for ScenarioID produces invalid Benchmark ID
- Changeable state list leads to inconsistent final time step of trajectory
- Various small bug fixes


## [2022.1] - 2022-04-05

### Added

- Video creation with custom draw parameters
- Obstacle icon with custom color

### Changed

- Remove support for Python 3.6

### Fixed

- Side-effect circle and rectangle init functions
- Parsing solution files with old time format
- Invalid lanelet occupancy computation using buffered polygons
- Various small bug fixes

## [2021.4] - 2021-12-21

### Added

- Polyline utility functions, e.g., resampling, path length, orientation, curvature, intersection
- `__eq__` and `__hash__` functions for LaneletNetwork and related classes (e.g., traffic sign, traffic light, stop
  line, etc.)
- Compatibility for Shapely 2.0
- New traffic signs for Germany

### Changed

- License switched to BSD-3
- Date in solution file now stored in the dateTime format (`%Y-%m-%dT%H:%M:%S`)

### Fixed

- Various small bug fixes

## [2021.3] - 2021-09-21

### Added

- Spatial indexing via STRTree in LaneletNetwork for faster computation of lanelet queries via positions or shapes

### Changed
- The function LaneletNetwork.create_from_lanelet_network accepts now a shape
and set of lanelet types which should be excluded
- Shapely polygon for lanelets is created by default
- Function convert_to_polygon() within Lanelet class is deprecated
and will be removed in the next release

### Fixed
- Various small bug fixes


## [2021.2] - 2021-07-01
### Added
- Parameter `draw_params={"focus_obstacle_id": obstacle_id}` focuses the plot or video on a dynamic_obstacle
- About 150 new traffic signs from Germany, Spain, and US added
- added new cost function TR1 to SolutionWriter

### Fixed
- Various small bug fixes


## [2021.1] - 2021-04-23
### Added
- Support for over 50 new traffic signs from Germany
- Support of phantom obstacles
- New visualization module which supports MVC pattern
- Functions for adding and removing traffic signs, traffic lights, and intersections from a scenario or lanelet network
- New icon visualization interface and new icons for bus, truck, and bicycle (developed by Simon Sagmeister - TUM FTM)

### Changed
- Scaling of traffic signs is coupled with the zoom level

### Fixed
- Various small bug fixes


## [2020.3] - 2020-10-30
### Added
- Support of environment obstacles, e.g. buildings
- Several new traffic signs
- New ScenarioID class for the representation of benchmarks
- New line marking types *unknown* and *no_marking*
- Crossings for intersections

### Changed
The creation of lanelet assignments for obstacles is now optional. This decreases the loading time of scenarios.
The lanelet assignment can still be performed manually after loading a scenario.

### Fixed
- Docstrings for properties
- Autoscaling visualization when plotting trajectories.
- Function *generate_object_id* works now if no element has been added before and reserves ID if object
will be added later
- Runtime warning during calls to find_lanelet_by_position
- Various small bug fixes


## [2020.2] - 2020-04-14
### Changed
- Traffic signs are not visualized by default

### Added
- new tags for compatibility with 2018b XML format
- new lanelet types: unknown and interstate

### Fixed
- Various small bug fixes


## [2020.1] - 2020-04-08
### Added
- Support of 2020a Scenarios with TrafficLights, TrafficSigns, and Intersection objects
- SignalStates store information on turn lights, horn, hazard lights, and blue lights of DynamicObstacles
- New Solution object simplifies creating solution xml files and automatically checks the validity of benchmark IDs
- GeoTransformation object stores location and transformation information for lanelet networks extracted
from the real world

###Changed
- SolutionReader can now read solution files generated by the CommonRoadSolutionReader
- CommonRoadSolutionWriter can directly write computation times and the CPU model into the solution file
- Visualization is improved and now supports legends

### Fixed
- Various small bug fixes

### Removed
- The old commonroad.common.solution_writer.CommonRoadSolutionWriter is deprecated and replaced by
commonroad.common.solution.CommonRoadSolutionWriter


## [2019.2] - 2019-06-27
### Changed
- Increased decimal precision of file writer
when lanelets join into or split from other lanelets
- Minimum of 3 points per lanelet border are required

### Fixed
- File writer writes goal lanes correctly
- LaneletNetwork/find_lanelet_by_id returns None instead of an error if it's not contained in network


## [2019.1] - 2019-05-21
### Added
- Initial version
