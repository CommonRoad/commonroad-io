import os
import unittest

import numpy as np
import pytest

from commonroad import SCENARIO_VERSION
from commonroad.common.common_lanelet import (
    LaneletType,
    LineMarking,
    RoadUser,
    StopLine,
)
from commonroad.common.common_scenario import (
    GeoTransformation,
    Location,
    TimeOfDay,
    Underground,
    Weather,
)
from commonroad.common.file_reader import CommonRoadFileReader, combine_map_dynamic
from commonroad.common.util import FileFormat, Interval
from commonroad.geometry.shape import Circle, Polygon, Rectangle
from commonroad.planning.planning_problem import (
    GoalRegion,
    PlanningProblem,
    PlanningProblemSet,
)
from commonroad.prediction.prediction import (
    Occupancy,
    SetBasedPrediction,
    TrajectoryPrediction,
)
from commonroad.scenario.intersection import (
    CrossingGroup,
    IncomingGroup,
    Intersection,
    OutgoingGroup,
)
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
from commonroad.scenario.obstacle import (
    DynamicObstacle,
    EnvironmentObstacle,
    ObstacleType,
    PhantomObstacle,
    StaticObstacle,
)
from commonroad.scenario.scenario import Scenario, ScenarioID, Tag
from commonroad.scenario.state import (
    CustomState,
    InitialState,
    KSState,
    SignalState,
    STDState,
    STState,
)
from commonroad.scenario.traffic_light import (
    TrafficLight,
    TrafficLightCycle,
    TrafficLightCycleElement,
    TrafficLightDirection,
    TrafficLightState,
)
from commonroad.scenario.traffic_sign import (
    TrafficSign,
    TrafficSignElement,
    TrafficSignIDGermany,
)
from commonroad.scenario.trajectory import Trajectory


class TestXMLFileReader(unittest.TestCase):
    def setUp(self):
        self.cwd_path = os.path.dirname(os.path.abspath(__file__))
        self.filename_all = self.cwd_path + "/../test_scenarios/xml/2020a/ZAM_TestReadingAll-1_1_T-1.xml"
        self.filename_urban = (
            self.cwd_path + "/../test_scenarios/xml/2020a/ZAM_TestReadingIntersectionTrafficSign-1_1_T-1.xml"
        )
        self.filename_lanelets = self.cwd_path + "/../test_scenarios/xml/2020a/ZAM_TestReadingLanelets-1_1_T-1.xml"
        self.filename_obstacle = self.cwd_path + "/../test_scenarios/xml/2020a/ZAM_TestReadingObstacles-1_1_T-1.xml"
        self.filename_planning_problem = (
            self.cwd_path + "/../test_scenarios/xml/2018b/ZAM_TestReadingPlanningProblem-1_1_T-1.xml"
        )
        self.filename_2018b = self.cwd_path + "/../test_scenarios/xml/2018b/USA_Lanker-1_1_T-1.xml"

        # setup for reading obstacles, lanelets, planning problem and all (without intersection)
        rectangle = Rectangle(4.3, 8.9, center=np.array([0.1, 0.5]), orientation=1.7)
        polygon = Polygon(
            np.array([np.array((0.0, 0.0)), np.array((0.0, 1.0)), np.array((1.0, 1.0)), np.array((1.0, 0.0))])
        )
        circ = Circle(2.0, np.array([0.0, 0.0]))
        occupancy_list = list()
        occupancy_list.append(Occupancy(0, rectangle))
        occupancy_list.append(Occupancy(1, circ))
        occupancy_list.append(Occupancy(2, polygon))
        occupancy_list.append(Occupancy(3, circ))

        set_pred = SetBasedPrediction(0, occupancy_list)

        states = []
        state = CustomState()
        for attr, value in {"time_step": 1, "orientation": 0, "position": np.array([0, 1])}.items():
            state.add_attribute(attr)
            state.set_value(attr, value)
        states.append(state)
        trajectory = Trajectory(1, states)
        init_state = InitialState(time_step=0, orientation=0, position=np.array([0, 0]))
        traj_pred = TrajectoryPrediction(trajectory, rectangle)

        initial_signal_state = SignalState(time_step=0, horn=True, hazard_warning_lights=True, braking_lights=False)
        signal_series = [SignalState(time_step=1, horn=False, hazard_warning_lights=False, braking_lights=True)]

        static_obs = StaticObstacle(3, ObstacleType("unknown"), obstacle_shape=circ, initial_state=init_state)
        dyn_set_obs = DynamicObstacle(
            1, ObstacleType("unknown"), initial_state=init_state, prediction=set_pred, obstacle_shape=rectangle
        )
        dyn_traj_obs = DynamicObstacle(
            2,
            ObstacleType("unknown"),
            initial_state=init_state,
            prediction=traj_pred,
            obstacle_shape=rectangle,
            initial_signal_state=initial_signal_state,
            signal_series=signal_series,
        )

        environment_obstacle_shape = Polygon(np.array([[0, 0], [8, 0], [4, -4]]))
        environment_obstacle_id = 1234
        self._environment_obstacle = EnvironmentObstacle(
            environment_obstacle_id, ObstacleType.BUILDING, environment_obstacle_shape
        )

        phantom_obstacle_id = 5678
        self._phantom_obstacle = PhantomObstacle(phantom_obstacle_id, set_pred)

        lanelet1 = Lanelet(
            right_vertices=np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]),
            center_vertices=np.array([[0.0, 1], [1.0, 1], [2, 1]]),
            left_vertices=np.array([[0.0, 2], [1.0, 2], [2, 2]]),
            lanelet_id=100,
            predecessor=[101],
            successor=[101],
            adjacent_left=103,
            adjacent_left_same_direction=True,
            line_marking_left_vertices=LineMarking.DASHED,
            line_marking_right_vertices=LineMarking.SOLID,
            lanelet_type={LaneletType.HIGHWAY, LaneletType.MAIN_CARRIAGE_WAY},
            user_one_way={RoadUser.VEHICLE},
            traffic_signs={201},
        )
        lanelet2 = Lanelet(
            right_vertices=np.array([[2.0, 0.0], [3.0, 0.0], [4, 0]]),
            center_vertices=np.array([[2.0, 1], [3.0, 1], [4, 1]]),
            left_vertices=np.array([[2.0, 2], [3.0, 2], [4, 2]]),
            lanelet_id=101,
            predecessor=[100],
            successor=[101],
            adjacent_left=104,
            adjacent_left_same_direction=True,
            line_marking_left_vertices=LineMarking.BROAD_DASHED,
            line_marking_right_vertices=LineMarking.BROAD_SOLID,
            lanelet_type={LaneletType.URBAN, LaneletType.BUS_LANE},
            user_bidirectional={RoadUser.BUS},
        )
        lanelet3 = Lanelet(
            right_vertices=np.array([[4.0, 0.0], [5.0, 0.0], [6, 0]]),
            center_vertices=np.array([[4.0, 1], [5.0, 1], [6, 1]]),
            left_vertices=np.array([[4.0, 2], [5.0, 2], [6, 2]]),
            lanelet_id=102,
            predecessor=[102],
            adjacent_left=105,
            adjacent_left_same_direction=True,
            line_marking_left_vertices=LineMarking.BROAD_DASHED,
            line_marking_right_vertices=LineMarking.BROAD_SOLID,
            lanelet_type={LaneletType.URBAN, LaneletType.BUS_LANE},
            user_bidirectional={RoadUser.BUS},
        )
        lanelet4 = Lanelet(
            right_vertices=np.array([[0.0, 2.0], [1.0, 2.0], [2, 2]]),
            center_vertices=np.array([[0.0, 3], [1.0, 3], [2, 3]]),
            left_vertices=np.array([[0.0, 4], [1.0, 4], [2, 4]]),
            lanelet_id=103,
            successor=[104],
            adjacent_right=100,
            adjacent_right_same_direction=True,
            line_marking_left_vertices=LineMarking.BROAD_DASHED,
            line_marking_right_vertices=LineMarking.BROAD_SOLID,
            lanelet_type={LaneletType.URBAN, LaneletType.BUS_LANE},
            user_bidirectional={RoadUser.BUS},
        )
        lanelet5 = Lanelet(
            right_vertices=np.array([[2.0, 2.0], [3.0, 2.0], [4, 2]]),
            center_vertices=np.array([[2.0, 3], [3.0, 3], [4, 3]]),
            left_vertices=np.array([[2.0, 4], [3.0, 4], [4, 4]]),
            lanelet_id=104,
            predecessor=[103],
            successor=[105],
            adjacent_right=101,
            adjacent_right_same_direction=True,
            line_marking_left_vertices=LineMarking.BROAD_DASHED,
            line_marking_right_vertices=LineMarking.BROAD_SOLID,
            lanelet_type={LaneletType.URBAN, LaneletType.BUS_LANE},
            user_bidirectional={RoadUser.BUS},
        )
        lanelet6 = Lanelet(
            right_vertices=np.array([[4.0, 0.0], [5.0, 0.0], [6, 0]]),
            center_vertices=np.array([[4.0, 1], [5.0, 1], [6, 1]]),
            left_vertices=np.array([[4.0, 2], [5.0, 2], [6, 2]]),
            lanelet_id=105,
            predecessor=[104],
            adjacent_right=102,
            adjacent_right_same_direction=True,
            line_marking_left_vertices=LineMarking.BROAD_DASHED,
            line_marking_right_vertices=LineMarking.BROAD_SOLID,
            lanelet_type={LaneletType.URBAN, LaneletType.BUS_LANE},
            user_bidirectional={RoadUser.BUS},
        )

        traffic_sign_201 = TrafficSign(
            traffic_sign_id=201,
            first_occurrence={100, 103},
            position=np.array([0.0, 0.0]),
            traffic_sign_elements=[TrafficSignElement(TrafficSignIDGermany.MAX_SPEED, ["10"])],
            virtual=False,
        )

        self.lanelet_network = LaneletNetwork().create_from_lanelet_list(
            list([lanelet1, lanelet2, lanelet3, lanelet4, lanelet5, lanelet6])
        )
        self.lanelet_network.add_traffic_sign(traffic_sign_201, [100])

        tags = {Tag.URBAN, Tag.INTERSTATE}
        geo_transformation = GeoTransformation("test", 0.0, 0.0, 0.0, 0.0)
        location = Location(2867714, 0.0, 0.0, geo_transformation)

        self.scenario = Scenario(
            0.1,
            ScenarioID.from_benchmark_id("ZAM_TestReadingAll-1_1_T-1", scenario_version=SCENARIO_VERSION),
            tags=tags,
        )
        self.lanelet_network.location = location
        self.scenario.add_objects(
            [
                static_obs,
                dyn_set_obs,
                dyn_traj_obs,
                self.lanelet_network,
                self._environment_obstacle,
                self._phantom_obstacle,
            ]
        )

        goal_region = GoalRegion(
            [
                STState(time_step=Interval(0, 1), velocity=Interval(0.0, 1), position=rectangle),
                STState(time_step=Interval(1, 2), velocity=Interval(0.0, 1), position=circ),
            ],
            {0: [101, 102], 1: list([102])},
        )
        planning_problem = PlanningProblem(
            1000,
            InitialState(
                velocity=0.1, position=np.array([[0], [0]]), orientation=0, yaw_rate=0, slip_angle=0, time_step=0
            ),
            goal_region,
        )
        self.planning_problem_set = PlanningProblemSet(list([planning_problem]))

        # setup for reading intersection scenario with traffic signs, traffic lights, stop signs (without obstacles)
        self.stop_line_17 = StopLine(
            np.array([169.2560351117039, -54.95658983061205]),
            np.array([168.6607857447963, -57.38341449560771]),
            LineMarking.SOLID,
            None,
            {204},
        )
        self.stop_line_13 = StopLine(
            np.array([174.1617095787515, -64.10832609867704]),
            np.array([176.4678542468245, -65.07388839655903]),
            LineMarking.SOLID,
            None,
            {201},
        )
        self.stop_line_12 = None
        self.lanelet_12_traffic_sign_ref = {112}
        self.lanelet_28_traffic_sign_ref = {105}
        self.lanelet_13_traffic_lights_ref = {201}
        self.traffic_sign_101 = TrafficSign(
            traffic_sign_id=101,
            position=np.array([206.9839751212892, 20.67847944866278]),
            first_occurrence={14},
            traffic_sign_elements=[TrafficSignElement(TrafficSignIDGermany.TOWN_SIGN, ["Landeshauptstadt München"])],
            virtual=False,
        )
        self.traffic_sign_105 = TrafficSign(
            traffic_sign_id=105,
            position=np.array([177.8639861239823, -48.79316329157203]),
            first_occurrence={14},
            traffic_sign_elements=[
                TrafficSignElement(TrafficSignIDGermany.PRIORITY, []),
                TrafficSignElement(TrafficSignIDGermany.GREEN_ARROW, []),
            ],
            virtual=False,
        )

        self.traffic_light_201 = TrafficLight(
            traffic_light_id=201,
            position=np.array([168.6607857447963, -57.38341449560771]),
            direction=TrafficLightDirection.ALL,
            active=True,
            traffic_light_cycle=TrafficLightCycle(
                [
                    TrafficLightCycleElement(state=TrafficLightState.RED, duration=15),
                    TrafficLightCycleElement(state=TrafficLightState.INACTIVE, duration=4),
                    TrafficLightCycleElement(state=TrafficLightState.RED_YELLOW, duration=4),
                    TrafficLightCycleElement(state=TrafficLightState.GREEN, duration=15),
                    TrafficLightCycleElement(state=TrafficLightState.YELLOW, duration=4),
                ],
                time_offset=2,
            ),
        )
        self.intersection_301 = Intersection(
            intersection_id=301,
            incomings=[
                IncomingGroup(
                    incoming_id=302,
                    incoming_lanelets={13},
                    outgoing_group_id=1,
                    outgoing_right={26},
                    outgoing_straight={22},
                    outgoing_left={20},
                ),
                IncomingGroup(
                    incoming_id=303,
                    incoming_lanelets={14},
                    outgoing_group_id=2,
                    outgoing_right={30},
                    outgoing_straight={24},
                    outgoing_left={28},
                ),
                IncomingGroup(
                    incoming_id=304,
                    incoming_lanelets={17},
                    outgoing_group_id=3,
                    outgoing_right={27},
                    outgoing_straight={23},
                    outgoing_left={31},
                ),
                IncomingGroup(
                    incoming_id=305,
                    incoming_lanelets={18},
                    outgoing_group_id=4,
                    outgoing_right={29},
                    outgoing_straight={21},
                    outgoing_left={25},
                ),
            ],
            outgoings=[OutgoingGroup(1, {1}), OutgoingGroup(2, {2}), OutgoingGroup(3, {3}), OutgoingGroup(4, {4})],
            crossings=[CrossingGroup(401, {32}, 302, 1)],
        )

    def test_open_2018b(self):
        scenario, planning_problem_set = CommonRoadFileReader(self.filename_2018b).open()
        exp_location = Location()
        exp_location_geo_name_id = exp_location.geo_name_id
        exp_location_gps_long = exp_location.gps_longitude
        exp_location_gps_lat = exp_location.gps_latitude
        exp_tags = {
            Tag.URBAN,
            Tag.SPEED_LIMIT,
            Tag.ONCOMING_TRAFFIC,
            Tag.MULTI_LANE,
            Tag.INTERSECTION,
            Tag.COMFORT,
            Tag.LANE_FOLLOWING,
        }
        exp_num_lanelets = 91
        exp_num_dynamic_obstacles = 24
        exp_num_static_obstacles = 0
        exp_num_traffic_signs = 91
        exp_num_traffic_lights = 0
        exp_num_intersections = 0
        self.assertSetEqual(exp_tags, scenario.tags)
        self.assertEqual(exp_location_geo_name_id, scenario.lanelet_network.location.geo_name_id)
        self.assertEqual(exp_location_gps_long, scenario.lanelet_network.location.gps_longitude)
        self.assertEqual(exp_location_gps_lat, scenario.lanelet_network.location.gps_latitude)
        self.assertEqual(exp_num_lanelets, len(scenario.lanelet_network.lanelets))
        self.assertEqual(exp_num_dynamic_obstacles, len(scenario.dynamic_obstacles))
        self.assertEqual(exp_num_static_obstacles, len(scenario.static_obstacles))
        self.assertEqual(exp_num_traffic_signs, len(scenario.lanelet_network.traffic_signs))
        self.assertEqual(exp_num_traffic_lights, len(scenario.lanelet_network.traffic_lights))
        self.assertEqual(exp_num_intersections, len(scenario.lanelet_network.intersections))

    def test_open_lanelets(self):
        lanelets = CommonRoadFileReader(self.filename_lanelets).open()

        exp_lanelet_zero_center_vertices = self.scenario.lanelet_network.lanelets[0].center_vertices
        exp_lanelet_zero_left_vertices = self.scenario.lanelet_network.lanelets[0].left_vertices
        exp_lanelet_zero_right_vertices = self.scenario.lanelet_network.lanelets[0].right_vertices
        exp_lanelet_zero_adj_left = self.scenario.lanelet_network.lanelets[0].adj_left
        exp_lanelet_zero_adj_right = self.scenario.lanelet_network.lanelets[0].adj_right
        exp_lanelet_zero_adj_left_same_direction = self.scenario.lanelet_network.lanelets[0].adj_left_same_direction
        exp_lanelet_zero_adj_right_same_direction = self.scenario.lanelet_network.lanelets[0].adj_right_same_direction
        exp_lanelet_one_predecessor = self.scenario.lanelet_network.lanelets[1].predecessor
        exp_lanelet_zero_successor = self.scenario.lanelet_network.lanelets[0].successor

        np.testing.assert_array_equal(
            exp_lanelet_zero_center_vertices, lanelets[0].lanelet_network.lanelets[0].center_vertices
        )
        np.testing.assert_array_equal(
            exp_lanelet_zero_left_vertices, lanelets[0].lanelet_network.lanelets[0].left_vertices
        )
        np.testing.assert_array_equal(
            exp_lanelet_zero_right_vertices, lanelets[0].lanelet_network.lanelets[0].right_vertices
        )
        self.assertEqual(exp_lanelet_zero_adj_left, lanelets[0].lanelet_network.lanelets[0].adj_left)
        self.assertEqual(exp_lanelet_zero_adj_right, lanelets[0].lanelet_network.lanelets[0].adj_right)
        self.assertEqual(
            exp_lanelet_zero_adj_left_same_direction, lanelets[0].lanelet_network.lanelets[0].adj_left_same_direction
        )
        self.assertEqual(
            exp_lanelet_zero_adj_right_same_direction, lanelets[0].lanelet_network.lanelets[0].adj_right_same_direction
        )
        self.assertEqual(exp_lanelet_one_predecessor, lanelets[0].lanelet_network.lanelets[1].predecessor)
        self.assertEqual(exp_lanelet_zero_successor, lanelets[0].lanelet_network.lanelets[0].successor)

    def test_open_obstacles(self):
        obstacles = CommonRoadFileReader(self.filename_obstacle).open()

        exp_obstacle_zero_id = self.scenario.obstacles[0].obstacle_id
        exp_obstacle_zero_type = self.scenario.obstacles[0].obstacle_type
        exp_obstacle_zero_role = self.scenario.obstacles[0].obstacle_role
        exp_obstacle_zero_shape = self.scenario.obstacles[0].obstacle_shape.__class__
        exp_obstacle_zero_radius = self.scenario.obstacles[0].obstacle_shape.radius
        exp_obstacle_zero_center = self.scenario.obstacles[0].obstacle_shape.center

        exp_obstacle_one_id = self.scenario.obstacles[1].obstacle_id
        exp_obstacle_one_type = self.scenario.obstacles[1].obstacle_type
        exp_obstacle_one_role = self.scenario.obstacles[1].obstacle_role
        exp_obstacle_one_shape = self.scenario.obstacles[1].obstacle_shape.__class__
        exp_obstacle_one_attributes = len(self.scenario.obstacles[1].initial_state.attributes)
        exp_obstacle_one_orientation = self.scenario.obstacles[1].initial_state.orientation
        exp_obstacle_one_prediction_zero_shape_center = (
            self.scenario.obstacles[1].prediction.occupancy_set[0].shape.center
        )

        exp_obstacle_two_id = self.scenario.obstacles[2].obstacle_id
        exp_obstacle_two_type = self.scenario.obstacles[2].obstacle_type
        exp_obstacle_two_role = self.scenario.obstacles[2].obstacle_role
        exp_obstacle_two_shape = self.scenario.obstacles[2].obstacle_shape.__class__
        exp_obstacle_two_shape_prediction_state_list_len = len(
            self.scenario.obstacles[2].prediction.trajectory.state_list
        )
        exp_obstacle_two_shape_prediction_final_state_attributes_len = len(
            self.scenario.obstacles[2].prediction.trajectory.final_state.attributes
        )

        self.assertEqual(exp_obstacle_zero_id, obstacles[0].obstacles[0].obstacle_id)
        self.assertEqual(exp_obstacle_zero_type, obstacles[0].obstacles[0].obstacle_type)
        self.assertEqual(exp_obstacle_zero_role, obstacles[0].obstacles[0].obstacle_role)
        self.assertEqual(exp_obstacle_zero_shape, obstacles[0].obstacles[0].obstacle_shape.__class__)
        self.assertEqual(exp_obstacle_zero_radius, obstacles[0].obstacles[0].obstacle_shape.radius)
        self.assertEqual(exp_obstacle_zero_center[0], obstacles[0].obstacles[0].obstacle_shape.center[0])
        self.assertEqual(exp_obstacle_zero_center[1], obstacles[0].obstacles[0].obstacle_shape.center[1])

        self.assertEqual(exp_obstacle_one_id, obstacles[0].obstacles[1].obstacle_id)
        self.assertEqual(exp_obstacle_one_type, obstacles[0].obstacles[1].obstacle_type)
        self.assertEqual(exp_obstacle_one_role, obstacles[0].obstacles[1].obstacle_role)
        self.assertEqual(exp_obstacle_one_shape, obstacles[0].obstacles[1].obstacle_shape.__class__)
        self.assertEqual(exp_obstacle_one_attributes, len(obstacles[0].obstacles[1].initial_state.attributes))
        self.assertEqual(exp_obstacle_one_orientation, obstacles[0].obstacles[1].initial_state.orientation)
        self.assertEqual(
            exp_obstacle_one_prediction_zero_shape_center[0],
            obstacles[0].obstacles[1].prediction.occupancy_set[0].shape.center[0],
        )
        self.assertEqual(
            exp_obstacle_one_prediction_zero_shape_center[1],
            obstacles[0].obstacles[1].prediction.occupancy_set[0].shape.center[1],
        )

        self.assertEqual(exp_obstacle_two_id, obstacles[0].obstacles[2].obstacle_id)
        self.assertEqual(exp_obstacle_two_type, obstacles[0].obstacles[2].obstacle_type)
        self.assertEqual(exp_obstacle_two_role, obstacles[0].obstacles[2].obstacle_role)
        self.assertEqual(exp_obstacle_two_shape, obstacles[0].obstacles[2].obstacle_shape.__class__)
        self.assertEqual(
            exp_obstacle_two_shape_prediction_state_list_len,
            len(obstacles[0].obstacles[2].prediction.trajectory.state_list),
        )
        self.assertEqual(
            exp_obstacle_two_shape_prediction_final_state_attributes_len,
            len(obstacles[0].obstacles[2].prediction.trajectory.final_state.attributes),
        )

    def test_open_planning_problem(self):
        planning_problem = CommonRoadFileReader(self.filename_planning_problem).open()

        exp_planning_problem_id = self.planning_problem_set.planning_problem_dict[1000].planning_problem_id
        exp_planning_problem_initial_state_velocity = self.planning_problem_set.planning_problem_dict[
            1000
        ].initial_state.velocity
        exp_planning_problem_initial_state_slip_angle = self.planning_problem_set.planning_problem_dict[
            1000
        ].initial_state.slip_angle
        exp_planning_problem_initial_state_yaw_rate = self.planning_problem_set.planning_problem_dict[
            1000
        ].initial_state.yaw_rate

        self.assertEqual(exp_planning_problem_id, planning_problem[1].planning_problem_dict[1000].planning_problem_id)
        self.assertEqual(
            exp_planning_problem_initial_state_velocity,
            planning_problem[1].planning_problem_dict[1000].initial_state.velocity,
        )
        self.assertEqual(
            exp_planning_problem_initial_state_slip_angle,
            planning_problem[1].planning_problem_dict[1000].initial_state.slip_angle,
        )
        self.assertEqual(
            exp_planning_problem_initial_state_yaw_rate,
            planning_problem[1].planning_problem_dict[1000].initial_state.yaw_rate,
        )

    def test_open_all(self):
        exp_num_lanelet_scenario = len(self.scenario.lanelet_network.lanelets)
        exp_num_obstacles_scenario = len(self.scenario.obstacles)
        exp_num_planning_problems = len(self.planning_problem_set.planning_problem_dict)
        exp_scenario_id = self.scenario.scenario_id
        exp_dt = self.scenario.dt

        exp_obstacle_zero_id = self.scenario.obstacles[0].obstacle_id
        exp_obstacle_zero_type = self.scenario.obstacles[0].obstacle_type
        exp_obstacle_zero_role = self.scenario.obstacles[0].obstacle_role
        exp_obstacle_zero_shape = self.scenario.obstacles[0].obstacle_shape.__class__
        exp_obstacle_zero_radius = self.scenario.obstacles[0].obstacle_shape.radius
        exp_obstacle_zero_center = self.scenario.obstacles[0].obstacle_shape.center

        exp_obstacle_one_id = self.scenario.obstacles[1].obstacle_id
        exp_obstacle_one_type = self.scenario.obstacles[1].obstacle_type
        exp_obstacle_one_role = self.scenario.obstacles[1].obstacle_role
        exp_obstacle_one_shape = self.scenario.obstacles[1].obstacle_shape.__class__
        exp_obstacle_one_attributes = len(self.scenario.obstacles[1].initial_state.attributes)
        exp_obstacle_one_orientation = self.scenario.obstacles[1].initial_state.orientation
        exp_obstacle_one_prediction_zero_shape_center = (
            self.scenario.obstacles[1].prediction.occupancy_set[0].shape.center
        )

        exp_obstacle_two_id = self.scenario.obstacles[2].obstacle_id
        exp_obstacle_two_type = self.scenario.obstacles[2].obstacle_type
        exp_obstacle_two_role = self.scenario.obstacles[2].obstacle_role
        exp_obstacle_two_shape = self.scenario.obstacles[2].obstacle_shape.__class__
        exp_obstacle_two_shape_prediction_state_list_len = len(
            self.scenario.obstacles[2].prediction.trajectory.state_list
        )
        exp_obstacle_two_shape_prediction_final_state_attributes_len = len(
            self.scenario.obstacles[2].prediction.trajectory.final_state.attributes
        )
        exp_obstacle_two_initial_signal_state_horn = self.scenario.obstacles[2].initial_signal_state.horn
        exp_obstacle_two_initial_signal_state_braking_lights = self.scenario.obstacles[
            2
        ].initial_signal_state.braking_lights
        exp_obstacle_two_initial_signal_state_hazard_warning_lights = self.scenario.obstacles[
            2
        ].initial_signal_state.hazard_warning_lights
        exp_obstacle_two_signal_state_time_step_1_horn = self.scenario.obstacles[2].signal_state_at_time_step(1).horn
        exp_obstacle_two_initial_signal_state_time_step_1_braking_lights = (
            self.scenario.obstacles[2].signal_state_at_time_step(1).braking_lights
        )
        exp_obstacle_two_initial_signal_state_time_step_1_hazard_warning_lights = (
            self.scenario.obstacles[2].signal_state_at_time_step(1).hazard_warning_lights
        )

        exp_lanelet_zero_center_vertices = self.scenario.lanelet_network.lanelets[0].center_vertices
        exp_lanelet_zero_left_vertices = self.scenario.lanelet_network.lanelets[0].left_vertices
        exp_lanelet_zero_right_vertices = self.scenario.lanelet_network.lanelets[0].right_vertices
        exp_lanelet_zero_adj_left = self.scenario.lanelet_network.lanelets[0].adj_left
        exp_lanelet_zero_adj_right = self.scenario.lanelet_network.lanelets[0].adj_right
        exp_lanelet_zero_adj_left_same_direction = self.scenario.lanelet_network.lanelets[0].adj_left_same_direction
        exp_lanelet_zero_adj_right_same_direction = self.scenario.lanelet_network.lanelets[0].adj_right_same_direction
        exp_lanelet_zero_succesor = self.scenario.lanelet_network.lanelets[0].successor
        exp_lanelet_zero_line_marking_left = self.lanelet_network.lanelets[0].line_marking_left_vertices
        exp_lanelet_zero_line_marking_right = self.lanelet_network.lanelets[0].line_marking_right_vertices
        exp_lanelet_one_predecessor = self.scenario.lanelet_network.lanelets[1].predecessor
        exp_lanelet_one_line_marking_left = self.lanelet_network.lanelets[1].line_marking_left_vertices
        exp_lanelet_one_line_marking_right = self.lanelet_network.lanelets[1].line_marking_right_vertices
        exp_lanelet_zero_type = self.lanelet_network.lanelets[0].lanelet_type
        exp_lanelet_one_type = self.lanelet_network.lanelets[1].lanelet_type
        exp_lanelet_zero_user_one_way = self.lanelet_network.lanelets[0].user_one_way
        exp_lanelet_one_user_bidirectional = self.lanelet_network.lanelets[1].user_bidirectional
        exp_lanelet_zero_traffic_signs = self.lanelet_network.lanelets[0].traffic_signs

        exp_static_obstacles_on_lanelet_zero = {self.scenario.static_obstacles[0].obstacle_id}

        exp_dynamic_obstacles_on_lanelet_zero = {
            0: {self.scenario.dynamic_obstacles[1].obstacle_id},
            1: {self.scenario.dynamic_obstacles[1].obstacle_id},
        }
        exp_dynamic_obstacles_on_lanelet_one = {
            0: {self.scenario.dynamic_obstacles[1].obstacle_id},
            1: {self.scenario.dynamic_obstacles[1].obstacle_id},
        }
        exp_lanelet_of_static_obstacle = {100}
        exp_lanelet_of_dynamic_obstacle_initial = {100, 101, 103, 104}
        exp_lanelet_of_dynamic_obstacle_prediction = {0: {100, 101, 103, 104}, 1: {100, 101, 103, 104}}

        exp_traffic_sign_id = self.lanelet_network.traffic_signs[0].traffic_sign_id
        exp_traffic_sign_position = self.lanelet_network.traffic_signs[0].position
        exp_traffic_sign_element_id = (
            self.lanelet_network.traffic_signs[0].traffic_sign_elements[0].traffic_sign_element_id.value
        )
        exp_traffic_sign_element_additional_value = (
            self.lanelet_network.traffic_signs[0].traffic_sign_elements[0].additional_values[0]
        )
        exp_traffic_sign_virtual = self.lanelet_network.traffic_signs[0].virtual

        exp_planning_problem_id = self.planning_problem_set.planning_problem_dict[1000].planning_problem_id
        exp_planning_problem_initial_state_velocity = self.planning_problem_set.planning_problem_dict[
            1000
        ].initial_state.velocity
        exp_planning_problem_initial_state_slip_angle = self.planning_problem_set.planning_problem_dict[
            1000
        ].initial_state.slip_angle
        exp_planning_problem_initial_state_yaw_rate = self.planning_problem_set.planning_problem_dict[
            1000
        ].initial_state.yaw_rate

        exp_location_geo_name_id = 2867714
        exp_location_latitude = 48.262333
        exp_location_longitude = 11.668775
        exp_location_geo = None
        exp_location_env_time_hours = 9
        exp_location_env_time_minutes = 12
        exp_location_env_underground = Underground.ICE
        exp_location_env_time_of_day = TimeOfDay.NIGHT
        exp_location_env_weather = Weather.LIGHT_RAIN
        exp_tags = {Tag.INTERSECTION, Tag.URBAN}

        xml_file = CommonRoadFileReader(self.filename_all).open(lanelet_assignment=True)

        self.assertEqual(exp_num_lanelet_scenario, len(xml_file[0].lanelet_network.lanelets))
        self.assertEqual(exp_num_obstacles_scenario, len(xml_file[0].obstacles))
        self.assertEqual(exp_num_planning_problems, len(xml_file[1].planning_problem_dict))
        self.assertEqual(exp_scenario_id, xml_file[0].scenario_id)
        self.assertEqual(exp_dt, xml_file[0].dt)

        self.assertEqual(exp_obstacle_zero_id, xml_file[0].obstacles[0].obstacle_id)
        self.assertEqual(exp_obstacle_zero_type, xml_file[0].obstacles[0].obstacle_type)
        self.assertEqual(exp_obstacle_zero_role, xml_file[0].obstacles[0].obstacle_role)
        self.assertEqual(exp_obstacle_zero_shape, xml_file[0].obstacles[0].obstacle_shape.__class__)
        self.assertEqual(exp_obstacle_zero_radius, xml_file[0].obstacles[0].obstacle_shape.radius)
        self.assertEqual(exp_obstacle_zero_center[0], xml_file[0].obstacles[0].obstacle_shape.center[0])
        self.assertEqual(exp_obstacle_zero_center[1], xml_file[0].obstacles[0].obstacle_shape.center[1])

        self.assertEqual(exp_obstacle_one_id, xml_file[0].obstacles[1].obstacle_id)
        self.assertEqual(exp_obstacle_one_type, xml_file[0].obstacles[1].obstacle_type)
        self.assertEqual(exp_obstacle_one_role, xml_file[0].obstacles[1].obstacle_role)
        self.assertEqual(exp_obstacle_one_shape, xml_file[0].obstacles[1].obstacle_shape.__class__)
        self.assertEqual(exp_obstacle_one_attributes, len(xml_file[0].obstacles[1].initial_state.attributes))
        self.assertEqual(exp_obstacle_one_orientation, xml_file[0].obstacles[1].initial_state.orientation)
        self.assertEqual(
            exp_obstacle_one_prediction_zero_shape_center[0],
            xml_file[0].obstacles[1].prediction.occupancy_set[0].shape.center[0],
        )
        self.assertEqual(
            exp_obstacle_one_prediction_zero_shape_center[1],
            xml_file[0].obstacles[1].prediction.occupancy_set[0].shape.center[1],
        )

        self.assertEqual(exp_obstacle_two_id, xml_file[0].obstacles[2].obstacle_id)
        self.assertEqual(exp_obstacle_two_type, xml_file[0].obstacles[2].obstacle_type)
        self.assertEqual(exp_obstacle_two_role, xml_file[0].obstacles[2].obstacle_role)
        self.assertEqual(exp_obstacle_two_shape, xml_file[0].obstacles[2].obstacle_shape.__class__)
        self.assertEqual(
            exp_obstacle_two_shape_prediction_state_list_len,
            len(xml_file[0].obstacles[2].prediction.trajectory.state_list),
        )
        self.assertEqual(
            exp_obstacle_two_shape_prediction_final_state_attributes_len,
            len(xml_file[0].obstacles[2].prediction.trajectory.final_state.attributes),
        )
        self.assertEqual(exp_obstacle_two_initial_signal_state_horn, xml_file[0].obstacles[2].initial_signal_state.horn)
        self.assertEqual(
            exp_obstacle_two_initial_signal_state_braking_lights,
            xml_file[0].obstacles[2].initial_signal_state.braking_lights,
        )
        self.assertEqual(
            exp_obstacle_two_initial_signal_state_hazard_warning_lights,
            xml_file[0].obstacles[2].initial_signal_state.hazard_warning_lights,
        )
        self.assertEqual(
            exp_obstacle_two_signal_state_time_step_1_horn, xml_file[0].obstacles[2].signal_state_at_time_step(1).horn
        )
        self.assertEqual(
            exp_obstacle_two_initial_signal_state_time_step_1_braking_lights,
            xml_file[0].obstacles[2].signal_state_at_time_step(1).braking_lights,
        )
        self.assertEqual(
            exp_obstacle_two_initial_signal_state_time_step_1_hazard_warning_lights,
            xml_file[0].obstacles[2].signal_state_at_time_step(1).hazard_warning_lights,
        )

        np.testing.assert_array_equal(
            exp_lanelet_zero_center_vertices, xml_file[0].lanelet_network.lanelets[0].center_vertices
        )
        np.testing.assert_array_equal(
            exp_lanelet_zero_left_vertices, xml_file[0].lanelet_network.lanelets[0].left_vertices
        )
        np.testing.assert_array_equal(
            exp_lanelet_zero_right_vertices, xml_file[0].lanelet_network.lanelets[0].right_vertices
        )
        self.assertEqual(exp_lanelet_zero_adj_left, xml_file[0].lanelet_network.lanelets[0].adj_left)
        self.assertEqual(exp_lanelet_zero_adj_right, xml_file[0].lanelet_network.lanelets[0].adj_right)
        self.assertEqual(
            exp_lanelet_zero_adj_left_same_direction, xml_file[0].lanelet_network.lanelets[0].adj_left_same_direction
        )
        self.assertEqual(
            exp_lanelet_zero_adj_right_same_direction, xml_file[0].lanelet_network.lanelets[0].adj_right_same_direction
        )
        self.assertEqual(exp_lanelet_zero_succesor, xml_file[0].lanelet_network.lanelets[0].successor)
        self.assertEqual(
            exp_lanelet_zero_line_marking_left, xml_file[0].lanelet_network.lanelets[0].line_marking_left_vertices
        )
        self.assertEqual(
            exp_lanelet_zero_line_marking_right, xml_file[0].lanelet_network.lanelets[0].line_marking_right_vertices
        )
        self.assertEqual(exp_lanelet_one_predecessor, xml_file[0].lanelet_network.lanelets[1].predecessor)
        self.assertEqual(
            exp_lanelet_one_line_marking_left, xml_file[0].lanelet_network.lanelets[1].line_marking_left_vertices
        )
        self.assertEqual(
            exp_lanelet_one_line_marking_right, xml_file[0].lanelet_network.lanelets[1].line_marking_right_vertices
        )
        self.assertEqual(exp_lanelet_zero_type, xml_file[0].lanelet_network.lanelets[0].lanelet_type)
        self.assertEqual(exp_lanelet_one_type, xml_file[0].lanelet_network.lanelets[1].lanelet_type)
        self.assertEqual(exp_lanelet_zero_user_one_way, xml_file[0].lanelet_network.lanelets[0].user_one_way)
        self.assertEqual(exp_lanelet_one_user_bidirectional, xml_file[0].lanelet_network.lanelets[1].user_bidirectional)
        self.assertSetEqual(exp_lanelet_zero_traffic_signs, xml_file[0].lanelet_network.lanelets[0].traffic_signs)

        self.assertSetEqual(
            exp_static_obstacles_on_lanelet_zero, xml_file[0].lanelet_network.lanelets[0].static_obstacles_on_lanelet
        )
        self.assertEqual(
            exp_dynamic_obstacles_on_lanelet_zero, xml_file[0].lanelet_network.lanelets[0].dynamic_obstacles_on_lanelet
        )
        self.assertEqual(
            exp_dynamic_obstacles_on_lanelet_one, xml_file[0].lanelet_network.lanelets[1].dynamic_obstacles_on_lanelet
        )
        self.assertSetEqual(exp_lanelet_of_static_obstacle, xml_file[0].obstacle_by_id(3).initial_shape_lanelet_ids)
        self.assertSetEqual(
            exp_lanelet_of_dynamic_obstacle_initial, xml_file[0].obstacle_by_id(2).initial_shape_lanelet_ids
        )
        self.assertEqual(
            exp_lanelet_of_dynamic_obstacle_prediction,
            xml_file[0].obstacle_by_id(2).prediction.shape_lanelet_assignment,
        )

        self.assertEqual(exp_traffic_sign_id, xml_file[0].lanelet_network.traffic_signs[0].traffic_sign_id)
        np.testing.assert_array_equal(exp_traffic_sign_position, xml_file[0].lanelet_network.traffic_signs[0].position)
        self.assertEqual(
            exp_traffic_sign_element_id,
            xml_file[0].lanelet_network.traffic_signs[0].traffic_sign_elements[0].traffic_sign_element_id.value,
        )
        self.assertEqual(
            exp_traffic_sign_element_additional_value,
            xml_file[0].lanelet_network.traffic_signs[0].traffic_sign_elements[0].additional_values[0],
        )
        self.assertEqual(exp_traffic_sign_virtual, xml_file[0].lanelet_network.traffic_signs[0].virtual)

        self.assertEqual(exp_planning_problem_id, xml_file[1].planning_problem_dict[1000].planning_problem_id)
        self.assertEqual(
            exp_planning_problem_initial_state_velocity, xml_file[1].planning_problem_dict[1000].initial_state.velocity
        )
        self.assertEqual(
            exp_planning_problem_initial_state_slip_angle,
            xml_file[1].planning_problem_dict[1000].initial_state.slip_angle,
        )
        self.assertEqual(
            exp_planning_problem_initial_state_yaw_rate, xml_file[1].planning_problem_dict[1000].initial_state.yaw_rate
        )

        self.assertSetEqual(exp_tags, xml_file[0].tags)
        self.assertEqual(exp_location_geo_name_id, xml_file[0].lanelet_network.location.geo_name_id)
        self.assertEqual(exp_location_latitude, xml_file[0].lanelet_network.location.gps_latitude)
        self.assertEqual(exp_location_longitude, xml_file[0].lanelet_network.location.gps_longitude)
        self.assertEqual(exp_location_geo, xml_file[0].lanelet_network.location.geo_transformation)
        self.assertEqual(exp_location_env_time_hours, xml_file[0].environment.time.hours)
        self.assertEqual(exp_location_env_time_minutes, xml_file[0].environment.time.minutes)
        self.assertEqual(exp_location_env_underground, xml_file[0].environment.underground)
        self.assertEqual(exp_location_env_time_of_day, xml_file[0].environment.time_of_day)
        self.assertEqual(exp_location_env_weather, xml_file[0].environment.weather)

    def test_open_intersection(self):
        exp_lanelet_stop_line_17_point_1 = self.stop_line_17.start
        exp_lanelet_stop_line_17_point_2 = self.stop_line_17.end
        exp_lanelet_stop_line_13_point_1 = self.stop_line_13.start
        exp_lanelet_stop_line_13_point_2 = self.stop_line_13.end
        exp_lanelet_stop_line_17_line_marking = self.stop_line_17.line_marking
        exp_lanelet_stop_line_13_line_marking = self.stop_line_13.line_marking
        exp_lanelet_stop_line_17_traffic_sign_ref = self.stop_line_17.traffic_sign_ref
        exp_lanelet_stop_line_13_traffic_sign_ref = self.stop_line_13.traffic_sign_ref
        exp_lanelet_stop_line_17_traffic_light_ref = self.stop_line_17.traffic_light_ref
        exp_lanelet_stop_line_13_traffic_light_ref = self.stop_line_13.traffic_light_ref
        exp_lanelet_stop_line_12 = self.stop_line_12
        exp_lanelet_traffic_sign_ref_12 = self.lanelet_12_traffic_sign_ref
        exp_lanelet_traffic_sign_ref_28 = self.lanelet_28_traffic_sign_ref
        exp_lanelet_traffic_lights_ref_13 = self.lanelet_13_traffic_lights_ref

        exp_traffic_sign_101_id = self.traffic_sign_101.traffic_sign_id
        exp_traffic_sign_101_position = self.traffic_sign_101.position
        exp_traffic_sign_101_element_id = self.traffic_sign_101.traffic_sign_elements[0].traffic_sign_element_id.value
        exp_traffic_sign_101_additional_value = self.traffic_sign_101.traffic_sign_elements[0].additional_values[0]
        exp_traffic_sign_101_virtual = self.traffic_sign_101.virtual
        exp_traffic_sign_105_id = self.traffic_sign_105.traffic_sign_id
        exp_traffic_sign_105_position = self.traffic_sign_105.position
        exp_traffic_sign_105_element_id_zero = self.traffic_sign_105.traffic_sign_elements[
            0
        ].traffic_sign_element_id.value
        exp_traffic_sign_105_element_id_one = self.traffic_sign_105.traffic_sign_elements[
            1
        ].traffic_sign_element_id.value
        exp_traffic_sign_105_virtual = self.traffic_sign_105.virtual

        exp_traffic_light_201_id = self.traffic_light_201.traffic_light_id
        exp_traffic_light_201_position = self.traffic_light_201.position
        exp_traffic_light_201_direction = self.traffic_light_201.direction
        exp_traffic_light_201_active = self.traffic_light_201.active
        exp_traffic_light_201_time_offset = self.traffic_light_201.traffic_light_cycle.time_offset
        exp_traffic_light_201_cycle_0_state = self.traffic_light_201.traffic_light_cycle.cycle_elements[0].state
        exp_traffic_light_201_cycle_0_duration = self.traffic_light_201.traffic_light_cycle.cycle_elements[0].duration
        exp_traffic_light_201_cycle_1_state = self.traffic_light_201.traffic_light_cycle.cycle_elements[1].state
        exp_traffic_light_201_cycle_1_duration = self.traffic_light_201.traffic_light_cycle.cycle_elements[1].duration
        exp_traffic_light_201_cycle_2_state = self.traffic_light_201.traffic_light_cycle.cycle_elements[2].state
        exp_traffic_light_201_cycle_2_duration = self.traffic_light_201.traffic_light_cycle.cycle_elements[2].duration
        exp_traffic_light_201_cycle_3_state = self.traffic_light_201.traffic_light_cycle.cycle_elements[3].state
        exp_traffic_light_201_cycle_3_duration = self.traffic_light_201.traffic_light_cycle.cycle_elements[3].duration

        exp_intersection_301_id = self.intersection_301.intersection_id
        exp_intersection_301_incoming_zero_id = self.intersection_301.incomings[0].incoming_id
        exp_intersection_301_incoming_zero_incoming_lanelets = self.intersection_301.incomings[0].incoming_lanelets
        exp_intersection_301_incoming_zero_successors_left = self.intersection_301.incomings[0].outgoing_left
        exp_intersection_301_incoming_zero_successors_right = self.intersection_301.incomings[0].outgoing_right
        exp_intersection_301_incoming_zero_successors_straight = self.intersection_301.incomings[0].outgoing_straight

        xml_file = CommonRoadFileReader(self.filename_urban).open()
        np.testing.assert_array_equal(
            exp_lanelet_stop_line_17_point_1, xml_file[0].lanelet_network.find_lanelet_by_id(17).stop_line.start
        )
        np.testing.assert_array_equal(
            exp_lanelet_stop_line_17_point_2, xml_file[0].lanelet_network.find_lanelet_by_id(17).stop_line.end
        )
        np.testing.assert_array_equal(
            exp_lanelet_stop_line_13_point_1, xml_file[0].lanelet_network.find_lanelet_by_id(13).stop_line.start
        )
        np.testing.assert_array_equal(
            exp_lanelet_stop_line_13_point_2, xml_file[0].lanelet_network.find_lanelet_by_id(13).stop_line.end
        )
        self.assertEqual(
            exp_lanelet_stop_line_17_line_marking,
            xml_file[0].lanelet_network.find_lanelet_by_id(17).stop_line.line_marking,
        )
        self.assertEqual(
            exp_lanelet_stop_line_13_line_marking,
            xml_file[0].lanelet_network.find_lanelet_by_id(13).stop_line.line_marking,
        )
        self.assertEqual(
            exp_lanelet_stop_line_17_traffic_sign_ref,
            xml_file[0].lanelet_network.find_lanelet_by_id(17).stop_line.traffic_sign_ref,
        )
        self.assertEqual(
            exp_lanelet_stop_line_13_traffic_sign_ref,
            xml_file[0].lanelet_network.find_lanelet_by_id(13).stop_line.traffic_sign_ref,
        )
        self.assertEqual(
            exp_lanelet_stop_line_17_traffic_light_ref,
            xml_file[0].lanelet_network.find_lanelet_by_id(17).stop_line.traffic_light_ref,
        )
        self.assertEqual(
            exp_lanelet_stop_line_13_traffic_light_ref,
            xml_file[0].lanelet_network.find_lanelet_by_id(13).stop_line.traffic_light_ref,
        )
        self.assertEqual(exp_lanelet_stop_line_12, xml_file[0].lanelet_network.find_lanelet_by_id(12).stop_line)
        self.assertSetEqual(
            exp_lanelet_traffic_sign_ref_12, xml_file[0].lanelet_network.find_lanelet_by_id(12).traffic_signs
        )
        self.assertSetEqual(
            exp_lanelet_traffic_sign_ref_28, xml_file[0].lanelet_network.find_lanelet_by_id(28).traffic_signs
        )
        self.assertSetEqual(
            exp_lanelet_traffic_lights_ref_13, xml_file[0].lanelet_network.find_lanelet_by_id(13).traffic_lights
        )

        self.assertEqual(exp_traffic_sign_101_id, xml_file[0].lanelet_network.traffic_signs[0].traffic_sign_id)
        np.testing.assert_array_equal(
            exp_traffic_sign_101_position, xml_file[0].lanelet_network.traffic_signs[0].position
        )
        self.assertEqual(
            exp_traffic_sign_101_element_id,
            xml_file[0].lanelet_network.traffic_signs[0].traffic_sign_elements[0].traffic_sign_element_id.value,
        )
        self.assertEqual(
            exp_traffic_sign_101_additional_value,
            xml_file[0].lanelet_network.traffic_signs[0].traffic_sign_elements[0].additional_values[0],
        )
        self.assertEqual(exp_traffic_sign_101_virtual, xml_file[0].lanelet_network.traffic_signs[0].virtual)

        self.assertEqual(exp_traffic_sign_105_id, xml_file[0].lanelet_network.traffic_signs[4].traffic_sign_id)
        np.testing.assert_array_equal(
            exp_traffic_sign_105_position, xml_file[0].lanelet_network.traffic_signs[4].position
        )
        self.assertEqual(
            exp_traffic_sign_105_element_id_zero,
            xml_file[0].lanelet_network.traffic_signs[4].traffic_sign_elements[0].traffic_sign_element_id.value,
        )
        self.assertEqual(
            exp_traffic_sign_105_element_id_one,
            xml_file[0].lanelet_network.traffic_signs[4].traffic_sign_elements[1].traffic_sign_element_id.value,
        )
        self.assertEqual(exp_traffic_sign_105_virtual, xml_file[0].lanelet_network.traffic_signs[4].virtual)

        self.assertEqual(exp_traffic_light_201_id, xml_file[0].lanelet_network.traffic_lights[0].traffic_light_id)
        np.testing.assert_array_equal(
            exp_traffic_light_201_position, xml_file[0].lanelet_network.traffic_lights[0].position
        )
        self.assertEqual(exp_traffic_light_201_direction, xml_file[0].lanelet_network.traffic_lights[0].direction)
        self.assertEqual(exp_traffic_light_201_active, xml_file[0].lanelet_network.traffic_lights[0].active)
        self.assertEqual(
            exp_traffic_light_201_time_offset,
            xml_file[0].lanelet_network.traffic_lights[0].traffic_light_cycle.time_offset,
        )
        self.assertEqual(
            exp_traffic_light_201_cycle_0_state,
            xml_file[0].lanelet_network.traffic_lights[0].traffic_light_cycle.cycle_elements[0].state,
        )
        self.assertEqual(
            exp_traffic_light_201_cycle_0_duration,
            xml_file[0].lanelet_network.traffic_lights[0].traffic_light_cycle.cycle_elements[0].duration,
        )
        self.assertEqual(
            exp_traffic_light_201_cycle_1_state,
            xml_file[0].lanelet_network.traffic_lights[0].traffic_light_cycle.cycle_elements[1].state,
        )
        self.assertEqual(
            exp_traffic_light_201_cycle_1_duration,
            xml_file[0].lanelet_network.traffic_lights[0].traffic_light_cycle.cycle_elements[1].duration,
        )
        self.assertEqual(
            exp_traffic_light_201_cycle_2_state,
            xml_file[0].lanelet_network.traffic_lights[0].traffic_light_cycle.cycle_elements[2].state,
        )
        self.assertEqual(
            exp_traffic_light_201_cycle_2_duration,
            xml_file[0].lanelet_network.traffic_lights[0].traffic_light_cycle.cycle_elements[2].duration,
        )
        self.assertEqual(
            exp_traffic_light_201_cycle_3_state,
            xml_file[0].lanelet_network.traffic_lights[0].traffic_light_cycle.cycle_elements[3].state,
        )
        self.assertEqual(
            exp_traffic_light_201_cycle_3_duration,
            xml_file[0].lanelet_network.traffic_lights[0].traffic_light_cycle.cycle_elements[3].duration,
        )

        self.assertSetEqual(
            exp_intersection_301_incoming_zero_incoming_lanelets,
            xml_file[0].lanelet_network.intersections[0].incomings[0].incoming_lanelets,
        )
        self.assertEqual(exp_intersection_301_id, xml_file[0].lanelet_network.intersections[0].intersection_id)
        self.assertEqual(
            exp_intersection_301_incoming_zero_id, xml_file[0].lanelet_network.intersections[0].incomings[0].incoming_id
        )
        self.assertSetEqual(
            exp_intersection_301_incoming_zero_successors_left,
            xml_file[0].lanelet_network.intersections[0].incomings[0].outgoing_left,
        )
        self.assertSetEqual(
            exp_intersection_301_incoming_zero_successors_right,
            xml_file[0].lanelet_network.intersections[0].incomings[0].outgoing_right,
        )
        self.assertSetEqual(
            exp_intersection_301_incoming_zero_successors_straight,
            xml_file[0].lanelet_network.intersections[0].incomings[0].outgoing_straight,
        )

    def test_open_with_lanelet_assignment(self):
        exp_static_obstacles_on_lanelet_zero = {self.scenario.static_obstacles[0].obstacle_id}
        exp_dynamic_obstacles_on_lanelet_zero = {
            0: {self.scenario.dynamic_obstacles[1].obstacle_id},
            1: {self.scenario.dynamic_obstacles[1].obstacle_id},
        }
        exp_dynamic_obstacles_on_lanelet_one = {
            0: {self.scenario.dynamic_obstacles[1].obstacle_id},
            1: {self.scenario.dynamic_obstacles[1].obstacle_id},
        }
        exp_lanelet_of_static_obstacle = {100}
        exp_lanelet_of_dynamic_obstacle_initial = {100, 101, 103, 104}
        exp_lanelet_of_dynamic_obstacle_prediction = {0: {100, 101, 103, 104}, 1: {100, 101, 103, 104}}

        xml_file = CommonRoadFileReader(self.filename_all).open(lanelet_assignment=True)

        self.assertSetEqual(
            exp_static_obstacles_on_lanelet_zero, xml_file[0].lanelet_network.lanelets[0].static_obstacles_on_lanelet
        )
        self.assertEqual(
            exp_dynamic_obstacles_on_lanelet_zero, xml_file[0].lanelet_network.lanelets[0].dynamic_obstacles_on_lanelet
        )
        self.assertEqual(
            exp_dynamic_obstacles_on_lanelet_one, xml_file[0].lanelet_network.lanelets[1].dynamic_obstacles_on_lanelet
        )
        self.assertSetEqual(exp_lanelet_of_static_obstacle, xml_file[0].obstacle_by_id(3).initial_shape_lanelet_ids)
        self.assertSetEqual(
            exp_lanelet_of_dynamic_obstacle_initial, xml_file[0].obstacle_by_id(2).initial_shape_lanelet_ids
        )
        self.assertEqual(
            exp_lanelet_of_dynamic_obstacle_prediction,
            xml_file[0].obstacle_by_id(2).prediction.shape_lanelet_assignment,
        )

    def test_open_without_lanelet_assignment(self):
        exp_static_obstacles_on_lanelet_zero = set()
        exp_dynamic_obstacles_on_lanelet_zero = {}
        exp_dynamic_obstacles_on_lanelet_one = {}
        exp_lanelet_of_static_obstacle = None
        exp_lanelet_of_dynamic_obstacle_initial = None
        exp_lanelet_of_dynamic_obstacle_prediction = None

        xml_file = CommonRoadFileReader(self.filename_all).open(lanelet_assignment=False)

        self.assertEqual(
            exp_static_obstacles_on_lanelet_zero, xml_file[0].lanelet_network.lanelets[0].static_obstacles_on_lanelet
        )
        self.assertEqual(
            exp_dynamic_obstacles_on_lanelet_zero, xml_file[0].lanelet_network.lanelets[0].dynamic_obstacles_on_lanelet
        )
        self.assertEqual(
            exp_dynamic_obstacles_on_lanelet_one, xml_file[0].lanelet_network.lanelets[1].dynamic_obstacles_on_lanelet
        )
        self.assertEqual(exp_lanelet_of_static_obstacle, xml_file[0].obstacle_by_id(3).initial_shape_lanelet_ids)
        self.assertEqual(
            exp_lanelet_of_dynamic_obstacle_initial, xml_file[0].obstacle_by_id(2).initial_shape_lanelet_ids
        )
        self.assertEqual(
            exp_lanelet_of_dynamic_obstacle_prediction,
            xml_file[0].obstacle_by_id(2).prediction.shape_lanelet_assignment,
        )

    def test_read_environment_obstacle(self):
        exp_environment_obstacle_id = self._environment_obstacle.obstacle_id
        exp_environment_obstacle_role = self._environment_obstacle.obstacle_role
        exp_environment_obstacle_type = self._environment_obstacle.obstacle_type
        exp_environment_obstacle_shape = self._environment_obstacle.obstacle_shape

        xml_file = CommonRoadFileReader(self.filename_all).open(lanelet_assignment=False)
        self.assertEqual(exp_environment_obstacle_id, xml_file[0].environment_obstacle[0].obstacle_id)
        self.assertEqual(exp_environment_obstacle_role, xml_file[0].environment_obstacle[0].obstacle_role)
        self.assertEqual(exp_environment_obstacle_type, xml_file[0].environment_obstacle[0].obstacle_type)
        np.testing.assert_array_almost_equal(
            exp_environment_obstacle_shape.vertices, xml_file[0].environment_obstacle[0].obstacle_shape.vertices
        )

    def test_read_phantom_obstacle(self):
        exp_phantom_obstacle_id = self._phantom_obstacle.obstacle_id
        exp_phantom_obstacle_role = self._phantom_obstacle.obstacle_role
        exp_obstacle_one_prediction_zero_shape_center = (
            self.scenario.obstacles[1].prediction.occupancy_set[0].shape.center
        )

        xml_file = CommonRoadFileReader(self.filename_all).open(lanelet_assignment=False)
        self.assertEqual(exp_phantom_obstacle_id, xml_file[0].phantom_obstacle[0].obstacle_id)
        self.assertEqual(exp_phantom_obstacle_role, xml_file[0].phantom_obstacle[0].obstacle_role)
        self.assertEqual(
            exp_obstacle_one_prediction_zero_shape_center[0],
            xml_file[0].obstacles[1].prediction.occupancy_set[0].shape.center[0],
        )
        self.assertEqual(
            exp_obstacle_one_prediction_zero_shape_center[1],
            xml_file[0].obstacles[1].prediction.occupancy_set[0].shape.center[1],
        )

    # def test_open_all_scenarios(self):
    #     scenarios_2020a = "TODO"
    #     scenarios_2018b = "TODO"
    #
    #     factory_2020a = scenarios_2020a + "/scenario-factory"
    #     hand_crafted_2020a = scenarios_2020a + "/hand-crafted"
    #     ngsim_lankershim_2020a = scenarios_2020a + "/NGSIM/Lankershim"
    #     ngsim_us101_2020a = scenarios_2020a + "/NGSIM/US101"
    #     ngsim_peachtree_2020a = scenarios_2020a + "/NGSIM/Peachtree"
    #     bicycle_2020a = scenarios_2020a + "/THI-Bicycle"
    #
    #     cooperative_2018b = scenarios_2018b + "/cooperative"
    #     bicycle_2018b = scenarios_2018b + "/THI-Bicycle"
    #     sumo_2018b = scenarios_2018b + "/SUMO"
    #     hand_crafted_2018b = scenarios_2018b + "/hand-crafted"
    #     ngsim_lankershim_2018b = scenarios_2018b + "/NGSIM/Lankershim"
    #     ngsim_us101_2018b = scenarios_2018b + "/NGSIM/US101"
    #     ngsim_peachtree_2018b = scenarios_2018b + "/NGSIM/Peachtree"
    #
    #     for scenario in os.listdir(hand_crafted_2020a):
    #         full_path = hand_crafted_2020a + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)
    #
    #     for scenario in os.listdir(ngsim_lankershim_2020a):
    #         full_path = ngsim_lankershim_2020a + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)
    #
    #     for scenario in os.listdir(ngsim_us101_2020a):
    #         full_path = ngsim_us101_2020a + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)
    #
    #     for scenario in os.listdir(ngsim_peachtree_2020a):
    #         full_path = ngsim_peachtree_2020a + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)
    #
    #     for scenario in os.listdir(bicycle_2020a):
    #         full_path = bicycle_2020a + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)
    #
    #     for scenario in os.listdir(factory_2020a):
    #         full_path = factory_2020a + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)
    #
    #     for scenario in os.listdir(cooperative_2018b):
    #         full_path = cooperative_2018b + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)
    #
    #     for scenario in os.listdir(sumo_2018b):
    #         full_path = sumo_2018b + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)
    #
    #     for scenario in os.listdir(bicycle_2018b):
    #         full_path = bicycle_2018b + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)
    #
    #     for scenario in os.listdir(ngsim_lankershim_2018b):
    #         full_path = ngsim_lankershim_2018b + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)
    #
    #     for scenario in os.listdir(ngsim_us101_2018b):
    #         full_path = ngsim_us101_2018b + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)
    #
    #     for scenario in os.listdir(ngsim_peachtree_2018b):
    #         full_path = ngsim_peachtree_2018b + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)
    #
    #     for scenario in os.listdir(hand_crafted_2018b):
    #         full_path = hand_crafted_2018b + "/" + scenario
    #         CommonRoadFileReader(full_path).open(lanelet_assignment=True)


class TestProtobufFileReader(unittest.TestCase):
    def setUp(self):
        self.cwd_path = os.path.dirname(os.path.abspath(__file__))
        self.out_path = self.cwd_path + "/../.pytest_cache"

        #  Carcarana
        self.filename_carcarana_xml = self.cwd_path + "/../test_scenarios/xml/2020a/ARG_Carcarana-4_5_T-1.xml"
        self.filename_carcarana_map_pb = self.cwd_path + "/../test_scenarios/protobuf/ARG_Carcarana-4.pb"
        self.filename_carcarana_scenario_pb = self.cwd_path + "/../test_scenarios/protobuf/ARG_Carcarana-4_5_T-1-SC.pb"
        self.filename_carcarana_dynamic_pb = self.cwd_path + "/../test_scenarios/protobuf/ARG_Carcarana-4_5_T-1.pb"

        #  Starnberg
        self.filename_starnberg_xml = self.cwd_path + "/../test_scenarios/xml/2020a/DEU_Starnberg-1_1_T-1.xml"
        self.filename_starnberg_map_pb = self.cwd_path + "/../test_scenarios/protobuf/DEU_Starnberg-1.pb"
        self.filename_starnberg_scenario_pb = self.cwd_path + "/../test_scenarios/protobuf/DEU_Starnberg-1_1_T-1-SC.pb"
        self.filename_starnberg_dynamic_pb = self.cwd_path + "/../test_scenarios/protobuf/DEU_Starnberg-1_1_T-1.pb"

        #  Anglet
        self.filename_anglet_xml = self.cwd_path + "/../test_scenarios/xml/2020a/FRA_Anglet-1_1_T-1.xml"
        self.filename_anglet_map_pb = self.cwd_path + "/../test_scenarios/protobuf/FRA_Anglet-1.pb"
        self.filename_anglet_scenario_pb = self.cwd_path + "/../test_scenarios/protobuf/FRA_Anglet-1_1_T-1-SC.pb"
        self.filename_anglet_dynamic_pb = self.cwd_path + "/../test_scenarios/protobuf/FRA_Anglet-1_1_T-1.pb"

        #  Peach
        self.filename_peach_xml = self.cwd_path + "/../test_scenarios/xml/2020a/USA_Peach-4_8_T-1.xml"
        self.filename_peach_map_pb = self.cwd_path + "/../test_scenarios/protobuf/USA_Peach-4.pb"
        self.filename_peach_scenario_pb = self.cwd_path + "/../test_scenarios/protobuf/USA_Peach-4_8_T-1-SC.pb"
        self.filename_peach_dynamic_pb = self.cwd_path + "/../test_scenarios/protobuf/USA_Peach-4_8_T-1.pb"

        #  All
        self.filename_all_xml = self.cwd_path + "/../test_scenarios/xml/2020a/ZAM_TestReadingAll-1_1_T-1.xml"
        self.filename_all_map_pb = self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingAll-1.pb"
        self.filename_all_scenario_pb = self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingAll-1_1_T-1-SC.pb"
        self.filename_all_dynamic_pb = self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingAll-1_1_T-1.pb"

        #  KS State
        self.filename_ks_xml = self.cwd_path + "/../test_scenarios/2018b/ZAM_TestReadingKsState-1_1_T-1.xml"
        self.filename_ks_map_pb = self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingKsState-1.pb"
        self.filename_ks_scenario_pb = (
            self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingKsState-1_1_T-1-SC.pb"
        )
        self.filename_ks_dynamic_pb = self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingKsState-1_1_T-1.pb"

        #  PM State
        self.filename_pm_xml = self.cwd_path + "/../test_scenarios/ZAM_TestReadingPmState-1_1_T-1.xml"
        self.filename_pm_map_pb = self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingPmState-1.pb"
        self.filename_pm_scenario_pb = (
            self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingPmState-1_1_T-1-SC.pb"
        )
        self.filename_pm_dynamic_pb = self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingPmState-1_1_T-1.pb"

        #  ST State
        self.filename_st_xml = self.cwd_path + "/../test_scenarios/ZAM_TestReadingStState-1_1_T-1.xml"
        self.filename_st_map_pb = self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingStState-1.pb"
        self.filename_st_scenario_pb = (
            self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingStState-1_1_T-1-SC.pb"
        )
        self.filename_st_dynamic_pb = self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingStState-1_1_T-1.pb"

        #  STD State
        self.filename_std_xml = self.cwd_path + "/../test_scenarios/ZAM_TestReadingStdState-1_1_T-1.xml"
        self.filename_std_map_pb = self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingStdState-1.pb"
        self.filename_std_scenario_pb = (
            self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingStdState-1_1_T-1-SC.pb"
        )
        self.filename_std_dynamic_pb = self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingStdState-1_1_T-1.pb"

        #  Custom State
        self.filename_custom_xml = self.cwd_path + "/../test_scenarios/ZAM_TestReadingCustomState-1_1_T-1.xml"
        self.filename_custom_map_pb = self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingCustomState-1.pb"
        self.filename_custom_scenario_pb = (
            self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingCustomState-1_1_T-1-SC.pb"
        )
        self.filename_custom_dynamic_pb = (
            self.cwd_path + "/../test_scenarios/protobuf/ZAM_TestReadingCustomState-1_1_T-1.pb"
        )

    def test_open_map(self):
        #  Carcarana
        self.assertTrue(read_compare_old_scenario_new_map(self.filename_carcarana_xml, self.filename_carcarana_map_pb))

        #  Starnberg
        self.assertTrue(read_compare_old_scenario_new_map(self.filename_starnberg_xml, self.filename_starnberg_map_pb))

        #  Anglet
        self.assertTrue(read_compare_old_scenario_new_map(self.filename_anglet_xml, self.filename_anglet_map_pb))

        #  Peach
        self.assertTrue(read_compare_old_scenario_new_map(self.filename_peach_xml, self.filename_peach_map_pb))

        #  All
        self.assertTrue(read_compare_old_scenario_new_map(self.filename_all_xml, self.filename_all_map_pb))

    def test_open_dynamic(self):
        #  Carcarana
        self.assertTrue(
            read_compare_old_scenario_new_dynamic(self.filename_carcarana_xml, self.filename_carcarana_dynamic_pb)
        )

        #  Starnberg
        self.assertTrue(
            read_compare_old_scenario_new_dynamic(self.filename_starnberg_xml, self.filename_starnberg_dynamic_pb)
        )

        #  Anglet
        self.assertTrue(
            read_compare_old_scenario_new_dynamic(self.filename_anglet_xml, self.filename_anglet_dynamic_pb)
        )

        #  Peach
        self.assertTrue(read_compare_old_scenario_new_dynamic(self.filename_peach_xml, self.filename_peach_dynamic_pb))

        #  All
        self.assertTrue(read_compare_old_scenario_new_dynamic(self.filename_all_xml, self.filename_all_dynamic_pb))

    def test_open_scenario(self):
        #  Carcarana
        self.assertTrue(
            read_compare_old_scenario_new_scenario(self.filename_carcarana_xml, self.filename_carcarana_scenario_pb)
        )

        #  Starnberg
        self.assertTrue(
            read_compare_old_scenario_new_scenario(self.filename_starnberg_xml, self.filename_starnberg_scenario_pb)
        )

        #  Anglet
        self.assertTrue(
            read_compare_old_scenario_new_scenario(self.filename_anglet_xml, self.filename_anglet_scenario_pb)
        )

        #  Peach
        self.assertTrue(
            read_compare_old_scenario_new_scenario(self.filename_peach_xml, self.filename_peach_scenario_pb)
        )

        #  All
        self.assertTrue(read_compare_old_scenario_new_scenario(self.filename_all_xml, self.filename_all_scenario_pb))

    def test_combine_map_dynamic(self):
        #  Carcarana
        self.assertTrue(
            read_compare_old_scenario_new_dynamic_map(
                self.filename_carcarana_xml, self.filename_carcarana_map_pb, self.filename_carcarana_dynamic_pb
            )
        )

        #  Starnberg
        self.assertTrue(
            read_compare_old_scenario_new_dynamic_map(
                self.filename_starnberg_xml, self.filename_starnberg_map_pb, self.filename_starnberg_dynamic_pb
            )
        )

        #  Anglet
        self.assertTrue(
            read_compare_old_scenario_new_dynamic_map(
                self.filename_anglet_xml, self.filename_anglet_map_pb, self.filename_anglet_dynamic_pb
            )
        )

        #  Peach
        self.assertTrue(
            read_compare_old_scenario_new_dynamic_map(
                self.filename_peach_xml, self.filename_peach_map_pb, self.filename_peach_dynamic_pb
            )
        )

        #  All
        self.assertTrue(
            read_compare_old_scenario_new_dynamic_map(
                self.filename_all_xml, self.filename_all_map_pb, self.filename_all_dynamic_pb
            )
        )

    def test_combine_all(self):
        #  Carcarana
        self.assertTrue(
            read_compare_old_scenario_new_all(
                self.filename_carcarana_xml,
                self.filename_carcarana_map_pb,
                self.filename_carcarana_dynamic_pb,
                self.filename_carcarana_scenario_pb,
            )
        )

        #  Starnberg
        self.assertTrue(
            read_compare_old_scenario_new_all(
                self.filename_starnberg_xml,
                self.filename_starnberg_map_pb,
                self.filename_starnberg_dynamic_pb,
                self.filename_starnberg_scenario_pb,
            )
        )

        #  Anglet
        self.assertTrue(
            read_compare_old_scenario_new_all(
                self.filename_anglet_xml,
                self.filename_anglet_map_pb,
                self.filename_anglet_dynamic_pb,
                self.filename_anglet_scenario_pb,
            )
        )

        #  Peach
        self.assertTrue(
            read_compare_old_scenario_new_all(
                self.filename_peach_xml,
                self.filename_peach_map_pb,
                self.filename_peach_dynamic_pb,
                self.filename_peach_scenario_pb,
            )
        )

        #  All
        self.assertTrue(
            read_compare_old_scenario_new_all(
                self.filename_all_xml,
                self.filename_all_map_pb,
                self.filename_all_dynamic_pb,
                self.filename_all_scenario_pb,
            )
        )

    def test_read_correct_matched_state(self):
        self._check_correct_matched_state_pb(self.filename_ks_map_pb, self.filename_ks_dynamic_pb, KSState)
        self._check_correct_matched_state_pb(self.filename_st_map_pb, self.filename_st_dynamic_pb, STState)
        self._check_correct_matched_state_pb(self.filename_std_map_pb, self.filename_std_dynamic_pb, STDState)
        self._check_correct_matched_state_pb(self.filename_custom_map_pb, self.filename_custom_dynamic_pb, CustomState)
        #  self._check_correct_matched_state_pb(self.filename_pm_map_pb, self.filename_pm_dynamic_pb,
        #                                       FileFormat.PROTOBUF, PMState)

    def test_wrong_filenames(self):
        reader = CommonRoadFileReader()

        with pytest.raises(NameError) as exc_info:
            reader.open()
        assert str(exc_info.value) == "Filename of the 2020a xml file is missing"

        with pytest.raises(NameError) as exc_info:
            reader.open_lanelet_network()
        assert str(exc_info.value) == "Filename of the 2020a xml file is missing"

        with pytest.raises(NameError) as exc_info:
            reader.open_map()
        assert str(exc_info.value) == "Filename of the 2024 map file is missing"

        with pytest.raises(NameError) as exc_info:
            reader.open_dynamic()
        assert str(exc_info.value) == "Filename of the 2024 dynamic file is missing"

        with pytest.raises(NameError) as exc_info:
            reader.open_scenario()
        assert str(exc_info.value) == "Filename of the 2024 scenario file is missing"

        with pytest.raises(NameError) as exc_info:
            reader.filename_map = self.filename_starnberg_map_pb
            reader.open_map_dynamic()
        assert str(exc_info.value) == "Filename of the 2024 dynamic file is missing"

        with pytest.raises(NameError) as exc_info:
            reader.filename_map = None
            reader.filename_dynamic = self.filename_starnberg_dynamic_pb
            reader.open_map_dynamic()
        assert str(exc_info.value) == "Filename of the 2024 map file is missing"

        with pytest.raises(NameError) as exc_info:
            reader.open_all()
        assert str(exc_info.value) == "Filename of the 2024 map file is missing"

        with pytest.raises(NameError) as exc_info:
            reader.filename_dynamic = self.filename_starnberg_dynamic_pb
            reader.filename_map = self.filename_starnberg_map_pb
            reader.open_all()
        assert str(exc_info.value) == "Filename of the 2024 scenario file is missing"

    def _check_correct_matched_state_pb(self, file_name_map: str, file_name_dynamic: str, state_type: type):
        road_network, environment_obstacles = CommonRoadFileReader(filename_map=file_name_map).open_map()
        dynamic = CommonRoadFileReader(filename_dynamic=file_name_dynamic).open_dynamic()
        dynamic.environment_obstacles = environment_obstacles

        scenario = combine_map_dynamic(road_network, dynamic, lanelet_assignment=True)
        obstacle = scenario.obstacles[0]
        self.assertIsInstance(obstacle.initial_state, InitialState)
        for state in obstacle.prediction.trajectory.state_list:
            self.assertIsInstance(state, state_type)

    def _check_correct_matched_state_xml(self, file_name: str, file_format: FileFormat, state_type: type):
        scenario, _ = CommonRoadFileReader(file_name).open()
        obstacle = scenario.obstacles[0]
        self.assertIsInstance(obstacle.initial_state, InitialState)
        for state in obstacle.prediction.trajectory.state_list:
            self.assertIsInstance(state, state_type)


def read_compare_old_scenario_new_scenario(xml_file_path: str, pb_scenario_file_path: str) -> bool:
    """
    Testing the similarities between the planning problems from the xml scenario and planning problems that the protobuf
    scenario reader function returns.
    """
    planning_problems = CommonRoadFileReader(xml_file_path).open()[1]
    scenario_pb = CommonRoadFileReader(filename_scenario=pb_scenario_file_path).open_scenario()

    #  In the old file reader we did not assign scenario tags to the planning problems
    for planning_problem in scenario_pb.planning_problems:
        planning_problem.scenario_tags = set()

    return list(planning_problems.planning_problem_dict.values()) == scenario_pb.planning_problems


def read_compare_old_scenario_new_map(xml_file_path: str, pb_map_file_path: str) -> bool:
    """
    Testing the similarities between the lanelet network from the xml scenario and the lanelet network that the protobuf
    map reader function returns.
    """
    scenario_xml = CommonRoadFileReader(xml_file_path).open()[0]
    map_pb, environment_obstacles = CommonRoadFileReader(filename_map=pb_map_file_path).open_map()

    # Make dates the same as they are dependent on the creation of the file
    scenario_xml.lanelet_network.meta_information.file_information.date = map_pb.meta_information.file_information.date

    # As the location is added to the lanelet network in the new format, we have to assign it to the old format scenario
    scenario_xml.lanelet_network.location = map_pb.location

    # As the new protobuf format Stop Line does not contain attributes "traffic_light_ref" and "traffic_sign_ref",
    # they are set to None in the xml format scenario
    for ll in scenario_xml.lanelet_network.lanelets:
        if ll.stop_line is not None:
            ll.stop_line.traffic_light_ref = None
            ll.stop_line.traffic_sign_ref = None

    #  Line_marking_left and line_marking_right vertices got removed from boundary so for now just equalize them
    for i in range(0, len(scenario_xml.lanelet_network.lanelets)):
        scenario_xml.lanelet_network.lanelets[i].line_marking_left_vertices = map_pb.lanelets[
            i
        ].line_marking_left_vertices
        scenario_xml.lanelet_network.lanelets[i].line_marking_right_vertices = map_pb.lanelets[
            i
        ].line_marking_right_vertices

    # We can't check the traffic lights as the formats differ. We can only do that in the combined map/dynamic file

    return (
        scenario_xml.lanelet_network.lanelets == map_pb.lanelets
        and scenario_xml.lanelet_network.meta_information.file_information == map_pb.meta_information.file_information
        and scenario_xml.lanelet_network.traffic_signs == map_pb.traffic_signs
        and scenario_xml.lanelet_network.areas == map_pb.areas
        and scenario_xml.lanelet_network.intersections == map_pb.intersections
        and scenario_xml.lanelet_network.location == map_pb.location
        and scenario_xml.environment_obstacle == environment_obstacles
    )


def read_compare_old_scenario_new_dynamic(xml_file_path: str, pb_dynamic_file_path: str) -> bool:
    """
    Testing the similarities between the obstacles from the xml scenario and the obstacles that the protobuf
    dynamic reader function returns.
    """
    scenario_xml = CommonRoadFileReader(xml_file_path).open()[0]
    dynamic_pb = CommonRoadFileReader(filename_dynamic=pb_dynamic_file_path).open_dynamic()

    return (
        scenario_xml.dynamic_obstacles == dynamic_pb.dynamic_obstacles
        and scenario_xml.static_obstacles == dynamic_pb.static_obstacles
        and scenario_xml.phantom_obstacle == dynamic_pb.phantom_obstacles
    )


def read_compare_old_scenario_new_dynamic_map(
    xml_file_path: str, pb_map_file_path: str, pb_dynamic_file_path: str
) -> bool:
    """
    Testing the similarities between the lanelet network and other scenario attributes from scenario, and the attributes
    that the protobuf reader for map and dynamic files returns.
    """
    scenario_xml = CommonRoadFileReader(xml_file_path).open()[0]
    map_pb, environment_obstacles = CommonRoadFileReader(filename_map=pb_map_file_path).open_map()
    dynamic_pb = CommonRoadFileReader(filename_dynamic=pb_dynamic_file_path).open_dynamic()
    dynamic_pb.environment_obstacles = environment_obstacles
    map_dynamic_pb = combine_map_dynamic(map_pb, dynamic_pb)

    map_dynamic_pb.lanelet_network.meta_information.file_information.date = (
        scenario_xml.lanelet_network.meta_information.file_information.date
    )
    scenario_xml.lanelet_network.location = map_pb.location
    scenario_xml.tags = map_dynamic_pb.tags
    map_dynamic_pb.file_information = scenario_xml.file_information

    # Environment is not a part of the location in the new format
    scenario_xml.environment = None

    # Boundaries are not important, it is ok if they are not the same as they are only used for protobuf mapping due
    # to the format difference
    scenario_xml.lanelet_network.boundaries = map_pb.boundaries

    # As the new protobuf format Stop Line does not contain attributes "traffic_light_ref" and "traffic_sign_ref",
    # they are set to None in the xml format scenario
    for ll in scenario_xml.lanelet_network.lanelets:
        if ll.stop_line is not None:
            ll.stop_line.traffic_light_ref = None
            ll.stop_line.traffic_sign_ref = None

    #  Line_marking_left and line_marking_right vertices got removed from boundary so for now just equalize them
    for i in range(0, len(scenario_xml.lanelet_network.lanelets)):
        scenario_xml.lanelet_network.lanelets[i].line_marking_left_vertices = map_pb.lanelets[
            i
        ].line_marking_left_vertices
        scenario_xml.lanelet_network.lanelets[i].line_marking_right_vertices = map_pb.lanelets[
            i
        ].line_marking_right_vertices

    return scenario_xml == map_dynamic_pb


def read_compare_old_scenario_new_all(
    xml_file_path: str, pb_map_file_path: str, pb_dynamic_file_path: str, pb_scenario_file_path: str
) -> bool:
    """
    Testing the similarities between the new and the old format reader scenario and planning problems.
    """
    scenario_xml, planning_problems = CommonRoadFileReader(xml_file_path).open()
    scenario_pb, planning_problems_pb, _ = CommonRoadFileReader(
        filename_dynamic=pb_dynamic_file_path, filename_map=pb_map_file_path, filename_scenario=pb_scenario_file_path
    ).open_all()

    scenario_pb.lanelet_network.meta_information.file_information.date = (
        scenario_xml.lanelet_network.meta_information.file_information.date
    )
    scenario_pb.file_information.date = scenario_xml.file_information.date

    scenario_xml.lanelet_network.location = scenario_pb.lanelet_network.location

    scenario_xml.lanelet_network.location.environment = None

    scenario_xml.tags = scenario_pb.tags
    for planning_problem in planning_problems_pb.planning_problem_dict.values():
        planning_problem.scenario_tags = set()

    # As the new protobuf format Stop Line does not contain attributes "traffic_light_ref" and "traffic_sign_ref",
    # they are set to None in the xml format scenario
    for ll in scenario_xml.lanelet_network.lanelets:
        if ll.stop_line is not None:
            ll.stop_line.traffic_light_ref = None
            ll.stop_line.traffic_sign_ref = None

    return scenario_xml == scenario_pb and planning_problems == planning_problems_pb


if __name__ == "__main__":
    unittest.main()
