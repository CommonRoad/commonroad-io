import os
import unittest

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import *
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet, GoalRegion
from commonroad.prediction.prediction import *
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LineMarking, LaneletType, RoadUser, StopLine
from commonroad.scenario.obstacle import *
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import *
from commonroad.scenario.traffic_sign import TrafficSign, TrafficSignElement, TrafficLightDirection, TrafficLight, \
    TrafficLightCycleElement, TrafficLightState
from commonroad.scenario.intersection import Intersection, IntersectionIncomingElement

class TestFileReader(unittest.TestCase):
    def setUp(self):
        self.cwd_path = os.path.dirname(os.path.abspath(__file__))
        self.filename_all = self.cwd_path + '/test_reading_all.xml'
        self.filename_urban = self.cwd_path + '/test_reading_intersection_traffic_sign.xml'
        self.filename_lanelets = self.cwd_path + '/test_reading_lanelets.xml'
        self.filename_obstacle = self.cwd_path + '/test_reading_obstacles.xml'
        self.filename_planning_problem = self.cwd_path + '/test_reading_planning_problem.xml'

        # setup for reading obstacles, lanelets, planning problem and all (without intersection)
        rectangle = Rectangle(4.3, 8.9, center=np.array([0.1, 0.5]), orientation=1.7)
        polygon = Polygon(np.array([np.array((0.0, 0.0)), np.array((0.0, 1.0)),
                                    np.array((1.0, 1.0)), np.array((1.0, 0.0))]))
        circ = Circle(2.0, np.array([0.0, 0.0]))
        occupancy_list = list()
        occupancy_list.append(Occupancy(0, rectangle))
        occupancy_list.append(Occupancy(1, circ))
        occupancy_list.append(Occupancy(2, polygon))
        occupancy_list.append(Occupancy(3, circ))

        set_pred = SetBasedPrediction(0, occupancy_list)

        states = []
        states.append(State(time_step=1, orientation=0, position=np.array([0, 1])))
        trajectory = Trajectory(0, states)
        init_state = State(time_step=0, orientation=0, position=np.array([0, 0]))
        traj_pred = TrajectoryPrediction(trajectory, rectangle)

        initial_signal_state = SignalState(time_step=0, horn=True, hazard_warning_lights=True, braking_lights=False)
        signal_series = [SignalState(time_step=1, horn=False, hazard_warning_lights=False, braking_lights=True)]

        static_obs = StaticObstacle(3, ObstacleType("unknown"), obstacle_shape=circ, initial_state=init_state)
        dyn_set_obs = DynamicObstacle(1, ObstacleType("unknown"),
                                      initial_state=traj_pred.trajectory.state_at_time_step(0),
                                      prediction=set_pred, obstacle_shape=rectangle)
        dyn_traj_obs = DynamicObstacle(2, ObstacleType("unknown"),
                                       initial_state=traj_pred.trajectory.state_at_time_step(0),
                                       prediction=traj_pred, obstacle_shape=rectangle,
                                       initial_signal_state=initial_signal_state, signal_series=signal_series)
        lanelet1 = Lanelet(left_vertices=np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]),
                           center_vertices=np.array([[0.0, 1], [1.0, 1], [2, 1]]),
                           right_vertices=np.array([[0.0, 2], [1.0, 2], [2, 2]]), lanelet_id=100,
                           predecessor=[101], successor=[101], adjacent_left=101, adjacent_left_same_direction=False,
                           adjacent_right=101, adjacent_right_same_direction=True,
                           line_marking_left_vertices=LineMarking.DASHED, line_marking_right_vertices=LineMarking.SOLID,
                           lanelet_type={LaneletType.HIGHWAY, LaneletType.MAIN_CARRIAGE_WAY},
                           user_one_way={RoadUser.VEHICLE}, traffic_signs={201})
        lanelet2 = Lanelet(left_vertices=np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]),
                           center_vertices=np.array([[0.0, 1], [1.0, 1], [2, 1]]),
                           right_vertices=np.array([[0.0, 2], [1.0, 2], [2, 2]]), lanelet_id=101,
                           predecessor=[100], successor=[100], adjacent_left=100, adjacent_left_same_direction=False,
                           adjacent_right=100, adjacent_right_same_direction=True,
                           line_marking_left_vertices=LineMarking.BROAD_DASHED,
                           line_marking_right_vertices=LineMarking.BROAD_SOLID,
                           lanelet_type={LaneletType.URBAN, LaneletType.BUS_LANE}, user_bidirectional={RoadUser.BUS})

        traffic_sign_201 = TrafficSign(traffic_sign_id=201, position=None,
                                       traffic_sign_elements=[TrafficSignElement("274", ["10"])], virtual=False)

        self.lanelet_network = LaneletNetwork().create_from_lanelet_list(list([lanelet1, lanelet2]))
        self.lanelet_network.add_traffic_sign(traffic_sign_201, [100])
        self.scenario = Scenario(0.1, 'ZAM_test_0-0-1')
        self.scenario.add_objects([static_obs, dyn_set_obs, dyn_traj_obs, self.lanelet_network])

        goal_region = GoalRegion([State(time_step=Interval(0, 1), velocity=Interval(0.0, 1), position=rectangle),
                                  State(time_step=Interval(1, 2), velocity=Interval(0.0, 1), position=circ)],
                                 {0: [101, 102], 1: list([102])})
        planning_problem = PlanningProblem(1000, State(velocity=0.1, position=np.array([[0], [0]]), orientation=0,
                                                       yaw_rate=0, slip_angle=0, time_step=0), goal_region)
        self.planning_problem_set = PlanningProblemSet(list([planning_problem]))

        # setup for reading intersection scenario with traffic signs, traffic lights, stop signs (without obstacles)
        self.stop_line_20 = StopLine(np.array([169.2560351117039, -54.95658983061205]),
                                     np.array([168.6607857447963, -57.38341449560771]), LineMarking.SOLID, None, 204)
        self.stop_line_28 = StopLine(np.array([180.1823263682323, -49.72698815669259]),
                                     np.array([177.8639861239823, -48.79316329157203]), LineMarking.SOLID, None, 201)
        self.stop_line_12 = None
        self.lanelet_12_traffic_sign_ref = {112}
        self.lanelet_28_traffic_sign_ref = {105}
        self.lanelet_28_traffic_lights_ref = {201}
        self.traffic_sign_101 = TrafficSign(traffic_sign_id=101,
                                            position=np.array([206.9839751212892, 20.67847944866278]),
                                            traffic_sign_elements=[TrafficSignElement("310",
                                                                                      ["Landeshauptstadt München"])],
                                            virtual=False)
        self.traffic_sign_105 = TrafficSign(traffic_sign_id=105,
                                            position=np.array([177.8639861239823, -48.79316329157203]),
                                            traffic_sign_elements=[TrafficSignElement("306", []),
                                                                   TrafficSignElement("720", [])], virtual=False)

        self.traffic_light_201 = TrafficLight(traffic_light_id=201,
                                              position=np.array([177.8639861239823, -48.79316329157203]),
                                              direction=TrafficLightDirection.ALL, active=True, time_offset=0,
                                              cycle=[TrafficLightCycleElement(state=TrafficLightState.RED,
                                                                              duration=15),
                                                     TrafficLightCycleElement(state=TrafficLightState.RED_YELLOW,
                                                                              duration=4),
                                                     TrafficLightCycleElement(state=TrafficLightState.GREEN,
                                                                              duration=15),
                                                     TrafficLightCycleElement(state=TrafficLightState.YELLOW,
                                                                              duration=4)])

        self.intersection_301 = \
            Intersection(301, [IntersectionIncomingElement(302, {24, 28, 30}, set(), set(), set(), set()),
                               IntersectionIncomingElement(302, {21, 25, 29}, set(), set(), set(), set()),
                               IntersectionIncomingElement(302, {23, 27, 31}, set(), set(), set(), set()),
                               IntersectionIncomingElement(302, {20, 22, 26}, set(), set(), set(), set())])

    def test_open_lanelets(self):
        lanelets = CommonRoadFileReader(self.filename_lanelets).open()

        exp_lanelet_zero_center_vertices = self.scenario.lanelet_network.lanelets[0].center_vertices
        exp_lanelet_zero_left_vertices = self.scenario.lanelet_network.lanelets[0].left_vertices
        exp_lanelet_zero_right_vertices = self.scenario.lanelet_network.lanelets[0].right_vertices
        exp_lanelet_zero_adj_left = self.scenario.lanelet_network.lanelets[0].adj_left
        exp_lanelet_zero_adj_right = self.scenario.lanelet_network.lanelets[0].adj_right
        exp_lanelet_zero_adj_left_same_direction = self.scenario.lanelet_network.lanelets[0].adj_left_same_direction
        exp_lanelet_zero_adj_right_same_direction = self.scenario.lanelet_network.lanelets[0].adj_right_same_direction
        exp_lanelet_zero_predecessor = self.scenario.lanelet_network.lanelets[0].predecessor
        exp_lanelet_zero_successor = self.scenario.lanelet_network.lanelets[0].successor

        np.testing.assert_array_equal(exp_lanelet_zero_center_vertices,
                                      lanelets[0].lanelet_network.lanelets[0].center_vertices)
        np.testing.assert_array_equal(exp_lanelet_zero_left_vertices,
                                      lanelets[0].lanelet_network.lanelets[0].left_vertices)
        np.testing.assert_array_equal(exp_lanelet_zero_right_vertices,
                                      lanelets[0].lanelet_network.lanelets[0].right_vertices)
        self.assertEqual(exp_lanelet_zero_adj_left, lanelets[0].lanelet_network.lanelets[0].adj_left)
        self.assertEqual(exp_lanelet_zero_adj_right, lanelets[0].lanelet_network.lanelets[0].adj_right)
        self.assertEqual(exp_lanelet_zero_adj_left_same_direction,
                         lanelets[0].lanelet_network.lanelets[0].adj_left_same_direction)
        self.assertEqual(exp_lanelet_zero_adj_right_same_direction,
                         lanelets[0].lanelet_network.lanelets[0].adj_right_same_direction)
        self.assertEqual(exp_lanelet_zero_predecessor, lanelets[0].lanelet_network.lanelets[0].predecessor)
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
        exp_obstacle_one_predicition_zero_shape_center = \
            self.scenario.obstacles[1].prediction.occupancy_set[0].shape.center

        exp_obstacle_two_id = self.scenario.obstacles[2].obstacle_id
        exp_obstacle_two_type = self.scenario.obstacles[2].obstacle_type
        exp_obstacle_two_role = self.scenario.obstacles[2].obstacle_role
        exp_obstacle_two_shape = self.scenario.obstacles[2].obstacle_shape.__class__
        exp_obstacle_two_shape_prediction_state_list_len = len(
            self.scenario.obstacles[2].prediction.trajectory.state_list)
        exp_obstacle_two_shape_prediction_final_state_attributes_len = len(
            self.scenario.obstacles[2].prediction.trajectory.final_state.attributes)

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
        self.assertEqual(exp_obstacle_one_predicition_zero_shape_center[0],
                         obstacles[0].obstacles[1].prediction.occupancy_set[
                             0].shape.center[0])
        self.assertEqual(exp_obstacle_one_predicition_zero_shape_center[1],
                         obstacles[0].obstacles[1].prediction.occupancy_set[
                             0].shape.center[1])

        self.assertEqual(exp_obstacle_two_id, obstacles[0].obstacles[2].obstacle_id)
        self.assertEqual(exp_obstacle_two_type, obstacles[0].obstacles[2].obstacle_type)
        self.assertEqual(exp_obstacle_two_role, obstacles[0].obstacles[2].obstacle_role)
        self.assertEqual(exp_obstacle_two_shape, obstacles[0].obstacles[2].obstacle_shape.__class__)
        self.assertEqual(exp_obstacle_two_shape_prediction_state_list_len, len(
            obstacles[0].obstacles[2].prediction.trajectory.state_list))
        self.assertEqual(exp_obstacle_two_shape_prediction_final_state_attributes_len, len(
            obstacles[0].obstacles[2].prediction.trajectory.final_state.attributes))

    def test_open_planning_problem(self):
        planning_problem = CommonRoadFileReader(self.filename_planning_problem).open()

        exp_planning_problem_id = self.planning_problem_set.planning_problem_dict[1000].planning_problem_id
        exp_planning_problem_initial_state_velocity = self.planning_problem_set.planning_problem_dict[
            1000].initial_state.velocity
        exp_planning_problem_initial_state_slip_angle = self.planning_problem_set.planning_problem_dict[
            1000].initial_state.slip_angle
        exp_planning_problem_initial_state_yaw_rate = self.planning_problem_set.planning_problem_dict[
            1000].initial_state.yaw_rate

        self.assertEqual(exp_planning_problem_id, planning_problem[1].planning_problem_dict[1000].planning_problem_id)
        self.assertEqual(exp_planning_problem_initial_state_velocity,
                         planning_problem[1].planning_problem_dict[1000].initial_state.velocity)
        self.assertEqual(exp_planning_problem_initial_state_slip_angle,
                         planning_problem[1].planning_problem_dict[1000].initial_state.slip_angle)
        self.assertEqual(exp_planning_problem_initial_state_yaw_rate,
                         planning_problem[1].planning_problem_dict[1000].initial_state.yaw_rate)

    def test_open_all(self):

        exp_num_lanelet_scenario = len(self.scenario.lanelet_network.lanelets)
        exp_num_obstacles_scenario = len(self.scenario.obstacles)
        exp_num_planning_problems = len(self.planning_problem_set.planning_problem_dict)
        exp_benchmark_id = self.scenario.benchmark_id
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
        exp_obstacle_one_predicition_zero_shape_center = \
            self.scenario.obstacles[1].prediction.occupancy_set[0].shape.center

        exp_obstacle_two_id = self.scenario.obstacles[2].obstacle_id
        exp_obstacle_two_type = self.scenario.obstacles[2].obstacle_type
        exp_obstacle_two_role = self.scenario.obstacles[2].obstacle_role
        exp_obstacle_two_shape = self.scenario.obstacles[2].obstacle_shape.__class__
        exp_obstacle_two_shape_prediction_state_list_len = \
            len(self.scenario.obstacles[2].prediction.trajectory.state_list)
        exp_obstacle_two_shape_prediction_final_state_attributes_len = \
            len(self.scenario.obstacles[2].prediction.trajectory.final_state.attributes)
        exp_obstacle_two_initial_signal_state_horn = self.scenario.obstacles[2].initial_signal_state.horn
        exp_obstacle_two_initial_signal_state_braking_lights = \
            self.scenario.obstacles[2].initial_signal_state.braking_lights
        exp_obstacle_two_initial_signal_state_hazard_warning_lights = \
            self.scenario.obstacles[2].initial_signal_state.hazard_warning_lights
        exp_obstacle_two_signal_state_time_step_1_horn = self.scenario.obstacles[2].signal_state_at_time_step(1).horn
        exp_obstacle_two_initial_signal_state_time_step_1_braking_lights = \
            self.scenario.obstacles[2].signal_state_at_time_step(1).braking_lights
        exp_obstacle_two_initial_signal_state_time_step_1_hazard_warning_lights = \
            self.scenario.obstacles[2].signal_state_at_time_step(1).hazard_warning_lights

        exp_lanelet_zero_center_vertices = self.scenario.lanelet_network.lanelets[0].center_vertices
        exp_lanelet_zero_left_vertices = self.scenario.lanelet_network.lanelets[0].left_vertices
        exp_lanelet_zero_right_vertices = self.scenario.lanelet_network.lanelets[0].right_vertices
        exp_lanelet_zero_adj_left = self.scenario.lanelet_network.lanelets[0].adj_left
        exp_lanelet_zero_adj_right = self.scenario.lanelet_network.lanelets[0].adj_right
        exp_lanelet_zero_adj_left_same_direction = self.scenario.lanelet_network.lanelets[0].adj_left_same_direction
        exp_lanelet_zero_adj_right_same_direction = self.scenario.lanelet_network.lanelets[0].adj_right_same_direction
        exp_lanelet_zero_predecessor = self.scenario.lanelet_network.lanelets[0].predecessor
        exp_lanelet_zero_succesor = self.scenario.lanelet_network.lanelets[0].successor
        exp_lanelet_zero_line_marking_left = self.lanelet_network.lanelets[0].line_marking_left_vertices
        exp_lanelet_zero_line_marking_right = self.lanelet_network.lanelets[0].line_marking_right_vertices
        exp_lanelet_one_line_marking_left = self.lanelet_network.lanelets[1].line_marking_left_vertices
        exp_lanelet_one_line_marking_right = self.lanelet_network.lanelets[1].line_marking_right_vertices
        exp_lanelet_zero_type = self.lanelet_network.lanelets[0].lanelet_type
        exp_lanelet_one_type = self.lanelet_network.lanelets[1].lanelet_type
        exp_lanelet_zero_user_one_way = self.lanelet_network.lanelets[0].user_one_way
        exp_lanelet_one_user_bidirectional = self.lanelet_network.lanelets[1].user_bidirectional
        exp_lanelet_zero_traffic_signs = self.lanelet_network.lanelets[0].traffic_signs

        exp_static_obstacles_on_lanelet_zero = {self.scenario.static_obstacles[0].obstacle_id}
        exp_static_obstacles_on_lanelet_one = {self.scenario.static_obstacles[0].obstacle_id}
        exp_dynamic_obstacles_on_lanelet_zero = {0: {self.scenario.dynamic_obstacles[1].obstacle_id},
                                                 1: {self.scenario.dynamic_obstacles[1].obstacle_id}}
        exp_dynamic_obstacles_on_lanelet_one = {0: {self.scenario.dynamic_obstacles[1].obstacle_id},
                                                1: {self.scenario.dynamic_obstacles[1].obstacle_id}}
        exp_lanelet_of_static_obstacle = {100, 101}
        exp_lanelet_of_dynamic_obstacle_initial = {100, 101}
        exp_lanelet_of_dynamic_obstacle_prediction = {0: {100, 101}, 1: {100, 101}}

        exp_traffic_sign_id = self.lanelet_network.traffic_signs[0].traffic_sign_id
        exp_traffic_sign_position = self.lanelet_network.traffic_signs[0].position
        exp_traffic_sign_element_id = \
            self.lanelet_network.traffic_signs[0].traffic_sign_elements[0].traffic_sign_element_id
        exp_traffic_sign_element_additional_value = \
            self.lanelet_network.traffic_signs[0].traffic_sign_elements[0].additional_values[0]
        exp_traffic_sign_virtual = self.lanelet_network.traffic_signs[0].virtual

        exp_planning_problem_id = self.planning_problem_set.planning_problem_dict[1000].planning_problem_id
        exp_planning_problem_initial_state_velocity = \
            self.planning_problem_set.planning_problem_dict[1000].initial_state.velocity
        exp_planning_problem_initial_state_slip_angle = \
            self.planning_problem_set.planning_problem_dict[1000].initial_state.slip_angle
        exp_planning_problem_initial_state_yaw_rate = \
            self.planning_problem_set.planning_problem_dict[1000].initial_state.yaw_rate

        xml_file = CommonRoadFileReader(self.filename_all).open()

        self.assertEqual(exp_num_lanelet_scenario,len(xml_file[0].lanelet_network.lanelets))
        self.assertEqual(exp_num_obstacles_scenario,len(xml_file[0].obstacles))
        self.assertEqual(exp_num_planning_problems, len(xml_file[1].planning_problem_dict))
        self.assertEqual(exp_benchmark_id, xml_file[0].benchmark_id)
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
        self.assertEqual(exp_obstacle_one_predicition_zero_shape_center[0],
                         xml_file[0].obstacles[1].prediction.occupancy_set[0].shape.center[0])
        self.assertEqual(exp_obstacle_one_predicition_zero_shape_center[1],
                         xml_file[0].obstacles[1].prediction.occupancy_set[0].shape.center[1])

        self.assertEqual(exp_obstacle_two_id, xml_file[0].obstacles[2].obstacle_id)
        self.assertEqual(exp_obstacle_two_type, xml_file[0].obstacles[2].obstacle_type)
        self.assertEqual(exp_obstacle_two_role, xml_file[0].obstacles[2].obstacle_role)
        self.assertEqual(exp_obstacle_two_shape, xml_file[0].obstacles[2].obstacle_shape.__class__)
        self.assertEqual(exp_obstacle_two_shape_prediction_state_list_len,
                         len(xml_file[0].obstacles[2].prediction.trajectory.state_list))
        self.assertEqual(exp_obstacle_two_shape_prediction_final_state_attributes_len,
                         len(xml_file[0].obstacles[2].prediction.trajectory.final_state.attributes))
        self.assertEqual(exp_obstacle_two_initial_signal_state_horn,
                         xml_file[0].obstacles[2].initial_signal_state.horn)
        self.assertEqual(exp_obstacle_two_initial_signal_state_braking_lights,
                         xml_file[0].obstacles[2].initial_signal_state.braking_lights)
        self.assertEqual(exp_obstacle_two_initial_signal_state_hazard_warning_lights,
                         xml_file[0].obstacles[2].initial_signal_state.hazard_warning_lights)
        self.assertEqual(exp_obstacle_two_signal_state_time_step_1_horn,
                         xml_file[0].obstacles[2].signal_state_at_time_step(1).horn)
        self.assertEqual(exp_obstacle_two_initial_signal_state_time_step_1_braking_lights,
                         xml_file[0].obstacles[2].signal_state_at_time_step(1).braking_lights)
        self.assertEqual(exp_obstacle_two_initial_signal_state_time_step_1_hazard_warning_lights,
                         xml_file[0].obstacles[2].signal_state_at_time_step(1).hazard_warning_lights)

        np.testing.assert_array_equal(exp_lanelet_zero_center_vertices,
                                      xml_file[0].lanelet_network.lanelets[0].center_vertices)
        np.testing.assert_array_equal(exp_lanelet_zero_left_vertices,
                                      xml_file[0].lanelet_network.lanelets[0].left_vertices)
        np.testing.assert_array_equal(exp_lanelet_zero_right_vertices,
                                      xml_file[0].lanelet_network.lanelets[0].right_vertices)
        self.assertEqual(exp_lanelet_zero_adj_left, xml_file[0].lanelet_network.lanelets[0].adj_left)
        self.assertEqual(exp_lanelet_zero_adj_right, xml_file[0].lanelet_network.lanelets[0].adj_right)
        self.assertEqual(exp_lanelet_zero_adj_left_same_direction,
                         xml_file[0].lanelet_network.lanelets[0].adj_left_same_direction)
        self.assertEqual(exp_lanelet_zero_adj_right_same_direction,
                         xml_file[0].lanelet_network.lanelets[0].adj_right_same_direction)
        self.assertEqual(exp_lanelet_zero_predecessor, xml_file[0].lanelet_network.lanelets[0].predecessor)
        self.assertEqual(exp_lanelet_zero_succesor, xml_file[0].lanelet_network.lanelets[0].successor)
        self.assertEqual(exp_lanelet_zero_line_marking_left,
                         xml_file[0].lanelet_network.lanelets[0].line_marking_left_vertices)
        self.assertEqual(exp_lanelet_zero_line_marking_right,
                         xml_file[0].lanelet_network.lanelets[0].line_marking_right_vertices)
        self.assertEqual(exp_lanelet_one_line_marking_left,
                         xml_file[0].lanelet_network.lanelets[1].line_marking_left_vertices)
        self.assertEqual(exp_lanelet_one_line_marking_right,
                         xml_file[0].lanelet_network.lanelets[1].line_marking_right_vertices)
        self.assertEqual(exp_lanelet_zero_type, xml_file[0].lanelet_network.lanelets[0].lanelet_type)
        self.assertEqual(exp_lanelet_one_type, xml_file[0].lanelet_network.lanelets[1].lanelet_type)
        self.assertEqual(exp_lanelet_zero_user_one_way, xml_file[0].lanelet_network.lanelets[0].user_one_way)
        self.assertEqual(exp_lanelet_one_user_bidirectional, xml_file[0].lanelet_network.lanelets[1].user_bidirectional)
        self.assertSetEqual(exp_lanelet_zero_traffic_signs, xml_file[0].lanelet_network.lanelets[1].traffic_signs)

        self.assertSetEqual(exp_static_obstacles_on_lanelet_zero,
                            xml_file[0].lanelet_network.lanelets[0].static_obstacles_on_lanelet)
        self.assertSetEqual(exp_static_obstacles_on_lanelet_one,
                            xml_file[0].lanelet_network.lanelets[1].static_obstacles_on_lanelet)
        self.assertEqual(exp_dynamic_obstacles_on_lanelet_zero,
                         xml_file[0].lanelet_network.lanelets[0].dynamic_obstacles_on_lanelet)
        self.assertEqual(exp_dynamic_obstacles_on_lanelet_one,
                         xml_file[0].lanelet_network.lanelets[1].dynamic_obstacles_on_lanelet)
        self.assertSetEqual(exp_lanelet_of_static_obstacle,
                            xml_file[0].obstacle_by_id(3).initial_lanelet_ids)
        self.assertSetEqual(exp_lanelet_of_dynamic_obstacle_initial,
                            xml_file[0].obstacle_by_id(2).initial_lanelet_ids)
        self.assertEqual(exp_lanelet_of_dynamic_obstacle_prediction,
                         xml_file[0].obstacle_by_id(2).prediction.lanelet_assignment)

        self.assertEqual(exp_traffic_sign_id, xml_file[0].lanelet_network.traffic_signs[0].traffic_sign_id)
        np.testing.assert_array_equal(exp_traffic_sign_position,
                                      xml_file[0].lanelet_network.traffic_signs[0].position)
        self.assertEqual(exp_traffic_sign_element_id,
                         xml_file[0].lanelet_network.traffic_signs[0].traffic_sign_elements[0].traffic_sign_element_id)
        self.assertEqual(exp_traffic_sign_element_additional_value,
                         xml_file[0].lanelet_network.traffic_signs[0].traffic_sign_elements[0].additional_values[0])
        self.assertEqual(exp_traffic_sign_virtual, xml_file[0].lanelet_network.traffic_signs[0].virtual)

        self.assertEqual(exp_planning_problem_id, xml_file[1].planning_problem_dict[1000].planning_problem_id)
        self.assertEqual(exp_planning_problem_initial_state_velocity,
                         xml_file[1].planning_problem_dict[1000].initial_state.velocity)
        self.assertEqual(exp_planning_problem_initial_state_slip_angle,
                         xml_file[1].planning_problem_dict[1000].initial_state.slip_angle)
        self.assertEqual(exp_planning_problem_initial_state_yaw_rate,
                         xml_file[1].planning_problem_dict[1000].initial_state.yaw_rate)

    def test_open_intersection(self):
        exp_lanelet_stop_line_20_point_1 = self.stop_line_20.start
        exp_lanelet_stop_line_20_point_2 = self.stop_line_20.end
        exp_lanelet_stop_line_28_point_1 = self.stop_line_28.start
        exp_lanelet_stop_line_28_point_2 = self.stop_line_28.end
        exp_lanelet_stop_line_20_line_marking = self.stop_line_20.line_marking
        exp_lanelet_stop_line_28_line_marking = self.stop_line_28.line_marking
        exp_lanelet_stop_line_20_traffic_sign_ref = self.stop_line_20.traffic_sign_ref
        exp_lanelet_stop_line_28_traffic_sign_ref = self.stop_line_28.traffic_sign_ref
        exp_lanelet_stop_line_20_traffic_light_ref = self.stop_line_20.traffic_light_ref
        exp_lanelet_stop_line_28_traffic_light_ref = self.stop_line_28.traffic_light_ref
        exp_lanelet_stop_line_12 = self.stop_line_12
        exp_lanelet_traffic_sign_ref_12 = self.lanelet_12_traffic_sign_ref
        exp_lanelet_traffic_sign_ref_28 = self.lanelet_28_traffic_sign_ref
        exp_lanelet_traffic_lights_ref_28 = self.lanelet_28_traffic_lights_ref

        exp_traffic_sign_101_id = self.traffic_sign_101.traffic_sign_id
        exp_traffic_sign_101_position = self.traffic_sign_101.position
        exp_traffic_sign_101_element_id = self.traffic_sign_101.traffic_sign_elements[0].traffic_sign_element_id
        exp_traffic_sign_101_additional_value = \
            self.traffic_sign_101.traffic_sign_elements[0].additional_values[0]
        exp_traffic_sign_101_virtual = self.traffic_sign_101.virtual
        exp_traffic_sign_105_id = self.traffic_sign_105.traffic_sign_id
        exp_traffic_sign_105_position = self.traffic_sign_105.position
        exp_traffic_sign_105_element_id_zero = self.traffic_sign_105.traffic_sign_elements[0].traffic_sign_element_id
        exp_traffic_sign_105_element_id_one = \
            self.traffic_sign_105.traffic_sign_elements[1].traffic_sign_element_id
        exp_traffic_sign_105_virtual = self.traffic_sign_105.virtual

        exp_traffic_light_201_id = self.traffic_light_201.traffic_light_id
        exp_traffic_light_201_position = self.traffic_light_201.position
        exp_traffic_light_201_direction = self.traffic_light_201.direction
        exp_traffic_light_201_active = self.traffic_light_201.active
        exp_traffic_light_201_time_offset = self.traffic_light_201.time_offset
        exp_traffic_light_201_cycle_0_state = self.traffic_light_201.cycle[0].state
        exp_traffic_light_201_cycle_0_duration = self.traffic_light_201.cycle[0].duration
        exp_traffic_light_201_cycle_1_state = self.traffic_light_201.cycle[1].state
        exp_traffic_light_201_cycle_1_duration = self.traffic_light_201.cycle[1].duration
        exp_traffic_light_201_cycle_2_state = self.traffic_light_201.cycle[2].state
        exp_traffic_light_201_cycle_2_duration = self.traffic_light_201.cycle[2].duration
        exp_traffic_light_201_cycle_3_state = self.traffic_light_201.cycle[3].state
        exp_traffic_light_201_cycle_3_duration = self.traffic_light_201.cycle[3].duration

        exp_intersection_301_incoming_zero_incoming_lanelets = self.intersection_301.incomings[0].incoming_lanelets
        exp_intersection_301_id = self.intersection_301.intersection_id
        exp_intersection_301_incoming_zero_id = self.intersection_301.incomings[0].incoming_id

        xml_file = CommonRoadFileReader(self.filename_urban).open()

        np.testing.assert_array_equal(exp_lanelet_stop_line_20_point_1,
                                      xml_file[0].lanelet_network.find_lanelet_by_id(20).stop_line.start)
        np.testing.assert_array_equal(exp_lanelet_stop_line_20_point_2,
                                      xml_file[0].lanelet_network.find_lanelet_by_id(20).stop_line.end)
        np.testing.assert_array_equal(exp_lanelet_stop_line_28_point_1,
                                      xml_file[0].lanelet_network.find_lanelet_by_id(28).stop_line.start)
        np.testing.assert_array_equal(exp_lanelet_stop_line_28_point_2,
                                      xml_file[0].lanelet_network.find_lanelet_by_id(28).stop_line.end)
        self.assertEqual(exp_lanelet_stop_line_20_line_marking,
                         xml_file[0].lanelet_network.find_lanelet_by_id(20).stop_line.line_marking)
        self.assertEqual(exp_lanelet_stop_line_28_line_marking,
                         xml_file[0].lanelet_network.find_lanelet_by_id(28).stop_line.line_marking)
        self.assertEqual(exp_lanelet_stop_line_20_traffic_sign_ref,
                         xml_file[0].lanelet_network.find_lanelet_by_id(20).stop_line.traffic_sign_ref)
        self.assertEqual(exp_lanelet_stop_line_28_traffic_sign_ref,
                         xml_file[0].lanelet_network.find_lanelet_by_id(28).stop_line.traffic_sign_ref)
        self.assertEqual(exp_lanelet_stop_line_20_traffic_light_ref,
                         xml_file[0].lanelet_network.find_lanelet_by_id(20).stop_line.traffic_light_ref)
        self.assertEqual(exp_lanelet_stop_line_28_traffic_light_ref,
                         xml_file[0].lanelet_network.find_lanelet_by_id(28).stop_line.traffic_light_ref)
        self.assertEqual(exp_lanelet_stop_line_12, xml_file[0].lanelet_network.find_lanelet_by_id(12).stop_line)
        self.assertSetEqual(exp_lanelet_traffic_sign_ref_12,
                            xml_file[0].lanelet_network.find_lanelet_by_id(12).traffic_signs)
        self.assertSetEqual(exp_lanelet_traffic_sign_ref_28,
                            xml_file[0].lanelet_network.find_lanelet_by_id(28).traffic_signs)
        self.assertSetEqual(exp_lanelet_traffic_lights_ref_28,
                            xml_file[0].lanelet_network.find_lanelet_by_id(28).traffic_lights)

        self.assertEqual(exp_traffic_sign_101_id, xml_file[0].lanelet_network.traffic_signs[0].traffic_sign_id)
        np.testing.assert_array_equal(exp_traffic_sign_101_position,
                                      xml_file[0].lanelet_network.traffic_signs[0].position)
        self.assertEqual(exp_traffic_sign_101_element_id,
                         xml_file[0].lanelet_network.traffic_signs[0].traffic_sign_elements[0].traffic_sign_element_id)
        self.assertEqual(exp_traffic_sign_101_additional_value,
                         xml_file[0].lanelet_network.traffic_signs[0].traffic_sign_elements[0].additional_values[0])
        self.assertEqual(exp_traffic_sign_101_virtual, xml_file[0].lanelet_network.traffic_signs[0].virtual)

        self.assertEqual(exp_traffic_sign_105_id, xml_file[0].lanelet_network.traffic_signs[4].traffic_sign_id)
        np.testing.assert_array_equal(exp_traffic_sign_105_position,
                                      xml_file[0].lanelet_network.traffic_signs[4].position)
        self.assertEqual(exp_traffic_sign_105_element_id_zero,
                         xml_file[0].lanelet_network.traffic_signs[4].traffic_sign_elements[0].traffic_sign_element_id)
        self.assertEqual(exp_traffic_sign_105_element_id_one,
                         xml_file[0].lanelet_network.traffic_signs[4].traffic_sign_elements[1].traffic_sign_element_id)
        self.assertEqual(exp_traffic_sign_105_virtual, xml_file[0].lanelet_network.traffic_signs[4].virtual)

        self.assertEqual(exp_traffic_light_201_id, xml_file[0].lanelet_network.traffic_lights[0].traffic_light_id)
        np.testing.assert_array_equal(exp_traffic_light_201_position,
                                      xml_file[0].lanelet_network.traffic_lights[0].position)
        self.assertEqual(exp_traffic_light_201_direction, xml_file[0].lanelet_network.traffic_lights[0].direction)
        self.assertEqual(exp_traffic_light_201_active, xml_file[0].lanelet_network.traffic_lights[0].active)
        self.assertEqual(exp_traffic_light_201_time_offset, xml_file[0].lanelet_network.traffic_lights[0].time_offset)
        self.assertEqual(exp_traffic_light_201_cycle_0_state,
                         xml_file[0].lanelet_network.traffic_lights[0].cycle[0].state)
        self.assertEqual(exp_traffic_light_201_cycle_0_duration,
                         xml_file[0].lanelet_network.traffic_lights[0].cycle[0].duration)
        self.assertEqual(exp_traffic_light_201_cycle_1_state,
                         xml_file[0].lanelet_network.traffic_lights[0].cycle[1].state)
        self.assertEqual(exp_traffic_light_201_cycle_1_duration,
                         xml_file[0].lanelet_network.traffic_lights[0].cycle[1].duration)
        self.assertEqual(exp_traffic_light_201_cycle_2_state,
                         xml_file[0].lanelet_network.traffic_lights[0].cycle[2].state)
        self.assertEqual(exp_traffic_light_201_cycle_2_duration,
                         xml_file[0].lanelet_network.traffic_lights[0].cycle[2].duration)
        self.assertEqual(exp_traffic_light_201_cycle_3_state,
                         xml_file[0].lanelet_network.traffic_lights[0].cycle[3].state)
        self.assertEqual(exp_traffic_light_201_cycle_3_duration,
                         xml_file[0].lanelet_network.traffic_lights[0].cycle[3].duration)

        self.assertSetEqual(exp_intersection_301_incoming_zero_incoming_lanelets,
                            xml_file[0].lanelet_network.intersections[0].incomings[0].incoming_lanelets)
        self.assertEqual(exp_intersection_301_id,
                            xml_file[0].lanelet_network.intersections[0].intersection_id)
        self.assertEqual(exp_intersection_301_incoming_zero_id,
                            xml_file[0].lanelet_network.intersections[0].incomings[0].incoming_id)

    # def test_open_all_scenarios(self):
    #     scenarios = self.cwd_path
    #     scenarios = scenarios[:len(scenarios)-51] + "/scenarios"
    #     cooperative = scenarios + "/cooperative"
    #     hand_crafted = scenarios + "/hand-crafted"
    #     NGSIM_Lankershim = scenarios + "/NGSIM/Lankershim"
    #     NGSIM_US101 = scenarios + "/NGSIM/US101"
    #
    #     for scenario in os.listdir(hand_crafted):
    #         full_path = hand_crafted + "/" + scenario
    #         CommonRoadFileReader(full_path).open()
    #
    #     for scenario in os.listdir(NGSIM_Lankershim):
    #         full_path = NGSIM_Lankershim + "/" + scenario
    #         CommonRoadFileReader(full_path).open()
    #
    #     for scenario in os.listdir(NGSIM_US101):
    #         full_path = NGSIM_US101 + "/" + scenario
    #         CommonRoadFileReader(full_path).open()
    #
    #     for scenario in os.listdir(cooperative):
    #         full_path = cooperative + "/" + scenario
    #         CommonRoadFileReader(full_path).open()

if __name__ == '__main__':
    unittest.main()
