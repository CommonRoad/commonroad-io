import unittest
from copy import deepcopy

from commonroad import SCENARIO_VERSION
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import *
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LineMarking
from commonroad.scenario.obstacle import *
from commonroad.scenario.scenario import Scenario, Environment, TimeOfDay, Time, Underground, Weather, Location, \
    ScenarioID
from commonroad.scenario.traffic_sign import TrafficSign, TrafficSignElement, TrafficSignIDGermany
from commonroad.scenario.trajectory import *
from commonroad.common.util import Interval


class TestScenario(unittest.TestCase):
    def setUp(self):
        self.rectangle = Rectangle(4, 5)
        polygon = Polygon(
            np.array([np.array((0.0, 0.0)), np.array((0.0, 1.0)), np.array((1.0, 1.0)), np.array((1.0, 0.0))]))
        self.circ = Circle(2.0)
        sg = ShapeGroup([self.circ, self.rectangle])
        occupancy_list = list()

        occupancy_list.append(Occupancy(0, self.rectangle))
        occupancy_list.append(Occupancy(1, self.circ))
        occupancy_list.append(Occupancy(2, polygon))
        occupancy_list.append(Occupancy(3, self.circ))

        self.lanelet1 = Lanelet(np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]), np.array([[0.0, 1], [1.0, 1], [2, 1]]),
                           np.array([[0.0, 2], [1.0, 2], [2, 2]]), 100,
                           [101], [101], 101, False, 101, True,
                           LineMarking.DASHED, LineMarking.DASHED,None, None, None,None, {1})
        self.lanelet1.add_static_obstacle_to_lanelet(0)
        self.lanelet2 = Lanelet(np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]), np.array([[0.0, 1], [1.0, 1], [2, 1]]),
                           np.array([[0.0, 2], [1.0, 2], [2, 2]]), 101,
                           [100], [100], 100, False, 100, True,
                           LineMarking.DASHED, LineMarking.DASHED, None, None, None, None, {1})
        self.lanelet1.add_dynamic_obstacle_to_lanelet(2, 0)
        self.lanelet1.add_dynamic_obstacle_to_lanelet(2, 1)
        self.lanelet_network = LaneletNetwork().create_from_lanelet_list(list([self.lanelet1, self.lanelet2]))
        traffic_sign_max_speed = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED.value, ['10.0'])
        traffic_sign = TrafficSign(3, [traffic_sign_max_speed], {100}, np.array([0.0, 2]))
        self.lanelet_network.add_traffic_sign(traffic_sign, [])
        self.set_pred = SetBasedPrediction(0, occupancy_list)

        states = list()
        states.append(State(time_step=0, orientation=0, position=np.array([0, 0]), velocity=5))
        states.append(State(time_step=1, orientation=0, position=np.array([0, 1]), velocity=10))
        trajectory = Trajectory(0, states)

        self.init_state = State(time_step=0, orientation=0, position=np.array([0, 0]), velocity=15)

        self.traj_pred = TrajectoryPrediction(trajectory, self.rectangle, {0: {100, 101}, 1: {100, 101}})

        self.static_obs_on_lanelet = StaticObstacle(0, ObstacleType("unknown"), obstacle_shape=self.circ,
                                                    initial_state=self.init_state, initial_shape_lanelet_ids={100, 101})
        self.static_obs = StaticObstacle(0, ObstacleType("unknown"), obstacle_shape=self.circ,
                                         initial_state=self.init_state, initial_shape_lanelet_ids={100, 101})
        self.dyn_set_obs = DynamicObstacle(1, ObstacleType("unknown"),
                                           initial_state=self.traj_pred.trajectory.state_at_time_step(0),
                                           prediction=self.set_pred, obstacle_shape=self.rectangle)
        self.dyn_traj_obs = DynamicObstacle(2, ObstacleType("unknown"),
                                            initial_state=self.traj_pred.trajectory.state_at_time_step(0),
                                            prediction=self.traj_pred, obstacle_shape=self.rectangle,
                                            initial_shape_lanelet_ids={100, 101})

        self.environment = Environment(Time(12, 15), TimeOfDay.NIGHT, Weather.SNOW, Underground.ICE)
        self.location = Location(geo_name_id=123, gps_latitude=456, gps_longitude=789, environment=self.environment)

        self.scenario = Scenario(0.1, 'test', location=self.location)

    def test_add_objects(self):

        expected_id_static_obs = self.static_obs.obstacle_id
        expected_id_dyn_set_obs = self.dyn_set_obs.obstacle_id
        expected_id_dyn_traj = self.dyn_traj_obs.obstacle_id
        expected_id_lanelet1 = self.lanelet1.lanelet_id
        expected_id_lanelet2 = self.lanelet2.lanelet_id

        self.scenario.add_objects(self.lanelet_network)
        self.scenario.add_objects(self.static_obs)
        self.scenario.add_objects(self.dyn_set_obs)
        self.scenario.add_objects(self.dyn_traj_obs)

        self.assertEqual(expected_id_static_obs, self.scenario.obstacles[0].obstacle_id)
        self.assertEqual(expected_id_dyn_set_obs, self.scenario.obstacles[1].obstacle_id)
        self.assertEqual(expected_id_dyn_traj, self.scenario.obstacles[2].obstacle_id)
        self.assertEqual(expected_id_lanelet1, self.scenario.lanelet_network.lanelets[0].lanelet_id)
        self.assertEqual(expected_id_lanelet2, self.scenario.lanelet_network.lanelets[1].lanelet_id)

        with self.assertRaises(ValueError):
            self.scenario.add_objects(self.rectangle)
            self.scenario.add_objects(self.static_obs)

    def test_remove_obstacle(self):
        expected_id_dyn_traj = self.dyn_set_obs.obstacle_id
        expected_id_lanelet1 = self.lanelet_network.lanelets[0].lanelet_id
        expected_id_lanelet2 = self.lanelet_network.lanelets[1].lanelet_id

        self.scenario.add_objects(self.lanelet_network)
        self.scenario.add_objects(self.static_obs)
        self.scenario.add_objects(self.dyn_traj_obs)
        self.scenario.add_objects(self.dyn_set_obs)

        self.scenario.remove_obstacle(self.static_obs)
        self.scenario.remove_obstacle(self.dyn_traj_obs)

        self.assertEqual(expected_id_dyn_traj,
                         self.scenario.obstacles[0].obstacle_id)
        self.assertEqual(expected_id_lanelet1,
                         self.scenario.lanelet_network.lanelets[0].lanelet_id)
        self.assertEqual(expected_id_lanelet2,
                         self.scenario.lanelet_network.lanelets[1].lanelet_id)

        with self.assertRaises(AssertionError):
            self.scenario.remove_obstacle(self.lanelet1)
            self.scenario.remove_obstacle(self.static_obs)

    def test_generate_object_id_empty(self):
        expected_generated_id = 1
        self.assertEqual(expected_generated_id,
                         self.scenario.generate_object_id())

    def test_generate_object_id_unique(self):
        expected_generated_id = 3

        self.scenario.add_objects(self.static_obs)
        self.scenario.add_objects(self.dyn_traj_obs)

        self.assertEqual(expected_generated_id,
                         self.scenario.generate_object_id())

        expected_generated_id = 4
        self.scenario.remove_obstacle([self.static_obs, self.dyn_traj_obs])
        # Ids should not be reused even when removing all objects
        self.assertEqual(expected_generated_id,
                         self.scenario.generate_object_id())

    def test_generate_object_id_positive(self):
        expected_generated_id = 3

        self.scenario.add_objects(self.static_obs)
        self.scenario.add_objects(self.dyn_traj_obs)

        self.assertEqual(expected_generated_id,
                         self.scenario.generate_object_id())

    def test_generate_object_id_negative(self):
        self.static_obs = StaticObstacle(-5, ObstacleType("unknown"), obstacle_shape=self.circ,
                                         initial_state=self.init_state)

        expected_generated_id = -4

        self.scenario.add_objects(self.static_obs)

        self.assertEqual(expected_generated_id, self.scenario.generate_object_id())

    def test_occupancies_at_time_step(self):
        self.scenario.add_objects(self.lanelet_network)
        self.scenario.add_objects(self.static_obs)
        self.scenario.add_objects(self.dyn_set_obs)

        expected_position_static_obs_time_step_0 = self.init_state.position
        expected_position_static_obs_time_step_1 = self.init_state.position
        expected_position_static_obs_time_step_2 = self.init_state.position
        expected_position_static_obs_time_step_3 = self.init_state.position
        expected_position_dyn_set_obs_time_step_0 = self.dyn_set_obs.initial_state.position
        expected_position_dyn_set_obs_time_step_1 = self.set_pred.occupancy_set[1].shape.center
        expected_position_dyn_set_obs_time_step_2 = self.set_pred.occupancy_set[2].shape.center
        expected_position_dyn_set_obs_time_step_3 = self.set_pred.occupancy_set[3].shape.center

        occupancy_at_0 = self.scenario.occupancies_at_time_step(0)
        occupancy_at_1 = self.scenario.occupancies_at_time_step(1)
        occupancy_at_2 = self.scenario.occupancies_at_time_step(2)
        occupancy_at_3 = self.scenario.occupancies_at_time_step(3)

        np.testing.assert_array_equal(expected_position_static_obs_time_step_0, occupancy_at_0[0].shape.center)
        np.testing.assert_array_equal(expected_position_static_obs_time_step_1, occupancy_at_1[0].shape.center)
        np.testing.assert_array_equal(expected_position_static_obs_time_step_2, occupancy_at_2[0].shape.center)
        np.testing.assert_array_equal(expected_position_static_obs_time_step_3, occupancy_at_3[0].shape.center)
        np.testing.assert_array_equal(expected_position_dyn_set_obs_time_step_0, occupancy_at_0[1].shape.center)
        np.testing.assert_array_equal(expected_position_dyn_set_obs_time_step_1, occupancy_at_1[1].shape.center)
        np.testing.assert_array_equal(expected_position_dyn_set_obs_time_step_2, occupancy_at_2[1].shape.center)
        np.testing.assert_array_equal(expected_position_dyn_set_obs_time_step_3, occupancy_at_3[1].shape.center)

    def test_obstacle_by_id(self):
        static_obs1 = StaticObstacle(-100, ObstacleType("unknown"), obstacle_shape=self.circ, initial_state=self.init_state)
        static_obs2 = StaticObstacle(0, ObstacleType("unknown"), obstacle_shape=self.circ, initial_state=self.init_state)
        static_obs3 = StaticObstacle(5000, ObstacleType("car"), obstacle_shape=self.circ, initial_state=self.init_state)
        dyn_set_obs1 = DynamicObstacle(20, ObstacleType("unknown"),
                                       initial_state=self.traj_pred.trajectory.state_at_time_step(0),
                                       prediction=self.set_pred, obstacle_shape=self.rectangle)
        dyn_set_obs2 = DynamicObstacle(-20, ObstacleType("unknown"),
                                       initial_state=self.traj_pred.trajectory.state_at_time_step(0),
                                       prediction=self.set_pred, obstacle_shape=self.rectangle)

        self.scenario.add_objects(static_obs1)
        self.scenario.add_objects(static_obs2)
        self.scenario.add_objects(static_obs3)
        self.scenario.add_objects(dyn_set_obs1)
        self.scenario.add_objects(dyn_set_obs2)

        self.assertEqual(static_obs1.__class__, self.scenario.obstacle_by_id(-100).__class__)
        self.assertEqual(static_obs1.obstacle_id, self.scenario.obstacle_by_id(-100).obstacle_id)
        self.assertEqual(static_obs1.obstacle_role, self.scenario.obstacle_by_id(-100).obstacle_role)
        self.assertEqual(static_obs1.obstacle_type, self.scenario.obstacle_by_id(-100).obstacle_type)
        np.testing.assert_array_almost_equal(static_obs1.initial_state.position, self.scenario.obstacle_by_id(-100).initial_state.position)
        self.assertEqual(static_obs2.__class__, self.scenario.obstacle_by_id(0).__class__)
        self.assertEqual(static_obs2.obstacle_id, self.scenario.obstacle_by_id(0).obstacle_id)
        self.assertEqual(static_obs2.obstacle_role, self.scenario.obstacle_by_id(0).obstacle_role)
        self.assertEqual(static_obs2.obstacle_type, self.scenario.obstacle_by_id(0).obstacle_type)
        np.testing.assert_array_almost_equal(static_obs2.initial_state.position,
                                             self.scenario.obstacle_by_id(0).initial_state.position)
        self.assertEqual(static_obs3.__class__, self.scenario.obstacle_by_id(5000).__class__)
        self.assertEqual(static_obs3.obstacle_id, self.scenario.obstacle_by_id(5000).obstacle_id)
        self.assertEqual(static_obs3.obstacle_role, self.scenario.obstacle_by_id(5000).obstacle_role)
        self.assertEqual(static_obs3.obstacle_type, self.scenario.obstacle_by_id(5000).obstacle_type)
        np.testing.assert_array_almost_equal(static_obs3.initial_state.position,
                                             self.scenario.obstacle_by_id(5000).initial_state.position)
        self.assertEqual(dyn_set_obs1.__class__, self.scenario.obstacle_by_id(20).__class__)
        self.assertEqual(dyn_set_obs1.obstacle_id, self.scenario.obstacle_by_id(20).obstacle_id)
        self.assertEqual(dyn_set_obs1.obstacle_role, self.scenario.obstacle_by_id(20).obstacle_role)
        self.assertEqual(dyn_set_obs1.obstacle_type, self.scenario.obstacle_by_id(20).obstacle_type)
        np.testing.assert_array_almost_equal(dyn_set_obs1.initial_state.position,
                                             self.scenario.obstacle_by_id(20).initial_state.position)
        self.assertEqual(dyn_set_obs2.__class__, self.scenario.obstacle_by_id(-20).__class__)
        self.assertEqual(dyn_set_obs2.obstacle_id, self.scenario.obstacle_by_id(-20).obstacle_id)
        self.assertEqual(dyn_set_obs2.obstacle_role, self.scenario.obstacle_by_id(-20).obstacle_role)
        self.assertEqual(dyn_set_obs2.obstacle_type, self.scenario.obstacle_by_id(-20).obstacle_type)
        np.testing.assert_array_almost_equal(dyn_set_obs2.initial_state.position,
                                             self.scenario.obstacle_by_id(-20).initial_state.position)

    def test_obstacles_by_role_and_type(self):
        static_obs1 = StaticObstacle(1, ObstacleType("unknown"), obstacle_shape=self.circ, initial_state=self.init_state)
        static_obs2 = StaticObstacle(2, ObstacleType("unknown"), obstacle_shape=self.circ, initial_state=self.init_state)
        static_obs3 = StaticObstacle(3, ObstacleType("car"), obstacle_shape=self.circ, initial_state=self.init_state)
        dyn_set_obs1 = DynamicObstacle(4, ObstacleType("unknown"),
                                           initial_state=self.traj_pred.trajectory.state_at_time_step(0),
                                           prediction=self.set_pred, obstacle_shape=self.rectangle)
        dyn_set_obs2 = DynamicObstacle(5, ObstacleType("car"),
                                           initial_state=self.traj_pred.trajectory.state_at_time_step(0),
                                           prediction=self.set_pred, obstacle_shape=self.rectangle)

        expected_obstacle_num_static_obstacles = 3
        expected_obstacle_num_dny_obstacles = 2
        expected_obstacle_num_obstacle_typ_one = 2
        expected_obstacle_num_obstacle_typ_one_and_dyn = 1

        self.scenario.add_objects([static_obs1,static_obs2,static_obs3, dyn_set_obs1, dyn_set_obs2])
        #self.scenario.add_objects(self.lanelet_network)

        output_one = self.scenario.obstacles_by_role_and_type(ObstacleRole.STATIC)
        output_two = self.scenario.obstacles_by_role_and_type(ObstacleRole.DYNAMIC)
        output_three = self.scenario.obstacles_by_role_and_type(None, ObstacleType("car"))
        output_four = self.scenario.obstacles_by_role_and_type(ObstacleRole.DYNAMIC, ObstacleType("car"))

        self.assertEqual(expected_obstacle_num_static_obstacles, len(output_one))
        self.assertEqual(expected_obstacle_num_dny_obstacles, len(output_two))
        self.assertEqual(expected_obstacle_num_obstacle_typ_one, len(output_three))
        self.assertEqual(expected_obstacle_num_obstacle_typ_one_and_dyn, len(output_four))

        self.scenario.remove_obstacle(dyn_set_obs1)
        self.scenario.remove_obstacle(dyn_set_obs2)

        expected_obstacle_num_obstacle_typ_one = 1

        output_five = self.scenario.obstacles_by_role_and_type(None, ObstacleType("car"))

        self.assertEqual(expected_obstacle_num_obstacle_typ_one, len(output_five))

    def test_obstacles_by_position_intervals(self):
        init_state1 = State(time_step=0, orientation=0, position=np.array([0, 0]))
        init_state2 = State(time_step=0, orientation=0, position=np.array([10, 10]))
        init_state3 = State(time_step=0, orientation=0, position=np.array([13, 13]))
        init_state4 = State(time_step=0, orientation=0, position=np.array([-13, -13]))
        static_obs1 = StaticObstacle(1, ObstacleType("unknown"), obstacle_shape=self.circ, initial_state=init_state1)
        static_obs2 = StaticObstacle(2, ObstacleType("unknown"), obstacle_shape=self.circ, initial_state=init_state2)
        static_obs3 = StaticObstacle(3, ObstacleType("car"), obstacle_shape=self.circ, initial_state=init_state3)
        static_obs4 = StaticObstacle(4, ObstacleType("car"), obstacle_shape=self.circ, initial_state=init_state4)
        dyn_set_obs1 = DynamicObstacle(5, ObstacleType("unknown"),
                                       initial_state=self.traj_pred.trajectory.state_at_time_step(0),
                                       prediction=self.set_pred, obstacle_shape=self.rectangle)

        expected_obstacle_ids_in_interval = [1, 2, 5]
        interval_x = Interval(-10, 10)
        interval_y = Interval(-10, 10)

        self.scenario.add_objects([static_obs1, static_obs2, static_obs3, static_obs4, dyn_set_obs1])

        obstacles_in_interval= self.scenario.obstacles_by_position_intervals([interval_x, interval_y])
        obstacle_ids_in_interval = []
        for obstacle in obstacles_in_interval:
            obstacle_ids_in_interval.append(obstacle.obstacle_id)

        np.testing.assert_array_equal(expected_obstacle_ids_in_interval, obstacle_ids_in_interval)

    def test_translate_rotate(self):
        rotation = np.pi
        translation = np.array([5.0, 5.0])
        expected_initial_state_position = np.array([-5.0, -5.0])

        self.scenario.add_objects(self.lanelet_network)
        self.scenario.add_objects(self.static_obs)

        self.scenario.translate_rotate(translation, rotation)

        np.testing.assert_array_almost_equal(expected_initial_state_position, self.scenario.obstacles[0].initial_state.position)

        with self.assertRaises(AssertionError):
            self.scenario.translate_rotate(np.array([2, -4]), 320)
        with self.assertRaises(AssertionError):
            self.scenario.translate_rotate(np.array([3, 5, -7]), np.pi / 2)
        with self.assertRaises(AssertionError):
            self.scenario.translate_rotate(np.array([3]), np.pi / 2)

    def test_is_object_id_used(self):
        static_obs1 = StaticObstacle(10, ObstacleType("unknown"), obstacle_shape=self.circ,
                                     initial_state=self.init_state, initial_shape_lanelet_ids={100, 101})
        static_obs2 = StaticObstacle(-10, ObstacleType("unknown"), obstacle_shape=self.circ,
                                     initial_state=self.init_state, initial_shape_lanelet_ids={100, 101})
        lanelet1 = Lanelet(np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]), np.array([[0.0, 1], [1.0, 1], [2, 1]]),
                                np.array([[0.0, 2], [1.0, 2], [2, 2]]), 100,
                                [101], [101], 101, False, 101, True,
                                LineMarking.DASHED, LineMarking.DASHED)

        lanelet2 = Lanelet(np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]), np.array([[0.0, 1], [1.0, 1], [2, 1]]),
                                np.array([[0.0, 2], [1.0, 2], [2, 2]]), 101,
                                [100], [100], 100, False, 100, True,
                                LineMarking.DASHED, LineMarking.DASHED)

        self.scenario.add_objects(lanelet1)

        self.assertTrue(self.scenario._is_object_id_used(100))

        self.scenario.add_objects(lanelet2)

        self.assertTrue(self.scenario._is_object_id_used(101))

        self.scenario.add_objects([static_obs1,static_obs2])

        self.assertTrue(self.scenario._is_object_id_used(10))
        self.assertTrue(self.scenario._is_object_id_used(-10))

        self.scenario.remove_obstacle(static_obs2)

        self.assertFalse(self.scenario._is_object_id_used(-10))

    def test_mark_object_id_as_used(self):
        lanelet1 = Lanelet(np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]), np.array([[0.0, 1], [1.0, 1], [2, 1]]),
                           np.array([[0.0, 2], [1.0, 2], [2, 2]]), 100,
                           [101], [101], 101, False, 101, True,
                           LineMarking.DASHED, LineMarking.DASHED)

        self.scenario.add_objects(self.static_obs)

        static_obs1 = StaticObstacle(-50, ObstacleType("unknown"), obstacle_shape=self.circ,
                                     initial_state=self.init_state)

        with self.assertRaises(ValueError):
            self.scenario._mark_object_id_as_used(0)

        self.scenario._mark_object_id_as_used(-50)

        with self.assertRaises(ValueError):
            self.scenario._mark_object_id_as_used(-50)
            self.scenario.add_objects(static_obs1)

        self.scenario.add_objects(lanelet1)

        with self.assertRaises(ValueError):
            self.scenario._mark_object_id_as_used(100)

    def test_obstacle_states_at_time_step(self):
        exp_states_time_zero = {1: self.dyn_set_obs.state_at_time(0), 2: self.dyn_traj_obs.state_at_time(0),
                                0: self.static_obs.initial_state}
        exp_states_time_one = {2: self.dyn_traj_obs.state_at_time(1), 0: self.static_obs.initial_state}
        self.scenario.add_objects(self.static_obs)
        self.scenario.add_objects(self.dyn_traj_obs)
        self.scenario.add_objects(self.dyn_set_obs)

        # test for time step 1
        self.assertEqual(len(exp_states_time_one), len(self.scenario.obstacle_states_at_time_step(1)))
        self.assertEqual(exp_states_time_one[2].velocity, self.scenario.obstacle_states_at_time_step(1)[2].velocity)
        self.assertEqual(exp_states_time_one[2].position[0],
                         self.scenario.obstacle_states_at_time_step(1)[2].position[0])
        self.assertEqual(exp_states_time_one[2].position[1],
                         self.scenario.obstacle_states_at_time_step(1)[2].position[1])
        self.assertEqual(exp_states_time_one[0].velocity, self.scenario.obstacle_states_at_time_step(1)[0].velocity)
        self.assertEqual(exp_states_time_one[0].position[0],
                         self.scenario.obstacle_states_at_time_step(1)[0].position[0])
        self.assertEqual(exp_states_time_one[0].position[1],
                         self.scenario.obstacle_states_at_time_step(1)[0].position[1])

        # test for time step zero
        self.assertEqual(len(exp_states_time_zero), len(self.scenario.obstacle_states_at_time_step(0)))
        self.assertEqual(exp_states_time_zero[1].velocity, self.scenario.obstacle_states_at_time_step(0)[1].velocity)
        self.assertEqual(exp_states_time_zero[1].position[0],
                         self.scenario.obstacle_states_at_time_step(0)[1].position[0])
        self.assertEqual(exp_states_time_zero[1].position[1],
                         self.scenario.obstacle_states_at_time_step(0)[1].position[1])
        self.assertEqual(exp_states_time_zero[2].velocity, self.scenario.obstacle_states_at_time_step(0)[2].velocity)
        self.assertEqual(exp_states_time_zero[2].position[0],
                         self.scenario.obstacle_states_at_time_step(0)[2].position[0])
        self.assertEqual(exp_states_time_zero[2].position[1],
                         self.scenario.obstacle_states_at_time_step(0)[2].position[1])
        self.assertEqual(exp_states_time_one[0].velocity, self.scenario.obstacle_states_at_time_step(1)[0].velocity)
        self.assertEqual(exp_states_time_one[0].position[0],
                         self.scenario.obstacle_states_at_time_step(1)[0].position[0])
        self.assertEqual(exp_states_time_one[0].position[1],
                         self.scenario.obstacle_states_at_time_step(1)[0].position[1])

    def test_location(self):
        self.environment = Environment(Time(12, 15), TimeOfDay.NIGHT, Weather.SNOW, Underground.ICE)
        self.location = Location(geo_name_id=123, gps_latitude=456, gps_longitude=789, environment=self.environment)
        exp_geo_name_id = 123
        exp_gps_latitude = 456
        exp_gps_longitude = 789
        exp_env_time_hours = 12
        exp_env_time_min = 15
        exp_env_time_of_day = TimeOfDay.NIGHT
        exp_env_weather = Weather.SNOW
        exp_env_underground = Underground.ICE

        self.assertEqual(exp_geo_name_id, self.scenario.location.geo_name_id)
        self.assertEqual(exp_gps_latitude, self.scenario.location.gps_latitude)
        self.assertEqual(exp_gps_longitude, self.scenario.location.gps_longitude)
        self.assertEqual(exp_env_time_hours, self.scenario.location.environment.time.hours)
        self.assertEqual(exp_env_time_min, self.scenario.location.environment.time.minutes)
        self.assertEqual(exp_env_time_of_day, self.scenario.location.environment.time_of_day)
        self.assertEqual(exp_env_weather, self.scenario.location.environment.weather)
        self.assertEqual(exp_env_underground, self.scenario.location.environment.underground)

    def test_assign_vehicles(self):
        states = list()
        states.append(State(time_step=0, orientation=0, position=np.array([1, .5]), velocity=5))
        states.append(State(time_step=1, orientation=0, position=np.array([1, .5]), velocity=10))
        trajectory = Trajectory(0, states)

        self.init_state = State(time_step=0, orientation=0, position=np.array([0, 0]), velocity=15)

        traj_pred = TrajectoryPrediction(trajectory, self.rectangle)
        dyn_traj_obs = DynamicObstacle(2, ObstacleType("unknown"),
                                            initial_state=traj_pred.trajectory.state_at_time_step(0),
                                            prediction=traj_pred, obstacle_shape=self.rectangle,
                                            initial_shape_lanelet_ids=None)
        sc = Scenario(dt=0.1, scenario_id='test')
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array(
            [[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5], [8, .5]])
        lanelet = Lanelet(right_vertices=right_vertices,left_vertices=left_vertices, center_vertices=center_vertices,lanelet_id=1)
        sc.add_objects([lanelet, dyn_traj_obs])
        scenario_tmp: Scenario = deepcopy(sc)
        # assign all time steps
        scenario_tmp.assign_obstacles_to_lanelets(time_steps=None)
        exp_dynamic_obstacles_on_lanelet_zero = {0: {2}, 1: {2}}
        exp_dynamic_lanelet_of_obstacle = {0: {1}, 1: {1}}
        self.assertEqual(exp_dynamic_obstacles_on_lanelet_zero,
                         scenario_tmp.lanelet_network.find_lanelet_by_id(1).dynamic_obstacles_on_lanelet)
        self.assertEqual(exp_dynamic_lanelet_of_obstacle[0],
                         scenario_tmp.obstacle_by_id(2).initial_center_lanelet_ids)
        self.assertEqual(exp_dynamic_lanelet_of_obstacle,
                         scenario_tmp.obstacle_by_id(2).prediction.center_lanelet_assignment)
        self.assertEqual(exp_dynamic_lanelet_of_obstacle,
                         scenario_tmp.obstacle_by_id(2).prediction.shape_lanelet_assignment)
        # assign one time step
        scenario_tmp: Scenario = deepcopy(sc)
        scenario_tmp.assign_obstacles_to_lanelets(time_steps=[1])
        exp_dynamic_obstacles_on_lanelet_zero = {1: {2}}
        exp_dynamic_lanelet_of_obstacle = {1: {1}}
        self.assertEqual(exp_dynamic_obstacles_on_lanelet_zero,
                         scenario_tmp.lanelet_network.find_lanelet_by_id(1).dynamic_obstacles_on_lanelet)
        self.assertEqual(None,
                         scenario_tmp.obstacle_by_id(2).initial_center_lanelet_ids)
        self.assertEqual(exp_dynamic_lanelet_of_obstacle,
                         scenario_tmp.obstacle_by_id(2).prediction.center_lanelet_assignment)
        self.assertEqual(exp_dynamic_lanelet_of_obstacle,
                         scenario_tmp.obstacle_by_id(2).prediction.shape_lanelet_assignment)
        # assign center only
        scenario_tmp: Scenario = deepcopy(sc)
        scenario_tmp.assign_obstacles_to_lanelets(time_steps=None, use_center_only=True)
        exp_dynamic_obstacles_on_lanelet_zero = {0: {2}, 1: {2}}
        exp_dynamic_lanelet_of_obstacle = {0: {1}, 1: {1}}
        self.assertEqual(exp_dynamic_obstacles_on_lanelet_zero,
                         scenario_tmp.lanelet_network.find_lanelet_by_id(1).dynamic_obstacles_on_lanelet)
        self.assertEqual(exp_dynamic_lanelet_of_obstacle[0],
                         scenario_tmp.obstacle_by_id(2).initial_center_lanelet_ids)
        self.assertEqual(exp_dynamic_lanelet_of_obstacle,
                         scenario_tmp.obstacle_by_id(2).prediction.center_lanelet_assignment)
        self.assertEqual(None,
                         scenario_tmp.obstacle_by_id(2).prediction.shape_lanelet_assignment)
        # assign only a selected obstacle
        scenario_tmp: Scenario = deepcopy(sc)
        dyn_traj_obs_3 = DynamicObstacle(3, ObstacleType("unknown"),
                                       initial_state=traj_pred.trajectory.state_at_time_step(0),
                                       prediction=traj_pred, obstacle_shape=self.rectangle,
                                       initial_shape_lanelet_ids=None)
        scenario_tmp.add_objects(dyn_traj_obs_3)
        scenario_tmp.assign_obstacles_to_lanelets(time_steps=None, obstacle_ids={3})
        exp_dynamic_obstacles_on_lanelet_zero = {0: {3}, 1: {3}}
        exp_dynamic_lanelet_of_obstacle = {0: {1}, 1: {1}}
        self.assertEqual(exp_dynamic_obstacles_on_lanelet_zero,
                         scenario_tmp.lanelet_network.find_lanelet_by_id(1).dynamic_obstacles_on_lanelet)
        self.assertEqual(None,
                         scenario_tmp.obstacle_by_id(2).initial_center_lanelet_ids)
        self.assertEqual(None,
                         scenario_tmp.obstacle_by_id(2).prediction.center_lanelet_assignment)
        self.assertEqual(None,
                         scenario_tmp.obstacle_by_id(2).prediction.shape_lanelet_assignment)
        self.assertEqual(exp_dynamic_lanelet_of_obstacle[0],
                         scenario_tmp.obstacle_by_id(3).initial_center_lanelet_ids)
        self.assertEqual(exp_dynamic_lanelet_of_obstacle,
                         scenario_tmp.obstacle_by_id(3).prediction.center_lanelet_assignment)
        self.assertEqual(exp_dynamic_lanelet_of_obstacle,
                         scenario_tmp.obstacle_by_id(3).prediction.shape_lanelet_assignment)


class TestScenarioID(unittest.TestCase):
    def test_from_benchmark_id(self):
        id_coop = "C-USA_US101-33_2_T-1"
        s_id = ScenarioID.from_benchmark_id(id_coop, SCENARIO_VERSION)
        self.assertEqual(id_coop, str(s_id))
        self.assertEqual(s_id.cooperative, True)
        self.assertEqual(s_id.country_id, "USA")
        self.assertEqual(s_id.map_name, "US101")
        self.assertEqual(s_id.map_id, 33)
        self.assertEqual(s_id.configuration_id, 2)
        self.assertEqual(s_id.prediction_type, 'T')
        self.assertEqual(s_id.prediction_id, 1)
        self.assertEqual(s_id.scenario_version, SCENARIO_VERSION)

        id_single = "USA_US101-33_2_T-1"
        s_id = ScenarioID.from_benchmark_id(id_single, SCENARIO_VERSION)
        self.assertEqual(id_single, str(s_id))
        self.assertEqual(s_id.cooperative, False)
        self.assertEqual(s_id.country_id, "USA")
        self.assertEqual(s_id.map_name, "US101")
        self.assertEqual(s_id.map_id, 33)
        self.assertEqual(s_id.configuration_id, 2)
        self.assertEqual(s_id.prediction_type, 'T')
        self.assertEqual(s_id.prediction_id, 1)
        self.assertEqual(s_id.scenario_version, SCENARIO_VERSION)

        id_no_pred = "USA_US101-33_2"
        s_id = ScenarioID.from_benchmark_id(id_no_pred, SCENARIO_VERSION)
        self.assertEqual(id_no_pred, str(s_id))
        self.assertEqual(s_id.cooperative, False)
        self.assertEqual(s_id.country_id, "USA")
        self.assertEqual(s_id.map_name, "US101")
        self.assertEqual(s_id.map_id, 33)
        self.assertEqual(s_id.configuration_id, 2)
        self.assertEqual(s_id.prediction_type, None)
        self.assertEqual(s_id.prediction_id, None)
        self.assertEqual(s_id.scenario_version, SCENARIO_VERSION)

        id_map_only = "USA_US101-33"
        s_id = ScenarioID.from_benchmark_id(id_map_only, SCENARIO_VERSION)
        self.assertEqual(id_map_only, str(s_id))
        self.assertEqual(s_id.cooperative, False)
        self.assertEqual(s_id.country_id, "USA")
        self.assertEqual(s_id.map_name, "US101")
        self.assertEqual(s_id.map_id, 33)
        self.assertEqual(s_id.configuration_id, None)
        self.assertEqual(s_id.prediction_type, None)
        self.assertEqual(s_id.prediction_id, None)
        self.assertEqual(s_id.scenario_version, SCENARIO_VERSION)

    # def test_read_all_files(self):
    #     folder = 'commonroad-scenarios/scenarios'
    #     from pathlib import Path
    #     files = list(Path(folder).rglob('*.xml'))
    #     for file in files:
    #         # if not "C-USA_Lanker-1_2_T-1" in str(file):
    #         #     continue
    #         sc, _ = CommonRoadFileReader(file).open()
    #         self.assertEqual(sc.orig_bid, str(sc.scenario_id))
    #         print(file)


if __name__ == '__main__':
    unittest.main()
