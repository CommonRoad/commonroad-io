import unittest
from commonroad.scenario.obstacle import *
from commonroad.prediction.prediction import *
from commonroad.scenario.state import InitialState, KSState


class TestObstacle(unittest.TestCase):
    def test_initialize_static_obstacle(self):
        obstacle_id = 100
        obstacle_type = ObstacleType.CONSTRUCTION_ZONE
        obstacle_role = ObstacleRole.STATIC
        shape = Rectangle(5.1, 2.6)

        static_obstacle = StaticObstacle(obstacle_id=obstacle_id, obstacle_type=obstacle_type,
                                         obstacle_shape=shape,
                                         initial_state=InitialState(**{'position': np.array([0, 0]),
                                                                       'orientation':  0}))

        np.testing.assert_equal(static_obstacle.obstacle_id, obstacle_id)
        self.assertEqual(obstacle_type, static_obstacle.obstacle_type)
        self.assertEqual(static_obstacle.obstacle_role, obstacle_role)
        np.testing.assert_equal(static_obstacle.obstacle_shape.vertices, shape.vertices)

    def test_initialize_dynamic_obstacle(self):
        initial_shape_lanelet_ids = {1, 2}
        initial_center_lanelet_ids = {3, 4}
        initial_signal_state = SignalState(braking_lights=True, time_step=0)
        signal_series = [SignalState(braking_lights=True, time_step=1), SignalState(indicator_right=True, time_step=2)]
        obstacle_id = 100
        obstacle_type = ObstacleType.CAR
        obstacle_role = ObstacleRole.DYNAMIC

        state_list = [InitialState(position=np.array([0.0, 0.0]), orientation=0.3, time_step=0),
                      KSState(position=np.array([1.0, 1.0]), orientation=0.3, time_step=1),
                      KSState(position=np.array([2.0, 1.0]), orientation=0.3, time_step=2)]
        trajectory = Trajectory(1, state_list[1::])
        shape = Rectangle(5.1, 2.6, np.array([0, 0]), 0)
        prediction = TrajectoryPrediction(trajectory, shape)
        initial_state = state_list[0]
        shape = Rectangle(5.1, 2.6, np.array([0, 0]), 0)
        initial_meta_information_state = MetaInformationState(meta_data_int={'1': 1, "2": 2})
        another_meta_information_state = MetaInformationState(meta_data_bool={'1': True, "2": False})
        meta_information_series = [initial_meta_information_state, another_meta_information_state]
        external_dataset_id = 3

        dynamic_obstacle = DynamicObstacle(obstacle_id=obstacle_id, obstacle_type=obstacle_type, prediction=prediction,
                                           initial_state=initial_state, obstacle_shape=shape,
                                           initial_shape_lanelet_ids=initial_shape_lanelet_ids,
                                           initial_center_lanelet_ids=initial_center_lanelet_ids,
                                           initial_signal_state=initial_signal_state, signal_series=signal_series,
                                           initial_meta_information_state=initial_meta_information_state,
                                           meta_information_series=meta_information_series,
                                           external_dataset_id=external_dataset_id)
        np.testing.assert_equal(dynamic_obstacle.obstacle_id, obstacle_id)
        self.assertEqual(obstacle_type, dynamic_obstacle.obstacle_type)
        self.assertEqual(dynamic_obstacle.obstacle_role, obstacle_role)
        self.assertEqual(dynamic_obstacle.prediction.trajectory.state_list, state_list[1::])
        self.assertEqual(dynamic_obstacle.initial_state, initial_state)
        self.assertEqual(dynamic_obstacle.initial_shape_lanelet_ids, initial_shape_lanelet_ids)
        self.assertEqual(dynamic_obstacle.initial_center_lanelet_ids, initial_center_lanelet_ids)
        self.assertEqual(dynamic_obstacle.initial_signal_state, initial_signal_state)
        self.assertEqual(dynamic_obstacle.signal_series, signal_series)
        self.assertEqual(dynamic_obstacle.initial_meta_information_state, initial_meta_information_state)
        self.assertEqual(dynamic_obstacle.meta_information_series, meta_information_series)
        self.assertEqual(dynamic_obstacle.external_dataset_id, external_dataset_id)
        np.testing.assert_equal(dynamic_obstacle.obstacle_shape.vertices, shape.vertices)

    def test_signal_state_at_time_step(self):
        initial_signal_state = SignalState(braking_lights=True, time_step=0)
        signal_series = [SignalState(braking_lights=True, time_step=1), SignalState(indicator_right=True, time_step=2)]
        obstacle_id = 100
        obstacle_type = ObstacleType.CAR

        state_list = [KSState(position=np.array([1.0, 1.0]), orientation=0.3, time_step=1)]
        trajectory = Trajectory(1, state_list)
        shape = Rectangle(5.1, 2.6, np.array([0, 0]), 0)
        prediction = TrajectoryPrediction(trajectory, shape)
        initial_state = InitialState(position=np.array([0.0, 0.0]), orientation=0.3, time_step=0)
        shape = Rectangle(5.1, 2.6, np.array([0, 0]), 0)

        dynamic_obstacle1 = DynamicObstacle(obstacle_id=obstacle_id, obstacle_type=obstacle_type, prediction=prediction,
                                            initial_state=initial_state, obstacle_shape=shape,
                                            initial_signal_state=initial_signal_state, signal_series=signal_series)
        dynamic_obstacle2 = DynamicObstacle(obstacle_id=obstacle_id, obstacle_type=obstacle_type, prediction=prediction,
                                            initial_state=initial_state, obstacle_shape=shape)
        self.assertEqual(dynamic_obstacle1.signal_state_at_time_step(1), signal_series[0])
        self.assertEqual(dynamic_obstacle2.signal_state_at_time_step(1), None)

    def test_transform_static_obstacle(self):
        rectangle = Rectangle(4.3, 8.9, np.array((2.5, - 1.8)), 1.7)
        shift = np.array([- 5.6, 12.6])
        angle = -0.3
        desired_rectangle: Rectangle = rectangle.translate_rotate(shift, angle)
        obstacle = StaticObstacle(obstacle_id=1, obstacle_type=ObstacleType.BICYCLE, obstacle_shape=Rectangle(4.3, 8.9),
                                  initial_state=InitialState(**{'position': np.array((2.5, - 1.8)),
                                                                'orientation': 1.7}))
        obstacle.translate_rotate(shift, angle)
        output_rectangle: Rectangle = obstacle.occupancy_at_time(0).shape
        self.assertAlmostEqual(desired_rectangle.length, obstacle.obstacle_shape.length)
        self.assertAlmostEqual(desired_rectangle.width, obstacle.obstacle_shape.width)
        self.assertAlmostEqual(desired_rectangle.center[0], output_rectangle.center[0])
        self.assertAlmostEqual(desired_rectangle.center[1], output_rectangle.center[1])
        self.assertAlmostEqual(desired_rectangle.orientation, output_rectangle.orientation)

    def test_transform_dynamic_obstacle(self):
        rect = Rectangle(5.1, 2.6)
        state_list = [KSState(position=np.array([0.0, 0.0]), orientation=0.3, time_step=1),
                      KSState(position=np.array([1.0, 1.0]), orientation=0.3, time_step=2),
                      KSState(position=np.array([2.0, 1.0]), orientation=0.3, time_step=3)]
        trajectory = Trajectory(1, state_list)
        prediction = TrajectoryPrediction(trajectory, rect)

        # without initial_state
        dynamic_obs = DynamicObstacle(obstacle_id=30, obstacle_type=ObstacleType.PARKED_VEHICLE,
                                      prediction=prediction,
                                      initial_state=InitialState(**{'position': np.array([0, 0]), 'orientation':  0,
                                                                    'time_step': 0}),
                                      obstacle_shape=rect)

        # translation
        dynamic_obs.translate_rotate(np.array([10, 0]), 0.0)
        # obstacle_shape must not be transformed
        np.testing.assert_array_almost_equal(dynamic_obs.obstacle_shape.center, np.array([0, 0]))

        # check transformation of statelist
        target_statelist = [[10, 0], [10, 0], [11, 1], [12, 1]]
        for i, target_state in enumerate(target_statelist):
            state = dynamic_obs.state_at_time(i)
            np.testing.assert_array_almost_equal(state.position, target_state)

        # check transformation of occupancies
        for i in range(3):
            print(dynamic_obs.occupancy_at_time(i).shape.center, target_statelist[i])
            np.testing.assert_array_almost_equal(dynamic_obs.occupancy_at_time(i).shape.center, target_statelist[i])

        # rotation
        dynamic_obs.translate_rotate(np.array([0, 0]), -0.3)
        self.assertEqual(dynamic_obs.obstacle_shape.orientation, 0.0)
        self.assertAlmostEqual(dynamic_obs.prediction.trajectory.state_list[2].orientation, 0.0)

    def test_get_occupancy_static_obstacle(self):
        initial_state = InitialState(**{'position': np.array([10.1, 5.1]), 'orientation': 0.33})
        rect = Rectangle(5.1, 2.6)
        static_obs = StaticObstacle(obstacle_id=30, obstacle_type=ObstacleType.PARKED_VEHICLE,
                                    obstacle_shape=rect, initial_state=initial_state)
        time_step = 10

        occupancy = static_obs.occupancy_at_time(time_step)
        assert occupancy.time_step == time_step
        np.testing.assert_array_almost_equal(occupancy.shape.center, initial_state.position)

    def test_get_occupanciy_dynamic_obstacle(self):
        rect = Rectangle(5.1, 2.6)
        state_list = [KSState(position=np.array([0.0, 0.0]), orientation=0.3, time_step=0),
                      KSState(position=np.array([0.0, 1.0]), orientation=0.3, time_step=1),
                      KSState(position=np.array([1.0, 1.0]), orientation=0.3, time_step=2),
                      KSState(position=np.array([2.0, 1.0]), orientation=0.3, time_step=3)]
        trajectory = Trajectory(1, state_list[1:])
        prediction = TrajectoryPrediction(trajectory, rect)

        dynamic_obs = DynamicObstacle(
            obstacle_id=30, obstacle_type=ObstacleType.PARKED_VEHICLE, initial_state=KSState(**{'position': np.array(
                [0, 0]), 'orientation': 0.0, 'time_step': 0}), obstacle_shape=rect, prediction=prediction)

        for i in range(5):
            if i <= 3:
                np.testing.assert_array_equal(dynamic_obs.occupancy_at_time(i).shape.center, state_list[i].position)
            else:
                assert dynamic_obs.occupancy_at_time(i) is None

    def test_state_at_time(self):
        rect = Rectangle(5.1, 2.6)
        state_list = [KSState(position=np.array([0.0, 0.0]), orientation=0.3, time_step=0),
                      KSState(position=np.array([0.0, 1.0]), orientation=0.3, time_step=1),
                      KSState(position=np.array([1.0, 1.0]), orientation=0.3, time_step=2),
                      KSState(position=np.array([2.0, 1.0]), orientation=0.3, time_step=3)]
        initial_state = InitialState(**{'position': np.array([0, 0]), 'orientation': 0.0, 'time_step': 0})
        trajectory = Trajectory(1, state_list[1:])
        prediction = TrajectoryPrediction(trajectory, rect)

        dynamic_obs = DynamicObstacle(
            obstacle_id=30, obstacle_type=ObstacleType.CAR, initial_state=initial_state,
            obstacle_shape=rect, prediction=prediction)

        for i in range(5):
            if i == 0:
                np.testing.assert_array_equal(dynamic_obs.state_at_time(i).position, initial_state.position)
                self.assertEqual(dynamic_obs.state_at_time(i).orientation, initial_state.orientation)
            elif 1 <= i <= 3:
                np.testing.assert_array_equal(dynamic_obs.state_at_time(i).position, state_list[i].position)
                self.assertEqual(dynamic_obs.state_at_time(i).orientation, state_list[i].orientation)
            else:
                assert dynamic_obs.state_at_time(i) is None

    def test_environmental_obstacle(self):
        environmental_obstacle_shape = Polygon(np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [4, 2], [3, 1],
                                                         [2, 1], [1, 1]]))
        environmental_obstacle_id = 1234
        environmental_obstacle_type = ObstacleType.BUILDING
        environmental_obstacle = EnvironmentObstacle(environmental_obstacle_id, environmental_obstacle_type,
                                                     environmental_obstacle_shape)

        self.assertEqual(environmental_obstacle.obstacle_id, environmental_obstacle_id)
        self.assertEqual(environmental_obstacle.obstacle_role, ObstacleRole.ENVIRONMENT)
        self.assertEqual(environmental_obstacle.obstacle_type, environmental_obstacle_type)
        np.testing.assert_array_almost_equal(environmental_obstacle_shape.vertices,
                                             environmental_obstacle.obstacle_shape.vertices)

    def test_phantom_obstacle(self):
        s1 = Rectangle(3, 10)
        s2 = Circle(4, np.array([2.0, 1.0]))
        s3 = ShapeGroup([s1, s2])
        self.occ1 = Occupancy(2, s1)
        self.occ2 = Occupancy(3, s3)
        sp = SetBasedPrediction(2, [self.occ1, self.occ2])

        phantom_obstacle_id = 1234
        phantom_obstacle_role = ObstacleRole.Phantom
        phantom_obstacle = PhantomObstacle(obstacle_id=phantom_obstacle_id, prediction=sp)

        self.assertEqual(phantom_obstacle.obstacle_id, phantom_obstacle_id)
        self.assertEqual(phantom_obstacle.obstacle_role, phantom_obstacle_role)
        np.testing.assert_array_almost_equal(s1.vertices, phantom_obstacle.occupancy_at_time(2).shape.vertices)
        np.testing.assert_array_almost_equal(s3.shapes[0].vertices,
                                             phantom_obstacle.occupancy_at_time(3).shape.shapes[0].vertices)
        np.testing.assert_array_almost_equal(s3.shapes[1].center,
                                             phantom_obstacle.occupancy_at_time(3).shape.shapes[1].center)


if __name__ == '__main__':
    unittest.main()
