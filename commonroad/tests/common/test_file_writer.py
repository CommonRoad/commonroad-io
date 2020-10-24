import logging
import os
import unittest

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import *
from lxml import etree
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet, GoalRegion
from commonroad.prediction.prediction import *
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LineMarking, LaneletType
from commonroad.scenario.obstacle import *
from commonroad.scenario.scenario import Scenario, Tag, Location
from commonroad.scenario.trajectory import *


class TestFileWriter(unittest.TestCase):
    def setUp(self):
        self.cwd_path = os.path.dirname(os.path.abspath(__file__))
        self.xsd_path = self.cwd_path + "/../../common/XML_commonRoad_XSD.xsd"
        self.out_path = self.cwd_path + "/../.pytest_cache"
        self.filename_read_1 = self.cwd_path + "/test_reading_intersection_traffic_sign.xml"
        self.filename_read_2 = self.cwd_path + "/test_reading_all.xml"
        self.filename_2018b = self.cwd_path + "/USA_Lanker-1_1_T-1.xml"
        self.filename_invalid = self.cwd_path + "/test_writing_invalid.xml"
        if not os.path.isdir(self.out_path):
            os.makedirs(self.out_path)
        else:
            for (dirpath, dirnames, filenames) in os.walk(self.out_path):
                for file in filenames:
                    if file.endswith('.xml'):
                        os.remove(os.path.join(dirpath, file))

    def test_read_write_file(self):
        scenario_1, planning_problem_set_1 = CommonRoadFileReader(self.filename_read_1).open()
        filename = self.out_path + '/test_reading_intersection_traffic_sign.xml'
        CommonRoadFileWriter(scenario_1, planning_problem_set_1, scenario_1.author, scenario_1.affiliation,
                             'test', scenario_1.tags,
                             scenario_1.location).write_to_file(filename=filename,
                                                                overwrite_existing_file=OverwriteExistingFile.ALWAYS,
                                                                check_validity=False)
        assert self.validate_with_xsd(self.out_path + '/test_reading_intersection_traffic_sign.xml')

        scenario_2, planning_problem_set_2 = CommonRoadFileReader(self.filename_read_2).open()
        filename = self.out_path + '/test_reading_all.xml'
        CommonRoadFileWriter(scenario_2, planning_problem_set_2, scenario_2.author, scenario_2.affiliation,
                             'test', scenario_2.tags,
                             scenario_2.location).write_to_file(filename=filename,
                                                                overwrite_existing_file=OverwriteExistingFile.ALWAYS,
                                                                check_validity=False)
        assert self.validate_with_xsd(self.out_path + '/test_reading_all.xml')

    def test_read_write_2018b_file(self):
        scenario, planning_problem_set = CommonRoadFileReader(self.filename_2018b).open()
        filename = self.out_path + "/USA_Lanker-1_1_T-1.xml"
        CommonRoadFileWriter(scenario, planning_problem_set, scenario.author, scenario.affiliation,
                             str(scenario.scenario_id), scenario.tags,
                             scenario.location).write_to_file(filename=filename,
                                                              overwrite_existing_file=OverwriteExistingFile.ALWAYS,
                                                              check_validity=True)

        assert self.validate_with_xsd(self.out_path + "/USA_Lanker-1_1_T-1.xml")

    def test_write_invalid_lanelet_file(self):
        scenario, planning_problem_set = CommonRoadFileReader(self.filename_invalid).open()
        filename = self.out_path + "/USA_Lanker-1_1_T-1_invalid.xml"
        fw = CommonRoadFileWriter(scenario, planning_problem_set, scenario.author, scenario.affiliation,
                                str(scenario.scenario_id), scenario.tags,
                                scenario.location)

        self.assertRaises(ValueError,
            fw.write_to_file,
            filename=filename,
            overwrite_existing_file=OverwriteExistingFile.ALWAYS,
            check_validity=True)

    def test_write_valid_lanelet_file(self):
        scenario, planning_problem_set = CommonRoadFileReader(self.filename_read_2).open()
        filename = self.out_path + '/test_reading_all.xml'
        try:
            CommonRoadFileWriter(scenario, planning_problem_set, scenario.author, scenario.affiliation,
                                    str(scenario.scenario_id), scenario.tags,
                                    scenario.location).write_to_file(filename=filename,
                                                                    overwrite_existing_file=OverwriteExistingFile.ALWAYS,
                                                                    check_validity=True)
        except:
            self.fail('Error occurred during writing a valid lanelet file')


    def test_writing_shapes(self):
        rectangle = Rectangle(4.3, 8.9, center=np.array([2.5, -1.8]), orientation=1.7)
        polygon = Polygon(np.array([np.array((0.0, 0.0)), np.array((0.0, 1.0)), np.array((1.0, 1.0)),
                                    np.array((1.0, 0.0))]))
        circ = Circle(2.0, np.array([10.0, 0.0]))
        sg = ShapeGroup([circ, rectangle])
        occupancy_list = list()
        occupancy_list.append(Occupancy(0, rectangle))
        occupancy_list.append(Occupancy(1, circ))
        occupancy_list.append(Occupancy(2, polygon))
        occupancy_list.append(Occupancy(3, circ))


        set_pred = SetBasedPrediction(0,occupancy_list)

        states = list()
        states.append(State(time_step=0, orientation=0, position=np.array([0, 0])))
        states.append(State(time_step=1, orientation=0, position=np.array([0, 1])))
        trajectory = Trajectory(0, states)

        init_state = State(time_step=0, orientation=0, position=np.array([0, 0]))

        traj_pred = TrajectoryPrediction(trajectory, rectangle)

        static_obs = StaticObstacle(3,ObstacleType("unknown"), obstacle_shape=circ, initial_state=init_state)
        dyn_set_obs = DynamicObstacle(1,ObstacleType("unknown"),
                                      initial_state=traj_pred.trajectory.state_at_time_step(0),
                                      prediction=set_pred, obstacle_shape=rectangle)
        dyn_traj_obs = DynamicObstacle(2, ObstacleType("unknown"),
                                       initial_state=traj_pred.trajectory.state_at_time_step(0),
                                       prediction=traj_pred, obstacle_shape=rectangle)
        lanelet1 = Lanelet(np.array([[12345.12, 0.0], [1.0,0.0],[2,0]]), np.array([[0.0, 1],[1.0,1],[2,1]]),
                           np.array([[0.0, 2], [1.0,2],[2,2]]), 100, [101], [101], 101, False, 101, True,
                           LineMarking.DASHED, LineMarking.SOLID, lanelet_type={LaneletType.URBAN})
        lanelet2 = Lanelet(np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]), np.array([[0.0, 1], [1.0, 1], [2, 1]]),
                           np.array([[0.0, 2], [1.0, 2], [2, 2]]), 101,
                           [100], [100], 100, False, 100, True,
                           LineMarking.SOLID, LineMarking.DASHED, lanelet_type={LaneletType.URBAN})

        lanelet_network = LaneletNetwork().create_from_lanelet_list(list([lanelet1, lanelet2]))
        scenario = Scenario(0.1,'ZAM_test_0-1')
        scenario.add_objects([static_obs, lanelet_network])

        goal_region = GoalRegion([State(time_step=Interval(0,1),velocity=Interval(0.0,1),position=rectangle),
                                  State(time_step=Interval(1,2),velocity=Interval(0.0,1),position=circ)],
                                 {0:[100,101],1:[101]})
        planning_problem = PlanningProblem(1000, State(velocity=0.1, position=np.array([[0],[0]]),  orientation=0,
                                                       yaw_rate=0, slip_angle=0, time_step=0), goal_region)
        planning_problem_set = PlanningProblemSet(list([planning_problem]))

        filename = self.out_path + '/test_writing_shapes.xml'
        location = Location(2867714, 48.262333, 11.668775, None)

        CommonRoadFileWriter(scenario, planning_problem_set, 'PrinceOfZAM', 'TU Munich', 'test', {Tag.URBAN},
                             location).write_to_file(filename=filename,
                                                     overwrite_existing_file=OverwriteExistingFile.ALWAYS)
        assert self.validate_with_xsd(self.out_path + '/test_writing_shapes.xml')

        # test overwriting
        CommonRoadFileWriter(scenario, planning_problem_set, 'PrinceOfZAM', 'TU Munich', 'test', {Tag.URBAN},
                             location).write_to_file(filename=filename,
                                                     overwrite_existing_file=OverwriteExistingFile.SKIP)
        CommonRoadFileWriter(scenario, planning_problem_set, 'PrinceOfZAM', 'TU Munich', 'test', {Tag.URBAN},
                             location).write_to_file(filename=filename,
                                                     overwrite_existing_file=OverwriteExistingFile.ALWAYS)
        CommonRoadFileWriter(scenario, planning_problem_set, 'PrinceOfZAM_no_problem', 'TU Munich', 'test', {Tag.URBAN},
                             location).write_scenario_to_file(filename=filename,
                                                              overwrite_existing_file=OverwriteExistingFile.ALWAYS)

    def validate_with_xsd(self, xml_path: str) -> bool:
        xmlschema_doc = etree.parse(self.xsd_path)
        xmlschema = etree.XMLSchema(xmlschema_doc)

        xml_doc = etree.parse(xml_path)
        try:
            xmlschema.assert_(xml_doc)
            return True
        except Exception as e:
            logging.error('xml produced by file_writer not conformant with xsd-scheme: ' + str(e))
            return False

    # def test_all_scenarios(self):
    #     scenarios = "update"
    #     cooperative = scenarios + "/cooperative"
    #     hand_crafted = scenarios + "/hand-crafted"
    #     ngsim_lankershim = scenarios + "/NGSIM/Lankershim"
    #     ngsim_us101 = scenarios + "/NGSIM/US101"
    #     sumo = scenarios + "/SUMO"
    #     bicycle = scenarios + "/THI-Bicycle"
    #
    #     for scenario_name in os.listdir(hand_crafted):
    #         full_path = hand_crafted + "/" + scenario_name
    #         print(full_path)
    #         scenario_tmp, planning_problem_tmp = CommonRoadFileReader(full_path).open()
    #         CommonRoadFileWriter(scenario_tmp, planning_problem_tmp, 'PrinceOfZAM', 'TU Munich', 'unittest'). \
    #             write_to_file(filename=self.out_path + '/' + scenario_tmp.benchmark_id + ".xml",
    #                           overwrite_existing_file=OverwriteExistingFile.SKIP)
    #         assert self.validate_with_xsd(self.out_path + '/' + scenario_tmp.benchmark_id + ".xml")

        # for scenario_name in os.listdir(cooperative):
        #     full_path = cooperative + "/" + scenario_name
        #     print(full_path)
        #     scenario_tmp, planning_problem_tmp = CommonRoadFileReader(full_path).open()
        #     CommonRoadFileWriter(scenario_tmp, planning_problem_tmp, 'PrinceOfZAM', 'TU Munich', 'unittest'). \
        #         write_to_file(filename=self.out_path + '/' + scenario_tmp.benchmark_id + ".xml",
        #                       overwrite_existing_file=OverwriteExistingFile.SKIP)
        #     assert self.validate_with_xsd(self.out_path + '/' + scenario_tmp.benchmark_id + ".xml")

        # for scenario_name in os.listdir(ngsim_lankershim):
        #     full_path = ngsim_lankershim + "/" + scenario_name
        #     print(full_path)
        #     scenario_tmp, planning_problem_tmp = CommonRoadFileReader(full_path).open()
        #     CommonRoadFileWriter(scenario_tmp, planning_problem_tmp, 'PrinceOfZAM', 'TU Munich', 'unittest'). \
        #         write_to_file(filename=self.out_path + '/' + scenario_tmp.benchmark_id + ".xml",
        #                       overwrite_existing_file=OverwriteExistingFile.SKIP)
        #     assert self.validate_with_xsd(self.out_path + '/' + scenario_tmp.benchmark_id + ".xml")

        # for scenario_name in os.listdir(ngsim_us101):
        #     full_path = ngsim_us101 + "/" + scenario_name
        #     print(full_path)
        #     scenario_tmp, planning_problem_tmp = CommonRoadFileReader(full_path).open()
        #     CommonRoadFileWriter(scenario_tmp, planning_problem_tmp, 'PrinceOfZAM', 'TU Munich', 'unittest'). \
        #         write_to_file(filename=self.out_path + '/' + scenario_tmp.benchmark_id + ".xml",
        #                       overwrite_existing_file=OverwriteExistingFile.SKIP)
        #     assert self.validate_with_xsd(self.out_path + '/' + scenario_tmp.benchmark_id + ".xml")


        # for scenario_name in os.listdir(sumo):
        #     full_path = sumo + "/" + scenario_name
        #     print(full_path)
        #     scenario_tmp, planning_problem_tmp = CommonRoadFileReader(full_path).open()
        #     CommonRoadFileWriter(scenario_tmp, planning_problem_tmp, 'PrinceOfZAM', 'TU Munich', 'unittest'). \
        #         write_to_file(filename=self.out_path + '/' + scenario_tmp.benchmark_id + ".xml",
        #                       overwrite_existing_file=OverwriteExistingFile.SKIP)
        #     assert self.validate_with_xsd(self.out_path + '/' + scenario_tmp.benchmark_id + ".xml")

        # for scenario_name in os.listdir(bicycle):
        #     full_path = bicycle + "/" + scenario_name
        #     print(full_path)
        #     scenario_tmp, planning_problem_tmp = CommonRoadFileReader(full_path).open()
        #     CommonRoadFileWriter(scenario_tmp, planning_problem_tmp, 'PrinceOfZAM', 'TU Munich', 'unittest'). \
        #         write_to_file(filename=self.out_path + '/' + scenario_tmp.benchmark_id + ".xml",
        #                       overwrite_existing_file=OverwriteExistingFile.SKIP)
        #     assert self.validate_with_xsd(self.out_path + '/' + scenario_tmp.benchmark_id + ".xml")


if __name__ == '__main__':
    unittest.main()
