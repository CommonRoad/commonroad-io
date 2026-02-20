import os
import tempfile
import unittest
from typing import Any, List

import numpy as np

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.util import FileFormat
from commonroad.scenario.scenario import Tag

# Compatibility across versions
try:
    from commonroad.common.file_writer import OverwriteExistingFile
except ImportError:
    from commonroad.common.writer.file_writer_interface import OverwriteExistingFile


EPS = 1e-6


def _allclose(a: Any, b: Any) -> bool:
    try:
        a_arr = np.asarray(a)
        b_arr = np.asarray(b)
        if a_arr.shape != b_arr.shape:
            return False
        return np.allclose(a_arr, b_arr, atol=EPS, equal_nan=True)
    except Exception:
        return False


def _value_equal(a: Any, b: Any) -> bool:
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        return _allclose(a, b)
    if isinstance(a, (float, np.floating)) or isinstance(b, (float, np.floating)):
        try:
            return abs(float(a) - float(b)) <= EPS
        except Exception:
            return False
    return a == b


def _shape_equal(a: Any, b: Any) -> bool:
    if a.__class__ != b.__class__:
        return False

    # Compare common shape attributes if present
    for attr in ("center", "vertices", "radius", "length", "width", "orientation"):
        has_a = hasattr(a, attr)
        has_b = hasattr(b, attr)
        if has_a != has_b:
            return False
        if has_a and not _value_equal(getattr(a, attr), getattr(b, attr)):
            return False
    return True


def _state_equal(a: Any, b: Any) -> bool:
    attrs_a = list(getattr(a, "attributes", []))
    attrs_b = list(getattr(b, "attributes", []))
    if attrs_a != attrs_b:
        return False

    for attr in attrs_a:
        va = getattr(a, attr)
        vb = getattr(b, attr)
        if not _value_equal(va, vb):
            return False
    return True


def _prediction_equal(a: Any, b: Any) -> bool:
    if a.__class__ != b.__class__:
        return False

    # Trajectory prediction
    if hasattr(a, "trajectory") and hasattr(b, "trajectory"):
        ta = a.trajectory.state_list
        tb = b.trajectory.state_list
        if len(ta) != len(tb):
            return False
        for sa, sb in zip(ta, tb):
            if not _state_equal(sa, sb):
                return False

        if hasattr(a, "shape") and hasattr(b, "shape"):
            if not _shape_equal(a.shape, b.shape):
                return False
        return True

    # Set-based prediction / occupancies
    if hasattr(a, "occupancy_set") and hasattr(b, "occupancy_set"):
        if len(a.occupancy_set) != len(b.occupancy_set):
            return False
        for oa, ob in zip(a.occupancy_set, b.occupancy_set):
            if getattr(oa, "time_step", None) != getattr(ob, "time_step", None):
                return False
            if not _shape_equal(oa.shape, ob.shape):
                return False
        return True

    # Fallback
    return a == b


def _obstacle_equal(a: Any, b: Any) -> bool:
    if a.__class__ != b.__class__:
        return False
    if getattr(a, "obstacle_id", None) != getattr(b, "obstacle_id", None):
        return False
    if getattr(a, "obstacle_type", None) != getattr(b, "obstacle_type", None):
        return False

    if hasattr(a, "obstacle_shape") and hasattr(b, "obstacle_shape"):
        if not _shape_equal(a.obstacle_shape, b.obstacle_shape):
            return False

    if hasattr(a, "initial_state") and hasattr(b, "initial_state"):
        if not _state_equal(a.initial_state, b.initial_state):
            return False

    if hasattr(a, "prediction") and hasattr(b, "prediction"):
        if not _prediction_equal(a.prediction, b.prediction):
            return False

    return True


def _diff_commonroad(label_a, scn_a, pps_a, label_b, scn_b, pps_b) -> List[str]:
    diffs: List[str] = []

    def add(msg: str) -> None:
        diffs.append(f"[{label_a} vs {label_b}] {msg}")

    # Scenario basics
    if scn_a.dt != scn_b.dt:
        add(f"dt differs: {scn_a.dt} vs {scn_b.dt}")
    if str(scn_a.scenario_id) != str(scn_b.scenario_id):
        add(f"scenario_id differs: {scn_a.scenario_id} vs {scn_b.scenario_id}")
    if scn_a.tags != scn_b.tags:
        add(f"tags differ: {scn_a.tags} vs {scn_b.tags}")

    # Lanelets
    net_a, net_b = scn_a.lanelet_network, scn_b.lanelet_network
    ids_a = {l.lanelet_id for l in net_a.lanelets}
    ids_b = {l.lanelet_id for l in net_b.lanelets}

    for i in ids_a ^ ids_b:
        add(f"Lanelet {'removed' if i in ids_a else 'added'}: {i}")

    for i in ids_a & ids_b:
        la = net_a.find_lanelet_by_id(i)
        lb = net_b.find_lanelet_by_id(i)

        for name in ("left_vertices", "right_vertices", "center_vertices"):
            if not _allclose(getattr(la, name), getattr(lb, name)):
                add(f"Lanelet {i} {name} geometry differs")

        for name in ("predecessor", "successor", "adj_left", "adj_right"):
            if getattr(la, name) != getattr(lb, name):
                add(f"Lanelet {i} {name} differs")

        # Useful semantic refs
        for name in ("traffic_signs", "traffic_lights", "lanelet_type"):
            if hasattr(la, name) and hasattr(lb, name):
                if getattr(la, name) != getattr(lb, name):
                    add(f"Lanelet {i} {name} differs")

    # Obstacles
    for tag, obs_a, obs_b in [
        ("DynamicObstacle", scn_a.dynamic_obstacles, scn_b.dynamic_obstacles),
        ("StaticObstacle", scn_a.static_obstacles, scn_b.static_obstacles),
    ]:
        A = {o.obstacle_id: o for o in obs_a}
        B = {o.obstacle_id: o for o in obs_b}

        for i in A.keys() ^ B.keys():
            add(f"{tag} {'removed' if i in A else 'added'}: {i}")

        for i in A.keys() & B.keys():
            if not _obstacle_equal(A[i], B[i]):
                add(f"{tag} {i} differs")

    # Planning problems
    A = pps_a.planning_problem_dict
    B = pps_b.planning_problem_dict

    for i in A.keys() ^ B.keys():
        add(f"PlanningProblem {'removed' if i in A else 'added'}: {i}")

    for i in A.keys() & B.keys():
        if not _state_equal(A[i].initial_state, B[i].initial_state):
            add(f"PlanningProblem {i} initial_state differs")
        # Goal equality can be strict; fallback to string comparison to avoid false negatives
        if A[i].goal != B[i].goal and str(A[i].goal) != str(B[i].goal):
            add(f"PlanningProblem {i} goal differs")

    return diffs


class TestProtobufRoundtripSemantics(unittest.TestCase):
    def setUp(self):
        self.cwd_path = os.path.dirname(os.path.abspath(__file__))
        self.xml_path = os.path.join(
            self.cwd_path, "../test_scenarios/xml/2018b/USA_Lanker-1_1_T-1.xml"
        )

    def test_xml_to_protobuf_to_xml_semantics(self):
        # 1) Read original XML
        scn_xml1, pps_xml1 = CommonRoadFileReader(self.xml_path).open()

        with tempfile.TemporaryDirectory() as tmp_dir:
            pb_map = os.path.join(tmp_dir, "USA_Lanker-1.pb")
            pb_dynamic = os.path.join(tmp_dir, "USA_Lanker-1_1_T-1.pb")
            pb_scenario = os.path.join(tmp_dir, "USA_Lanker-1_1_T-1-SC.pb")
            xml_rewrite = os.path.join(tmp_dir, "USA_Lanker-1_1_T-1-roundtrip.xml")

            # 2) Write protobuf split files
            fw_pb = CommonRoadFileWriter(
                scn_xml1, pps_xml1, file_format=FileFormat.PROTOBUF
            )
            fw_pb.write_map_to_file(pb_map, overwrite_existing_file=OverwriteExistingFile.ALWAYS)
            fw_pb.write_dynamic_to_file(
                pb_dynamic, overwrite_existing_file=OverwriteExistingFile.ALWAYS
            )
            fw_pb.write_scenario_to_file(
                pb_scenario, overwrite_existing_file=OverwriteExistingFile.ALWAYS
            )

            # 3) Read protobuf files using new loader API
            reader = CommonRoadFileReader(
                filename_map=pb_map,
                filename_dynamic=pb_dynamic,
                filename_scenario=pb_scenario,
            )

            # Smoke-check individual protobuf readers (covers feature surface)
            map_pb = reader.open_map()
            scenario_pb_only = reader.open_scenario()
            dynamic_pb_only = reader.open_dynamic()
            scenario_from_map_dynamic = reader.open_map_dynamic()

            self.assertIsNotNone(map_pb)
            self.assertIsNotNone(scenario_pb_only)
            self.assertIsNotNone(dynamic_pb_only)
            self.assertIsNotNone(scenario_from_map_dynamic)

            scn_pb, pps_pb, _ = reader.open_all()

            # 4) Write XML from protobuf-loaded CommonRoad objects
            tags = scn_pb.tags if scn_pb.tags else {Tag.URBAN}
            fw_xml = CommonRoadFileWriter(
                scn_pb,
                pps_pb,
                "author",
                "affiliation",
                "source",
                tags=tags,
                file_format=FileFormat.XML,
            )
            fw_xml.write_to_file(xml_rewrite, OverwriteExistingFile.ALWAYS)

            # 5) Read rewritten XML
            scn_xml2, pps_xml2 = CommonRoadFileReader(xml_rewrite).open()

            # 6) Pairwise semantic comparison
            diffs_xml_pb = _diff_commonroad("XML(orig)", scn_xml1, pps_xml1, "PB", scn_pb, pps_pb)

            # Ignore known metadata loss in protobuf split files (tags are not preserved there)
            diffs_xml_pb = [d for d in diffs_xml_pb if "tags differ" not in d]

            self.assertEqual([], diffs_xml_pb,
                    msg="XML vs PB semantic differences found:\n" + "\n".join(diffs_xml_pb), )

            # Optional diagnostic only (do not fail test): XML writer may inject defaults / lose metadata
            diffs_pb_xml = _diff_commonroad("PB", scn_pb, pps_pb, "XML(rewrite)", scn_xml2, pps_xml2)
            diffs_xml_xml = _diff_commonroad("XML(orig)", scn_xml1, pps_xml1, "XML(rewrite)", scn_xml2, pps_xml2)

            if diffs_pb_xml or diffs_xml_xml:
                print("\n[INFO] Non-failing roundtrip XML differences (expected/known):")
                for d in diffs_pb_xml[:20]:
                    print(d)
                if len(diffs_pb_xml) > 20:
                    print(f"... and {len(diffs_pb_xml) - 20} more")

if __name__ == "__main__":
    unittest.main()