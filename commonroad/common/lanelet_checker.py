from commonroad.scenario.lanelet import LaneletNetwork
import enum
import numpy as np
from typing import Dict, Set, Tuple

__author__ = "Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW CAR@TUM"]
__version__ = "2020.3"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "release"


class LaneletCheckerErrorCode(enum.Enum):
    """
    Enum describing different types of line markings, i.e. dashed or solid lines
    """
    ERROR_1 = 1  # Predecessor ID does not exist
    ERROR_2 = 2  # Successor ID does not exist
    ERROR_3 = 3  # found unexpected predecessors
    ERROR_4 = 4  # found unexpected successor
    ERROR_5 = 5  # Predecessor vertices do not match
    ERROR_6 = 6  # Successor vertices do not match
    ERROR_7 = 7  # Successor/Predecessor relations missing
    ERROR_8 = 8  # Self-reference predecessor
    ERROR_9 = 9  # Self-reference successor


def check_successor_predecessor_relationships(lanelet_network: LaneletNetwork,
                                              errors: Dict[int, Set[Tuple[LaneletCheckerErrorCode, int]]] = None) \
        -> Dict[int, Set[Tuple[LaneletCheckerErrorCode, int]]]:
    """
    Evaluates whether all successor and predecessor relationships are correct.
    Note that the error appearance is not disjoint.

    :param lanelet_network: scenario lanelet network to check
    :param errors: already existing dictionary with lanelet IDs
    and corresponding error codes with error causing lanelet reference
    :return: dictionary with error codes for each lanelet
    """
    if errors is None:
        errors = {}
    lanelets = lanelet_network.lanelets
    all_lanelet_ids = [ln.lanelet_id for ln in lanelets]

    for la_1 in lanelets:
        errors[la_1.lanelet_id] = set()
        # check if preceding lanelets exist at all and print output if not
        for pred_id in la_1.predecessor:
            if pred_id not in all_lanelet_ids:
                errors[la_1.lanelet_id].add((LaneletCheckerErrorCode.ERROR_1, pred_id))
                print('ERROR 1 - Lanelet', la_1.lanelet_id, 'predecessor', pred_id, 'does not exist')
            else:  # check if vertices match
                if not (np.array_equal(la_1.left_vertices[0],
                                       lanelet_network.find_lanelet_by_id(pred_id).left_vertices[-1])
                        and np.array_equal(la_1.right_vertices[0],
                                           lanelet_network.find_lanelet_by_id(pred_id).right_vertices[-1])):
                    errors[la_1.lanelet_id].add((LaneletCheckerErrorCode.ERROR_5, pred_id))
                    print('ERROR 5 - Lanelet', la_1.lanelet_id, 'predecessor', pred_id, 'does not match')

        # check if succeeding lanelets exist at all and print output if not
        for suc_id in la_1.successor:
            if suc_id not in all_lanelet_ids:
                errors[la_1.lanelet_id].add((LaneletCheckerErrorCode.ERROR_2, suc_id))
                print('ERROR 2 - Lanelet', la_1.lanelet_id, 'successor', suc_id, 'does not exist')
            else:  # check if vertices match
                if not (np.array_equal(la_1.left_vertices[-1],
                                       lanelet_network.find_lanelet_by_id(suc_id).left_vertices[0])
                        and np.array_equal(la_1.right_vertices[-1],
                                           lanelet_network.find_lanelet_by_id(suc_id).right_vertices[0])):
                    errors[la_1.lanelet_id].add((LaneletCheckerErrorCode.ERROR_6, suc_id))
                    print('ERROR 6 - Lanelet', la_1.lanelet_id, 'successor', suc_id, 'does not match')

        # check if relationship to preceding lanelets is correct
        # preceding lanelet needs to have current lanelet as successor
        pred_id_matches = [x.lanelet_id for x in lanelets if la_1.lanelet_id in x.successor]
        passed_predecessor_validation = True
        if not la_1.predecessor:
            # current lanelet expects no predecessors
            if pred_id_matches:
                # found unexpected predecessors
                passed_predecessor_validation = False
        else:
            if not sorted(la_1.predecessor) == sorted(pred_id_matches):
                # expected predecessors for current lanelet do not match found predecessors
                passed_predecessor_validation = False
        # print output if defects in predecessor-successor relationships were found
        if not passed_predecessor_validation:
            errors[la_1.lanelet_id].add((LaneletCheckerErrorCode.ERROR_3, tuple(set(pred_id_matches))))
            print('ERROR 3 - Lanelet', la_1.lanelet_id, 'expected predecessors', la_1.predecessor,
                  ', but found', pred_id_matches)

        # check if relationship to succeeding lanelets is correct
        # succeeding lanelet needs to have current lanelet as predecessor
        suc_id_matches = [x.lanelet_id for x in lanelets if la_1.lanelet_id in x.predecessor]
        passed_successor_validation = True
        if not la_1.successor:
            # no successors expected
            if suc_id_matches:
                # found unexpected successors
                passed_successor_validation = False
        else:
            if not sorted(la_1.successor) == sorted(suc_id_matches):
                # expected successors do not match found successors
                passed_successor_validation = False
        # print output if defects in predecessor-successor relationships were found
        if not passed_successor_validation:
            errors[la_1.lanelet_id].add((LaneletCheckerErrorCode.ERROR_4, tuple(set(suc_id_matches))))
            print('ERROR 4 - Lanelet', la_1.lanelet_id, 'expected successors', la_1.successor,
                  ', but found', suc_id_matches)

        # check all lanelets if their should be a relationship, but it is not modeled
        for la_2 in lanelets:
            if np.array_equal(la_1.left_vertices[-1], la_2.left_vertices[0]) and \
                    np.array_equal(la_1.right_vertices[-1], la_2.right_vertices[0]) \
                    and la_2.lanelet_id not in la_1.successor and la_1.lanelet_id not in la_2.predecessor:
                errors[la_1.lanelet_id].add((LaneletCheckerErrorCode.ERROR_7, la_2.lanelet_id))
                print('ERROR 7 - Lanelet', la_1.lanelet_id, 'and', la_2.lanelet_id, 'successor/predecessor '
                                                                                    'relation missing')

        # check for self-references
        if la_1.lanelet_id in la_1.predecessor:
            errors[la_1.lanelet_id].add((LaneletCheckerErrorCode.ERROR_8, la_1.lanelet_id))
            print('ERROR 8 - Lanelet', la_1.lanelet_id, 'predecessor', 'self-reference')
        if la_1.lanelet_id in la_1.successor:
            errors[la_1.lanelet_id].add((LaneletCheckerErrorCode.ERROR_9, la_1.lanelet_id))
            print('ERROR 9 - Lanelet', la_1.lanelet_id, 'successor', 'self-reference')
    return errors


def exist_successor_predecessor_relationships(error_summary: Dict[int, Set[Tuple[LaneletCheckerErrorCode, int]]]) \
        -> bool:
    """
    Checks whether there are successor and predecessor errors in the error summary

    :param error_summary: dictionary with lanelet IDs
    and corresponding error codes with error causing lanelet reference
    :return: bool indicating whether there are successor and predecessor errors
    """
    suc_pre_error_codes = {LaneletCheckerErrorCode.ERROR_1.value, LaneletCheckerErrorCode.ERROR_2.value,
                           LaneletCheckerErrorCode.ERROR_3.value, LaneletCheckerErrorCode.ERROR_4.value,
                           LaneletCheckerErrorCode.ERROR_5.value, LaneletCheckerErrorCode.ERROR_6.value,
                           LaneletCheckerErrorCode.ERROR_7.value, LaneletCheckerErrorCode.ERROR_8.value,
                           LaneletCheckerErrorCode.ERROR_9.value}
    for error_set in error_summary.values():
        if any(er[0] in suc_pre_error_codes for er in error_set):
            return True
    return False


def repair_successor_predecessor_relationships(lanelet_network: LaneletNetwork,
                                               error_summary: Dict[int, Set[Tuple[LaneletCheckerErrorCode, int]]]) \
        -> LaneletNetwork:
    """
    Repairs successor and predecessor relationship errors in lanelet network

    :param lanelet_network: scenario lanelet network to check
    :param error_summary: dictionary with lanelet IDs
    and corresponding error codes with error causing lanelet reference
    :return: repaired lanelet network
    """
    while exist_successor_predecessor_relationships(error_summary):
        for l_id, error_set in error_summary.items():
            if LaneletCheckerErrorCode.ERROR_1 in error_set:
                lanelet_network.find_lanelet_by_id(l_id).predecessor.remove(22)
                break
        error_summary = check_successor_predecessor_relationships(lanelet_network)

    print(error_summary)


def check_adjacency_relationships(lanelet_network: LaneletNetwork) -> bool:
    """
    Evaluates whether all adjacency relationships are correct

    :param lanelet_network: scenario lanelet network to check
    :return: boolean indicating satisfaction
    """
    print('----- Running Adjacency Validation -----')

    adj_relationships_correct = True
    lanelets = lanelet_network.lanelets
    lanelets.sort(key=lambda l: l.lanelet_id)
    all_lanelet_ids = [la.lanelet_id for la in lanelets]

    for _, curr_lnl in enumerate(lanelets):
        # check if neighboring lanelets exist at all and print output if not
        if curr_lnl.adj_left and curr_lnl.adj_left not in all_lanelet_ids:
            adj_relationships_correct = False
            print('ERROR 1 - Lanelet', curr_lnl.lanelet_id, 'left neighbor', curr_lnl.adj_left, 'does not exist\n')
        if curr_lnl.adj_right and curr_lnl.adj_right not in all_lanelet_ids:
            adj_relationships_correct = False
            print('ERROR 2 - Lanelet', curr_lnl.lanelet_id, 'right neighbor', curr_lnl.adj_right, 'does not exist\n')

        if curr_lnl.adj_left == curr_lnl.lanelet_id:
            adj_relationships_correct = False
            print('ERROR 3 - Lanelet', curr_lnl.lanelet_id, 'neighbors itself to the left.')

        if curr_lnl.adj_right == curr_lnl.lanelet_id:
            adj_relationships_correct = False
            print('ERROR 4 - Lanelet', curr_lnl.lanelet_id, 'neighbors itself to the right.')

        # check if current lanelet's left neighbor, if existent, adjacency matches
        if curr_lnl.adj_left and curr_lnl.adj_left in all_lanelet_ids:
            left_neighbor = lanelet_network.find_lanelet_by_id(curr_lnl.adj_left)
            if curr_lnl.adj_left_same_direction:
                if not left_neighbor.adj_right == curr_lnl.lanelet_id:
                    adj_relationships_correct = False
                    print("ERROR 5 - Lanelet", curr_lnl.lanelet_id, "'s left equally-directed neighbor",
                          left_neighbor.lanelet_id, "'s right neighbor is", left_neighbor.adj_right)

                else:
                    pass

                # check the left neighbor's right neighbor's direction is equally-directed
                if not left_neighbor.adj_right_same_direction:
                    adj_relationships_correct = False
                    print("ERROR 6 - Lanelet", curr_lnl.lanelet_id, "'s left equally-directed neighbor",
                          left_neighbor.lanelet_id, "'s right neighbor is opposite-directed")
                else:
                    pass

                # check the left neighbor's left neighbor is not current lanelet as well
                if left_neighbor.adj_left == curr_lnl.lanelet_id:
                    adj_relationships_correct = False
                    print("ERROR 7 - Lanelet", curr_lnl.lanelet_id, "'s left equally-directed neighbor",
                          left_neighbor.lanelet_id, "'s left neighbor is Lanelet", curr_lnl.lanelet_id)
                else:
                    pass

            else:
                if not left_neighbor.adj_left == curr_lnl.lanelet_id:
                    adj_relationships_correct = False
                    print("ERROR 8 - Lanelet", curr_lnl.lanelet_id, "'s left opposite-directed neighbor",
                          left_neighbor.lanelet_id, "'s left neighbor is", left_neighbor.adj_left)
                else:
                    pass

                # check the left neighbor's right neighbor's direction is opposite-directed
                if left_neighbor.adj_left_same_direction:
                    adj_relationships_correct = False
                    print("ERROR 9 - Lanelet", curr_lnl.lanelet_id, "'s left opposite-directed neighbor",
                          left_neighbor.lanelet_id, "'s left neighbor is equally-directed")
                else:
                    pass

                # check the left neighbor's right neighbor is not current lanelet as well
                if left_neighbor.adj_right == curr_lnl.lanelet_id:
                    adj_relationships_correct = False
                    print("ERROR 10 - Lanelet", curr_lnl.lanelet_id, "'s left opposite-directed neighbor",
                          left_neighbor.lanelet_id, "'s right neighbor is Lanelet", curr_lnl.lanelet_id)
                else:
                    pass

        # check if current lanelet's right neighbor, if existent, adjacency matches
        if curr_lnl.adj_right and curr_lnl.adj_right in all_lanelet_ids:
            right_neighbor = lanelet_network.find_lanelet_by_id(curr_lnl.adj_right)
            if curr_lnl.adj_right_same_direction:
                if not right_neighbor.adj_left == curr_lnl.lanelet_id:
                    adj_relationships_correct = False
                    print("ERROR 11 - Lanelet", curr_lnl.lanelet_id, "'s right equally-directed neighbor",
                          right_neighbor.lanelet_id, "'s left neighbor is", right_neighbor.adj_left)
                else:
                    pass

                # check the right neighbor's left neighbor's direction is equally-directed
                if not right_neighbor.adj_left_same_direction:
                    adj_relationships_correct = False
                    print("ERROR 12 - Lanelet", curr_lnl.lanelet_id, "'s right equally-directed neighbor",
                          right_neighbor.lanelet_id, "'s left neighbor is opposite-directed")
                else:
                    pass

                # check the right neighbor's right neighbor is not current lanelet as well
                if right_neighbor.adj_right == curr_lnl.lanelet_id:
                    adj_relationships_correct = False
                    print("ERROR 13 - Lanelet", curr_lnl.lanelet_id, "'s right equally-directed neighbor",
                          right_neighbor.lanelet_id, "'s right neighbor is Lanelet", curr_lnl.lanelet_id)
                else:
                    pass

            else:
                if not right_neighbor.adj_right == curr_lnl.lanelet_id:
                    adj_relationships_correct = False
                    print("ERROR 14 - Lanelet", curr_lnl.lanelet_id, "'s right opposite-directed neighbor",
                          right_neighbor.lanelet_id, "'s right neighbor is", right_neighbor.adj_right)
                else:
                    print("to validate: PASS 10 - Lanelet", curr_lnl.lanelet_id, "'s right opposite-directed neighbor",
                          right_neighbor.lanelet_id, "'s left neighbor is", right_neighbor.adj_left)

                # check the right neighbor's right neighbor's direction is opposite-directed
                if right_neighbor.adj_right_same_direction:
                    adj_relationships_correct = False
                    print("ERROR 15 - Lanelet", curr_lnl.lanelet_id, "'s right opposite-directed neighbor",
                          right_neighbor.lanelet_id, "'s right neighbor is equally-directed")
                else:
                    adj_relationships_correct = False
                    print("to validate: PASS 11 - Lanelet", curr_lnl.lanelet_id, "'s right opposite-directed neighbor",
                          right_neighbor.lanelet_id, "'s right neighbor is opposite-directed")

                # check the right neighbor's left neighbor is not current lanelet as well
                if right_neighbor.adj_left == curr_lnl.lanelet_id:
                    adj_relationships_correct = False
                    print("ERROR 16 - Lanelet", curr_lnl.lanelet_id, "'s right opposite-directed neighbor",
                          right_neighbor.lanelet_id, "'s left neighbor is Lanelet", curr_lnl.lanelet_id)
                else:
                    print("to validate: PASS 12 - Lanelet", curr_lnl.lanelet_id, "'s right opposite-directed neighbor",
                          right_neighbor.lanelet_id, "'s left neighbor is", right_neighbor.adj_left,
                          "and not Lanelet", curr_lnl.lanelet_id)

        # check for other lanelets that are claiming a relationship to the current lanelet that is not expected
        # do not check lanelet against itself, left neighbor and right neighbor
        for other_lanelet in lanelets:
            if not other_lanelet.lanelet_id == curr_lnl.lanelet_id and \
                    not other_lanelet.lanelet_id == curr_lnl.adj_left and \
                    not other_lanelet.lanelet_id == curr_lnl.adj_right:
                if other_lanelet.adj_left == curr_lnl.lanelet_id:
                    adj_relationships_correct = False
                    print('ERROR 17 - Lanelet', curr_lnl.lanelet_id, 'gets unexpected relationship from',
                          other_lanelet.lanelet_id)
                if other_lanelet.adj_right == curr_lnl.lanelet_id:
                    adj_relationships_correct = False
                    print('ERROR 18 - Lanelet', curr_lnl.lanelet_id, 'gets unexpected relationship from',
                          other_lanelet.lanelet_id)
    print('----- Finished Adjacency Validation -----\n\n')
    return adj_relationships_correct


def check_lanelet_network(lanelet_network: LaneletNetwork) -> None:
    """
    Check a CommonRoad scenario's lanelet network for errors

    :param lanelet_network: scenario lanelet network to check
    :return: None
    """

    print('===========================================')
    print('===== RUNNING LANELET-NETWORK CHECKER =====')
    print('===========================================')
    print('\n')
    status_suc_pre = check_successor_predecessor_relationships(lanelet_network)
    # validity_status_adjacencies = check_adjacencies_relationships(lanelet_network)
    for key, value in status_suc_pre.items():
        if value != set():
            raise ValueError


def repair_relationships(lanelet_network: LaneletNetwork) -> LaneletNetwork:
    """
    Repairs

    :param lanelet_network: scenario lanelet network to repair
    :return: repaired lanelet network
    """
    # repair relationships
    list_ids_lanelets = [lanelet_network.find_lanelet_by_id(id_lanelet) for la in list_ids_lanelets]
    dict_relations = dict()
    for id in list_ids_lanelets:
        dict_relations[id] = {"p": set(), "s": set(),
                              "left_same": set(), "right_same": set(),
                              "left_diff": set(), "right_diff": set()}

    for lanelet1 in lanelet_network_new.lanelets:
        for lanelet2 in lanelet_network_new.lanelets:
            id1 = lanelet1.lanelet_id
            id2 = lanelet2.lanelet_id

            if id1 == id2: continue

            # check predecessor-successors
            if np.array_equal(lanelet1.left_vertices[-1], lanelet2.left_vertices[0]) and \
                    np.array_equal(lanelet1.right_vertices[-1], lanelet2.right_vertices[0]):
                dict_relations[id1]["s"].add(id2)
                dict_relations[id2]["p"].add(id1)

            # check adjacencies
            # some adjacent lanelets don't share exact amount of vertices!
            cnt_equal = 0
            for v1 in lanelet1.left_vertices:
                for v2 in lanelet2.right_vertices:
                    if np.array_equal(v1, v2): cnt_equal += 1

            if cnt_equal >= 2:
                dict_relations[id1]["left_same"].add(id2)
                dict_relations[id2]["right_same"].add(id1)

            cnt_equal = 0
            for v1 in lanelet1.left_vertices:
                for v2 in lanelet2.left_vertices:
                    if np.array_equal(v1, v2): cnt_equal += 1

            if cnt_equal >= 2:
                dict_relations[id1]["left_diff"].add(id2)
                dict_relations[id2]["left_diff"].add(id1)

            cnt_equal = 0
            for v1 in lanelet1.right_vertices:
                for v2 in lanelet2.right_vertices:
                    if np.array_equal(v1, v2): cnt_equal += 1

            if cnt_equal >= 2:
                dict_relations[id1]["right_diff"].add(id2)
                dict_relations[id2]["right_diff"].add(id1)
            # how it should be, but not applicable in this case
    #         if np.array_equal(lanelet1.left_vertices, lanelet2.right_vertices):
    #             dict_relations[id1]["left_same"].add(id2)
    #             dict_relations[id2]["right_same"].add(id1)

    #         if np.array_equal(lanelet1.left_vertices, lanelet2.left_vertices):
    #             dict_relations[id1]["left_diff"].add(id2)
    #             dict_relations[id2]["left_diff"].add(id1)

    #         if np.array_equal(lanelet1.right_vertices, lanelet2.right_vertices):
    #             dict_relations[id1]["right_diff"].add(id2)
    #             dict_relations[id2]["right_diff"].add(id1)

    # %%

    # reconstruct new network
    list_lanelet_repaired = []

    for lanelet in lanelet_network_new.lanelets:
        id_lanelet = lanelet.lanelet_id

        list_lanelets_adjacent_left = list(dict_relations[id_lanelet]["left_same"]) + list(
            dict_relations[id_lanelet]["left_diff"])
        if len(list_lanelets_adjacent_left):
            adjacent_left = list_lanelets_adjacent_left[0]
        else:
            adjacent_left = None

        list_lanelets_adjacent_right = list(dict_relations[id_lanelet]["right_same"]) + list(
            dict_relations[id_lanelet]["right_diff"])
        if len(list_lanelets_adjacent_right):
            adjacent_right = list_lanelets_adjacent_right[0]
        else:
            adjacent_right = None

        if len(list(dict_relations[id_lanelet]["left_same"]) + list(dict_relations[id_lanelet]["left_diff"])):
            if list(dict_relations[id_lanelet]["left_same"]):
                dir_left_same = True
            else:
                dir_left_same = False
        else:
            dir_left_same = None

        if len(list(dict_relations[id_lanelet]["right_same"]) + list(dict_relations[id_lanelet]["right_diff"])):
            if list(dict_relations[id_lanelet]["right_same"]):
                dir_right_same = True
            else:
                dir_right_same = False
        else:
            dir_right_same = None

        list_lanelet_repaired.append(Lanelet(left_vertices=lanelet.left_vertices,
                                             center_vertices=lanelet.center_vertices,
                                             right_vertices=lanelet.right_vertices,
                                             lanelet_id=lanelet.lanelet_id,
                                             predecessor=list(dict_relations[id_lanelet]["p"]),
                                             successor=list(dict_relations[id_lanelet]["s"]),
                                             adjacent_left=adjacent_left,
                                             adjacent_left_same_direction=dir_left_same,
                                             adjacent_right=adjacent_right,
                                             adjacent_right_same_direction=dir_right_same))
    lanelet_network_final = LaneletNetwork.create_from_lanelet_list(list_lanelet_repaired)
