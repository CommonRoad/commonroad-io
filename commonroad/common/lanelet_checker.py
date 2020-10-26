from commonroad.scenario.lanelet import LaneletNetwork

__author__ = "Fabian Höltke, Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["CAR@TUM"]
__version__ = "2020.2"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "release"


def check_adjacencies_relationships(lanelet_network: LaneletNetwork) -> bool:
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
            if not other_lanelet.lanelet_id == curr_lnl.lanelet_id and\
                    not other_lanelet.lanelet_id == curr_lnl.adj_left and\
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


def check_successor_predecessor_relationships(lanelet_network: LaneletNetwork) -> bool:
    """
    Evaluates whether all successor and predecessor relationships are correct

    :param lanelet_network: scenario lanelet network to check
    :return: boolean indicating satisfaction
    """
    print('----- Running Successor-Predecessor Validation -----')
    suc_pre_relationships_correct = True
    lanelets = lanelet_network.lanelets
    all_lanelet_ids = [ln.lanelet_id for ln in lanelets]

    for curr_lnl in lanelets:
        # check if preceeding/succeeding lanelets exist at all and print output if not
        for pred_id in curr_lnl.predecessor:
            if pred_id not in all_lanelet_ids:
                suc_pre_relationships_correct = False
                print('ERROR 1 - Lanelet', curr_lnl.lanelet_id, 'precedessor', pred_id, 'does not exist\n')
        for pred_id in curr_lnl.successor:
            if pred_id not in all_lanelet_ids:
                suc_pre_relationships_correct = False
                print('ERROR 2 - Lanelet', curr_lnl.lanelet_id, 'successor', pred_id, 'does not exist\n')

        # check if relationship to preceeding lanelets is correct
        # preceding lanelet needs to have current lanelet as successor
        pred_id_matches = [x.lanelet_id for x in lanelets if curr_lnl.lanelet_id in x.successor]
        passed_predecessor_validation = True
        if not curr_lnl.predecessor:
            # current lanelet expects no predecessors
            if pred_id_matches:
                # found unexpected predecessors
                passed_predecessor_validation = False
        else:
            if not sorted(curr_lnl.predecessor) == sorted(pred_id_matches):
                # expected predecessors for current lanelet do not match found predecessors
                passed_predecessor_validation = False

        # check if relationship to succeeding lanelets is correct
        # succeeding lanelet needs to have current lanelet as predecessor
        succ_id_matches = [x.lanelet_id for x in lanelets if curr_lnl.lanelet_id in x.predecessor]
        passed_successor_validation = True
        if not curr_lnl.successor:
            # no successors expected
            if succ_id_matches:
                # found unexpected successors
                passed_successor_validation = False
        else:
            if not sorted(curr_lnl.successor) == sorted(succ_id_matches):
                # expected successors do not match found successors
                passed_successor_validation = False

        # print output if defects in predecessor-successor relationships were found
        if not passed_predecessor_validation:
            suc_pre_relationships_correct = False
            print('ERROR 3 - Lanelet', curr_lnl.lanelet_id, 'expected predecessors', curr_lnl.predecessor,
                  ', but found', pred_id_matches)

        if not passed_successor_validation:
            suc_pre_relationships_correct = False
            print('ERROR 4 - Lanelet', curr_lnl.lanelet_id, 'expected successors', curr_lnl.successor,
                  ', but found', succ_id_matches)

    print('----- Finished Successor-Predecessor Validation-----\n\n')
    return suc_pre_relationships_correct


def check_lanelets_for_at_least_two_vertices(lanelet_network: LaneletNetwork) -> bool:
    """
    Evaluates whether all successor and predecessor relationships are correct

    :param lanelet_network: scenario lanelet network to check
    :return: boolean indicating satisfaction
    """
    print('----- Running Check For At Least Two Left And Right Vertices -----')
    min_num_vertices = True
    n = 0
    for curr_lnl in lanelet_network.lanelets:
        left_vertices = curr_lnl.left_vertices
        right_vertices = curr_lnl.right_vertices
        if len(left_vertices) <= 2 or len(right_vertices) <= 2:
            n += 1
    if n > 0:
        min_num_vertices = False
        print('ERROR - There are', n, 'Lanelets with less then 3 vertices')
    print('----- Finished Check For At Least Two Left And Right Vertices -----\n\n')

    return min_num_vertices


def check_lanelets_for_id_uniqueness(lanelet_network: LaneletNetwork) -> bool:
    """
    Evaluates whether all lanelet IDs are unique

    :param lanelet_network: scenario lanelet network to check
    :return: boolean indicating satisfaction
    """
    print('----- Running Lanelet ID Uniqueness Validation-----')
    ids_unique = True
    all_lanelet_ids = sorted([la.lanelet_id for la in lanelet_network.lanelets])
    all_lanelet_ids_set = sorted(list(set(all_lanelet_ids)))
    if not all_lanelet_ids == all_lanelet_ids_set:
        ids_unique = False
        print('ERROR - Lanelet IDs are not unique')
    print('----- Finished Lanelet ID Uniqueness Validation -----\n\n')
    return ids_unique


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
    validity_status_id_uniqueness = check_lanelets_for_id_uniqueness(lanelet_network)
    validity_status_at_least_two_vertices = check_lanelets_for_at_least_two_vertices(lanelet_network)
    validity_status_suc_pre = check_successor_predecessor_relationships(lanelet_network)
    validity_status_adjacencies = check_adjacencies_relationships(lanelet_network)

    if not (validity_status_id_uniqueness and validity_status_at_least_two_vertices and validity_status_suc_pre
            and validity_status_adjacencies):
        raise ValueError
