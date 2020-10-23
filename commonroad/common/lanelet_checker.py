import sys
import matplotlib.pyplot as plt
from prettytable import PrettyTable
from math import isclose

from commonroad.visualization.draw_dispatch_cr import draw_object


def draw_lanelet_network_section_for_id_list(lanelet_network, id_list_tagged):
    draw_params_not_tagged = {'lanelet': {'draw_left_bound': True,
                                   'draw_right_bound': True,
                                   'draw_center_bound': False,
                                   'draw_border_vertices': False,  # does not work? - object has not attribute 'extend'
                                   'draw_start_and_direction': False,
                                   'show_label': False,
                                   'draw_linewidth': 0.5,
                                   'fill_lanelet': False,
                                   }}

    draw_params_tagged = {'lanelet': {'draw_left_bound': True,
                                      'draw_right_bound': True,
                                      'draw_center_bound': True,
                                      'draw_border_vertices': False,  # does not work? - object has not attribute 'extend'
                                      'draw_start_and_direction': True,
                                      'show_label': True,
                                      'draw_linewidth': 0.5,
                                      'fill_lanelet': True,
                                      }}

    lanelets_to_plot_with_tag = [l for l in lanelet_network.lanelets if l.lanelet_id in id_list_tagged]
    lanelets_to_plot_without_tag = lanelet_network.lanelets

    # determine plot limits
    x_min = y_min = 999999999999999
    x_max = y_max = -99999999999999
    for lnl in lanelets_to_plot_with_tag:
        for lv_l, lv_r in zip(lnl.left_vertices, lnl.right_vertices):
            x_min = min([x_min, lv_l[0], lv_r[0]])
            x_max = max([x_max, lv_l[0], lv_r[0]])
            y_min = min([y_min, lv_l[1], lv_r[1]])
            y_max = max([y_max, lv_l[1], lv_r[1]])
    c = 30
    limits = [x_min - c, x_max + c, y_min - c, y_max + c]

    plt.figure(figsize=(25, 10))
    draw_object(lanelets_to_plot_without_tag, draw_params=draw_params_not_tagged, plot_limits=limits)
    draw_object(lanelets_to_plot_with_tag, draw_params=draw_params_tagged, plot_limits=limits)
    plt.gca().set_aspect('equal')
    plt.show()
    return


def print_and_show_adjacencies_for_id_list(lanelet_network, ids):
    lanelets_to_print_and_show = [l for l in lanelet_network.lanelets if l.lanelet_id in ids]

    t = PrettyTable(['ADJACENT LEFT', 'CENTER', 'ADJACENT RIGHT'])
    for lanelet in lanelets_to_print_and_show:
        t.add_row([lanelet.adj_left, lanelet.lanelet_id, lanelet.adj_right])
    print(t)
    print()

    ids_to_plot_with_tag = [l.lanelet_id for l in lanelets_to_print_and_show]

    draw_lanelet_network_section_for_id_list(lanelet_network, ids_to_plot_with_tag)
    return


def print_and_show_succession_for_id_list(lanelet_network, ids):
    lanelets_to_print_and_show = [l for l in lanelet_network.lanelets if l.lanelet_id in ids]

    t = PrettyTable(['PREDECESSORS', 'MIDDLE', 'SUCCESSORS'])
    for lnl in lanelets_to_print_and_show:
        t.add_row([lnl.predecessor, lnl.lanelet_id, lnl.successor])
    print(t)
    print()

    ids_to_plot_with_tag = [l.lanelet_id for l in lanelets_to_print_and_show]

    draw_lanelet_network_section_for_id_list(lanelet_network, ids_to_plot_with_tag)
    return


def check_adjacencies_relationships(lanelet_network):
    print('----- Running Adjacency Validation -----')

    lanelets = lanelet_network.lanelets
    lanelets.sort(key=lambda l: l.lanelet_id)
    all_lanelet_ids = [l.lanelet_id for l in lanelets]

    for i,curr_lnl in enumerate(lanelets):
        # check if neighboring lanelets exist at all and print output if not
        if curr_lnl.adj_left and curr_lnl.adj_left not in all_lanelet_ids:
            print('ERROR 1 - Lanelet', curr_lnl.lanelet_id, 'left neighbor', curr_lnl.adj_left, 'does not exist\n')
        if curr_lnl.adj_right and curr_lnl.adj_right not in all_lanelet_ids:
            print('ERROR 2 - Lanelet', curr_lnl.lanelet_id, 'right neighbor', curr_lnl.adj_right, 'does not exist\n')

        if curr_lnl.adj_left == curr_lnl.lanelet_id:
            print('ERROR 3 - Lanelet', curr_lnl.lanelet_id, 'neighbors itself to the left.')

        if curr_lnl.adj_right == curr_lnl.lanelet_id:
            print('ERROR 4 - Lanelet', curr_lnl.lanelet_id, 'neighbors itself to the right.')

        # check if current lanelet's left neighbor, if existent, adjacency matches
        if curr_lnl.adj_left and curr_lnl.adj_left in all_lanelet_ids:
            left_neighbor = lanelet_network.find_lanelet_by_id(curr_lnl.adj_left)
            if curr_lnl.adj_left_same_direction:
                if not left_neighbor.adj_right == curr_lnl.lanelet_id:
                    print("ERROR 5 - Lanelet", curr_lnl.lanelet_id, "'s left equally-directed neighbor", left_neighbor.lanelet_id, "'s right neighbor is", left_neighbor.adj_right)
                    print_and_show_adjacencies_for_id_list(lanelet_network, [left_neighbor.lanelet_id, left_neighbor.adj_right, curr_lnl.lanelet_id])
                else:
                    pass # print("PASS 1 - Lanelet", curr_lnl.lanelet_id, "'s left equally-directed neighbor", left_neighbor.lanelet_id, "'s right neighbor is", left_neighbor.adj_right)

                # check the left neighbor's right neighbor's direction is equally-directed
                if not left_neighbor.adj_right_same_direction:
                    print("ERROR 6 - Lanelet", curr_lnl.lanelet_id, "'s left equally-directed neighbor", left_neighbor.lanelet_id, "'s right neighbor is opposite-directed")
                else:
                    pass  # print("PASS 2 - Lanelet", curr_lnl.lanelet_id, "'s left equally-directed neighbor", left_neighbor.lanelet_id, "'s right neighbor is equally-directed")

                # check the left neighbor's left neighbor is not current lanelet as well
                if left_neighbor.adj_left == curr_lnl.lanelet_id:
                    print("ERROR 7 - Lanelet", curr_lnl.lanelet_id, "'s left equally-directed neighbor", left_neighbor.lanelet_id, "'s left neighbor is Lanelet", curr_lnl.lanelet_id)
                else:
                    pass # print("PASS 3 - Lanelet", curr_lnl.lanelet_id, "'s left equally-directed neighbor", left_neighbor.lanelet_id, "'s left neighbor is", left_neighbor.adj_left, "and not Lanelet", curr_lnl.lanelet_id)

            else:
                if not left_neighbor.adj_left == curr_lnl.lanelet_id:
                    print("ERROR 8 - Lanelet", curr_lnl.lanelet_id, "'s left opposite-directed neighbor", left_neighbor.lanelet_id, "'s left neighbor is", left_neighbor.adj_left)
                    print_and_show_adjacencies_for_id_list(lanelet_network, [left_neighbor.lanelet_id, left_neighbor.adj_left, curr_lnl.lanelet_id])
                else:
                    pass  # print("PASS 4 - Lanelet", curr_lnl.lanelet_id, "'s left opposite-directed neighbor", left_neighbor.lanelet_id, "'s left neighbor is", left_neighbor.adj_left)

                # check the left neighbor's right neighbor's direction is opposite-directed
                if left_neighbor.adj_left_same_direction:
                    print("ERROR 9 - Lanelet", curr_lnl.lanelet_id, "'s left opposite-directed neighbor", left_neighbor.lanelet_id, "'s left neighbor is equally-directed")
                else:
                    pass  # print("PASS 5 - Lanelet", curr_lnl.lanelet_id, "'s left opposite-directed neighbor", left_neighbor.lanelet_id, "'s right neighbor is opposite-directed")

                # check the left neighbor's right neighbor is not current lanelet as well
                if left_neighbor.adj_right == curr_lnl.lanelet_id:
                    print("ERROR 10 - Lanelet", curr_lnl.lanelet_id, "'s left opposite-directed neighbor", left_neighbor.lanelet_id, "'s right neighbor is Lanelet", curr_lnl.lanelet_id)
                else:
                    pass # print("PASS 6 - Lanelet", curr_lnl.lanelet_id, "'s left opposite-directed neighbor", left_neighbor.lanelet_id, "'s right neighbor is", left_neighbor.adj_right, "and not Lanelet", curr_lnl.lanelet_id)


        # check if current lanelet's right neighbor, if existent, adjacency matches
        if curr_lnl.adj_right and curr_lnl.adj_right in all_lanelet_ids:
            right_neighbor = lanelet_network.find_lanelet_by_id(curr_lnl.adj_right)
            if curr_lnl.adj_right_same_direction:
                if not right_neighbor.adj_left == curr_lnl.lanelet_id:
                    print("ERROR 11 - Lanelet", curr_lnl.lanelet_id, "'s right equally-directed neighbor", right_neighbor.lanelet_id, "'s left neighbor is", right_neighbor.adj_left)
                    print_and_show_adjacencies_for_id_list(lanelet_network, [curr_lnl.lanelet_id, right_neighbor.adj_left, right_neighbor.lanelet_id])
                else:
                    pass  # print("PASS 7 - Lanelet", curr_lnl.lanelet_id, "'s right equally-directed neighbor", right_neighbor.lanelet_id, "'s left neighbor is", right_neighbor.adj_left)

                # check the right neighbor's left neighbor's direction is equally-directed
                if not right_neighbor.adj_left_same_direction:
                    print("ERROR 12 - Lanelet", curr_lnl.lanelet_id, "'s right equally-directed neighbor", right_neighbor.lanelet_id, "'s left neighbor is opposite-directed")
                else:
                    pass  # print("PASS 8 - Lanelet", curr_lnl.lanelet_id, "'s right equally-directed neighbor", right_neighbor.lanelet_id, "'s left neighbor is equally-directed")

                # check the right neighbor's right neighbor is not current lanelet as well
                if right_neighbor.adj_right == curr_lnl.lanelet_id:
                    print("ERROR 13 - Lanelet", curr_lnl.lanelet_id, "'s right equally-directed neighbor", right_neighbor.lanelet_id, "'s right neighbor is Lanelet", curr_lnl.lanelet_id)
                else:
                    pass # print("PASS 9 - Lanelet", curr_lnl.lanelet_id, "'s right equally-directed neighbor", right_neighbor.lanelet_id, "'s right neighbor is", right_neighbor.adj_right, "and not Lanelet", curr_lnl.lanelet_id)

            else:
                if not right_neighbor.adj_right == curr_lnl.lanelet_id:
                    print("ERROR 14 - Lanelet", curr_lnl.lanelet_id, "'s right opposite-directed neighbor", right_neighbor.lanelet_id, "'s right neighbor is", right_neighbor.adj_right)
                    print_and_show_adjacencies_for_id_list(lanelet_network, [curr_lnl.lanelet_id, right_neighbor.lanelet_id, right_neighbor.adj_right])
                else:
                    print("to validate: PASS 10 - Lanelet", curr_lnl.lanelet_id, "'s right opposite-directed neighbor", right_neighbor.lanelet_id, "'s left neighbor is", right_neighbor.adj_left)

                # check the right neighbor's right neighbor's direction is opposite-directed
                if right_neighbor.adj_right_same_direction:
                    print("ERROR 15 - Lanelet", curr_lnl.lanelet_id, "'s right opposite-directed neighbor", right_neighbor.lanelet_id, "'s right neighbor is equally-directed")
                else:
                    print("to validate: PASS 11 - Lanelet", curr_lnl.lanelet_id, "'s right opposite-directed neighbor", right_neighbor.lanelet_id, "'s right neighbor is opposite-directed")

                # check the right neighbor's left neighbor is not current lanelet as well
                if right_neighbor.adj_left == curr_lnl.lanelet_id:
                    print("ERROR 16 - Lanelet", curr_lnl.lanelet_id, "'s right opposite-directed neighbor", right_neighbor.lanelet_id, "'s left neighbor is Lanelet", curr_lnl.lanelet_id)
                else:
                    print("to validate: PASS 12 - Lanelet", curr_lnl.lanelet_id, "'s right opposite-directed neighbor", right_neighbor.lanelet_id, "'s left neighbor is", right_neighbor.adj_left, "and not Lanelet", curr_lnl.lanelet_id)

        # check for other lanelets that are claiming a relationship to the current lanelet that is not expected
        # do not check lanelet against itself, left neighbor and right neighbor
        for other_lanelet in lanelets:
            if not other_lanelet.lanelet_id == curr_lnl.lanelet_id and\
                    not other_lanelet.lanelet_id == curr_lnl.adj_left and\
                    not other_lanelet.lanelet_id == curr_lnl.adj_right:
                if other_lanelet.adj_left == curr_lnl.lanelet_id:
                    print('ERROR 17 - Lanelet', curr_lnl.lanelet_id, 'gets unexpected relationship from', other_lanelet.lanelet_id)
                    print_and_show_adjacencies_for_id_list(lanelet_network, [curr_lnl.lanelet_id, other_lanelet.lanelet_id])
                if other_lanelet.adj_right == curr_lnl.lanelet_id:
                    print('ERROR 18 - Lanelet', curr_lnl.lanelet_id, 'gets unexpected relationship from', other_lanelet.lanelet_id)
                    print_and_show_adjacencies_for_id_list(lanelet_network, [curr_lnl.lanelet_id, other_lanelet.lanelet_id])
    print('----- Finished Adjacency Validation -----\n\n')
    return


def check_successor_predecessor_relationships(lanelet_network):
    print('----- Running Successor-Predecessor Validation -----')

    lanelets = lanelet_network.lanelets
    all_lanelet_ids = [ln.lanelet_id for ln in lanelets]

    for curr_lnl in lanelets:
        # check if preceeding/succeeding lanelets exist at all and print output if not
        for pred_id in curr_lnl.predecessor:
            if pred_id not in all_lanelet_ids:
                print('ERROR 1 - Lanelet', curr_lnl.lanelet_id, 'precedessor', pred_id, 'does not exist\n')
        for pred_id in curr_lnl.successor:
            if pred_id not in all_lanelet_ids:
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
            print('ERROR 3 - Lanelet', curr_lnl.lanelet_id, 'expected predecessors', curr_lnl.predecessor, ', but found', pred_id_matches)
            print_and_show_succession_for_id_list(lanelet_network, pred_id_matches + curr_lnl.predecessor + [curr_lnl.lanelet_id])

        if not passed_successor_validation:
            print('ERROR 4 - Lanelet', curr_lnl.lanelet_id, 'expected successors', curr_lnl.successor, ', but found', succ_id_matches)
            print_and_show_succession_for_id_list(lanelet_network, [curr_lnl.lanelet_id] + curr_lnl.successor + succ_id_matches)
    print('----- Finished Successor-Predecessor Validation-----\n\n')
    return


def check_lanelets_for_at_least_two_vertices(lanelet_network):
    print('----- Running Check For At Least Two Left And Right Vertices -----')
    n = 0
    for curr_lnl in lanelet_network.lanelets:
        left_vertices = curr_lnl.left_vertices
        right_vertices = curr_lnl.right_vertices
        if len(left_vertices) <= 2 or len(right_vertices) <= 2:
            n +=1
    if n > 0:
        print('ERROR - There are', n, 'Lanelets with less then 3 vertices')
        all_ids = [l.lanelet_id for l in lanelet_network.lanelets]
        draw_lanelet_network_section_for_id_list(lanelet_network, all_ids)
    print('----- Finished Check For At Least Two Left And Right Vertices -----\n\n')

    return


def check_lanelets_for_id_uniqueness(lanelet_network):
    print('----- Running Lanelet ID Uniqueness Validation-----')
    all_lanelet_ids = sorted([l.lanelet_id for l in lanelet_network.lanelets])
    all_lanelet_ids_set = sorted(list(set(all_lanelet_ids)))
    if not all_lanelet_ids == all_lanelet_ids_set:
        print('ERROR - Lanelet IDs are not unique')
        all_ids = [l.lanelet_id for l in lanelet_network.lanelets]
        draw_lanelet_network_section_for_id_list(lanelet_network, all_ids)
    else:
        pass
    print('----- Finished Lanelet ID Uniqueness Validation -----\n\n')
    return


def check_lanelet_network(lanelet_network):
    """
    Check a CommonRoad scenario's lanelet for logic errors

    :param lanelet_network: scenario lanelet network to check
    :return: None
    """

    print('===========================================')
    print('===== RUNNING LANELET-NETWORK CHECKER =====')
    print('===========================================')
    print('\n')

    check_lanelets_for_id_uniqueness(lanelet_network)
    check_lanelets_for_at_least_two_vertices(lanelet_network)
    check_successor_predecessor_relationships(lanelet_network)
    check_adjacencies_relationships(lanelet_network)
