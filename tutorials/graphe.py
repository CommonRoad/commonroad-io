import networkx as nx

from commonroad.scenario.scenario import Scenario

import matplotlib.pyplot as plt
class Graphe:
    def __init__(self,scenario:Scenario):
        G = nx.DiGraph()
        open = []
        color_map = []


        for i in range(len(scenario.lanelet_network.lanelets)):
            if scenario.lanelet_network.lanelets[i].predecessor == []:
                open.append(scenario.lanelet_network.lanelets[i].lanelet_id)

        while open != []:
            current = scenario.lanelet_network.find_lanelet_by_id(open[0])
            open.remove(open[0])
            for i in range(len(current.successor)):
                open.append(current.successor[i])
                G.add_edge(str(current.lanelet_id), str(current.successor[i]))
            for i in range(len(current.adj_left_same_direction)):
                open.append(current.adj_left_same_direction[i])
                G.add_edge(str(current.lanelet_id), str(current.adj_left_same_direction[i]))
            for i in range(len(current.adj_right_same_direction)):
                open.append(current.adj_right_same_direction[i])
                G.add_edge(str(current.lanelet_id), str(current.adj_right_same_direction[i]))



        for node in G:
            if scenario.lanelet_network.find_lanelet_by_id(int(node)).layer!=0:
                color_map.append('blue')
            if scenario.lanelet_network.find_lanelet_by_id(int(node)).layer==0:
                color_map.append('red')

        nx.draw(G, node_color=color_map, with_labels=True)


