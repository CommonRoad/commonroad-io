from typing import Any
import matplotlib.pyplot as plt
import click
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import select
import time
import os
import sys
from commonroad.scenario.traffic_sign import TrafficSign, TrafficLight



"""
        :param traffic_sign_id: ID of traffic sign
        :param traffic_sign_elements: list of traffic sign elements
        :param position: position of traffic sign
        :param virtual: boolean indicating if this traffic sign is also
        placed there in the real environment or it
        is added for other reasons (e.g., completeness of scenario)
"""

class traffic_sign
    def __init__(self,ax,plt,scenario,traffic_sign):#(self.ax, self.fig, self.scenario, self.list_traffic_lights,scenario.lanelet_network.lanelets[i]))

        self.fig = fig
        self.ax = ax
        self.scenario = scenario
        self.traffic_sign=traffic_sign
        print(self.traffic_sign)





    def find_color(self):
        """
        find the traffic light
        """



    def new_sign(self):

        for patch in self.list_patchcollection:
            patch.remove()
        for patch in range(len(self.list_patchcollection)):
            self.list_patchcollection.remove(self.list_patchcollection[0])
        self.construction()

    def construction(self):

        for l in range(len(self.scenario.lanelet_network.lanelets)):
            lanelet = self.scenario.lanelet_network.
            colors = ["tab:grey"]
            for i in range(int(len(lanelet) / 2)):
                a = (lanelet[i][0], lanelet[i][1], 0)
                b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], 0)
                c = (lanelet[i + 1][0], lanelet[i + 1][1], 0)
                d = (lanelet[-i][0], lanelet[-i][1], 0)
                lan = [[b, c, a, d]]
                if self.bridge_flag:
                    self.bridge.append(lan[0])

                lancolect = Poly3DCollection(lan, linewidth=0, edgecolor="k", facecolor=colors, rasterized=False,
                                             alpha=1, zorder=1)

                self.ax.add_collection3d(lancolect)

        color = self.find_color()
        if self.find_color() == self.traffic_light.cycle[2].state:
            color = "tab:red"
        list = [  # red
            (+ length * 0.5, - width * 0.5, top), (+ length * 0.5, + width * 0.5, top),
            (+ length * 0.5, + width * 0.5, top * 2 / 3), (+ length * 0.5, - width * 0.5, top * 2 / 3)]


        patchcollection = Poly3DCollection([list], linewidth=0, facecolor=color, edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)


