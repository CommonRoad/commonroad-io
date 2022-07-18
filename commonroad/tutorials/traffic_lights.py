from typing import Any

import matplotlib.pyplot as plt
import click
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from commonroad.scenario.scenario import Scenario
import numpy as np
import select
import time
import os
import sys
from commonroad.scenario.traffic_sign import TrafficSign, TrafficLight, TrafficLightState, TrafficLightState, \
    TrafficLightDirection

from fonction import *


class trafic_light():

    def __init__(self, ax, scenario, traffic_light: TrafficLight, long: float, hight: float, size: float):
        """

        Constructor of a traffic light and traffics signs object

        :param ax: the figure where the traffic must be constructed
        :param scenario: scenario in which the information is stored
        :param long: the figure or the drone must be built
        :param hight: the figure or the drone must be built
        :param size: the figure or the drone must be built

        """

        self._ax = ax
        self._scenario = scenario
        self.traffic_light = traffic_light
        self._time = 0
        self.list_patchcollection = []
        self.position = self.traffic_light.position
        self.size = size
        self.hight = hight
        self.long = long
        self.orientation = 0

        if self.traffic_light.direction == TrafficLightDirection.LEFT:
            self.position[0] += 3
        if self.traffic_light.direction == TrafficLightDirection.RIGHT:
            self.position[0] -= 3

    def __str__(self):
        traffic_str = "\n"
        traffic_str += "Drone:\n"
        traffic_str += "- position: {}\n".format(self.position)
        traffic_str += "- orientation: {}\n".format(self.orientation)

        return traffic_str

    def __eq__(self, other):
        if self._scenario == other.scenario and self.size == other.size and self.hight == other.hight and self.long ==\
                other.long:
            return True
        return False

    @property
    def ax(self):
        return self._ax

    @ax.setter
    def ax(self, ax):
        self._ax = ax

    @property
    def scenario(self):
        return self._scenario

    @scenario.setter
    def scenario(self, scenario):
        self._scenario = scenario

    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, time):
        self._time = time

    def find_color(self):
        """
         calculate the time in relation to the cycle

        :return: the color used by the figure
        """
        time = self._scenario.dt * self._time * 100 + self.traffic_light.time_offset  # same time unit

        full_cycle = 0
        for i in range(len(self.traffic_light.cycle)):
            full_cycle += self.traffic_light.cycle[i].duration

        cycle = 0

        for i in range(len(self.traffic_light.cycle)):
            if cycle <= time % full_cycle < cycle + self.traffic_light.cycle[i].duration:
                state = self.traffic_light.cycle[i].state
                return color(state)
            cycle += self.traffic_light.cycle[i].duration

    def new_light(self):
        """
        acctualization of the traffics
        """

        for patch in self.list_patchcollection:
            patch.remove()
        for patch in range(len(self.list_patchcollection)):
            self.list_patchcollection.remove(self.list_patchcollection[0])
        self.construction()

    def construction(self):
        """
        build the new traffic lights
        """
        length = self.size
        width = self.size
        bottom = 0
        top = 1.5

        list = [  # underneath
            (- length * 0.5, - width * 0.5, bottom), (- length * 0.5, + width * 0.5, bottom),
            (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:grey", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)

        ###############################################################################################################

        list = [  # top
            (- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
            (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, - width * 0.5, top)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:grey", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)

        ###############################################################################################################
        list = [  # bottom
            (- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
            (- length * 0.5, + width * 0.5, bottom), (- length * 0.5, - width * 0.5, bottom)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:grey", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)

        ###############################################################################################################

        list = [  # right
            (- length * 0.5, + width * 0.5, bottom), (- length * 0.5, + width * 0.5, top),
            (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, + width * 0.5, bottom)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:grey", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)

        ###############################################################################################################
        list = [  # left
            (- length * 0.5, - width * 0.5, bottom), (- length * 0.5, - width * 0.5, top),
            (+ length * 0.5, - width * 0.5, top), (+ length * 0.5, - width * 0.5, bottom)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:grey", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)

        ###############################################################################################################
        color = "tab:grey"
        if self.find_color() == "tab:green":
            color = "tab:green"
        list = [  # green
            (+ length * 0.5, - width * 0.5, top / 3), (+ length * 0.5, + width * 0.5, top / 3),
            (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor=color, edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)

        color = "tab:grey"
        if self.find_color() == "tab:orange":
            color = "tab:orange"
        list = [  # orange
            (+ length * 0.5, - width * 0.5, top * 2 / 3), (+ length * 0.5, + width * 0.5, top * 2 / 3),
            (+ length * 0.5, + width * 0.5, top / 3), (+ length * 0.5, - width * 0.5, top / 3)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor=color, edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)

        color = "tab:grey"
        if self.find_color() == "tab:red":
            color = "tab:red"
        list = [  # red
            (+ length * 0.5, - width * 0.5, top), (+ length * 0.5, + width * 0.5, top),
            (+ length * 0.5, + width * 0.5, top * 2 / 3), (+ length * 0.5, - width * 0.5, top * 2 / 3)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor=color, edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)
        ################################################################################################################

        color = self.find_color()

        for l in range(len(self._scenario.lanelet_network.lanelets)):
            if self.traffic_light.traffic_light_id in self._scenario.lanelet_network.lanelets[l].traffic_lights:
                lanelet = self._scenario.lanelet_network.lanelets[l].polygon.vertices
                for i in range(int(len(lanelet) / 2)):
                    for j in range(1):
                        a = (lanelet[i][0], lanelet[i][1], 0.1 * j)
                        b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], 0.1 * j)
                        c = (lanelet[i + 1][0], lanelet[i + 1][1], 0.1 * j)
                        d = (lanelet[-i][0], lanelet[-i][1], 0.1 * j)
                        list = [b, c, a, d]

                        patchcollection = Poly3DCollection([list], linewidth=0, facecolor=color, edgecolor="k",
                                                           rasterized=True)

                        self._ax.add_collection3d(patchcollection)
