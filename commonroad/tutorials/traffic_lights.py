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
from commonroad.scenario.traffic_sign import TrafficSign, TrafficLight


class trafic_light():

    def __init__(self, ax, fig,scenario, traffic_light):

        self.fig = fig
        self.ax = ax
        self.scenario=scenario
        self.traffic_light = traffic_light
        self.time = 0
        self.list_patchcollection = []
        self.position = self.traffic_light.position
        self.size = 0.5
        self.hight = 3
        self.long = 1.5
        self.orientation = 0
        self.r = 0.25

    """
            help to find param
                    :param traffic_light_id: ID of traffic light
                    :param cycle: list of traffic light cycle elements
                    :param time_offset: offset of traffic light cycle
                    :param position: position of traffic light
                    :param direction: driving directions for which the traffic light is valid
                    :param active: boolean indicating if traffic light is currently active
                    """

    def find_color(self):
        traffic_light = self.traffic_light
        time = self.scenario.dt * self.time * 100 + self.traffic_light.time_offset  # same time unit

        full_cycle = self.traffic_light.cycle[0].duration + self.traffic_light.cycle[1].duration + \
                     self.traffic_light.cycle[2].duration
        if 0 <= time % full_cycle < self.traffic_light.cycle[0].duration:
            return self.traffic_light.cycle[0].state
        if self.traffic_light.cycle[0].duration <= time % full_cycle < self.traffic_light.cycle[0].duration+self.traffic_light.cycle[1].duration:
            return self.traffic_light.cycle[1].state
        return self.traffic_light.cycle[2].state

    def new_light(self):

        self.construction()
        for patch in self.list_patchcollection:
            patch.remove()
        for patch in range(len(self.list_patchcollection)):
            self.list_patchcollection.remove(self.list_patchcollection[0])

    def construction(self):
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
        self.ax.add_collection3d(patchcollection)
        list = []

        ###############################################################################################################

        list = [  # top
            (- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
            (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, - width * 0.5, top)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:grey", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)
        list = []



        ###############################################################################################################
        list = [  # bottom
            (- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
            (- length * 0.5, + width * 0.5, bottom), (- length * 0.5, - width * 0.5, bottom)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:grey", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)
        list = []

        ###############################################################################################################

        list = [  # right
            (- length * 0.5, + width * 0.5, bottom), (- length * 0.5, + width * 0.5, top),
            (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, + width * 0.5, bottom)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:grey", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)
        list = []

        ###############################################################################################################
        list = [  # left
            (- length * 0.5, - width * 0.5, bottom), (- length * 0.5, - width * 0.5, top),
            (+ length * 0.5, - width * 0.5, top), (+ length * 0.5, - width * 0.5, bottom)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:grey", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)
        list = []
        ###############################################################################################################
        """
        list = []#green
        accurate = 10
        pi=3.1415
        r=self.r
        for i in range(accurate):
            for j in range(accurate):
                phi = -pi / 2 + pi * i / accurate
                theta = -pi + 2 * pi * j / accurate

                list.append((r * np.sin(theta) * np.cos(phi) , r * np.sin(theta) * np.sin(phi) ,
                             r *np.cos(theta) ))


        list = add_coor(self.position[0], self.position[1]+0.2, self.hight + self.long / 6,
                        rotation_z(self.orientation, list))
        color="tab:blue"
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:blue", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)

        list = []  # orange
        accurate = 10
        pi = 3.1415
        for i in range(accurate):
            for j in range(accurate):
                phi = -pi / 2 + pi * i / accurate
                theta = -pi + 2 * pi * j / accurate

                list.append((r * np.sin(theta) * np.cos(phi), r * np.sin(theta) * np.sin(phi), r *np.cos(theta)))

        list = add_coor(self.position[0], self.position[1]+0.2, self.hight + self.long / 2,
                        rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:blue", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)

        list = []  # red
        accurate = 10
        pi = 3.1415
        for i in range(accurate):
            for j in range(accurate):
                phi = -pi / 2 + pi * i / accurate
                theta = -pi + 2 * pi * j / accurate

                list.append((r * np.sin(theta) * np.cos(phi), r * np.sin(theta) * np.sin(phi), r * np.cos(theta)))

        list = add_coor(self.position[0], self.position[1]+0.2, self.hight + self.long*5 / 6,
                        rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:blue", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)
        """

        ###############################################################################################################
        color = "tab:grey"
        if self.find_color()==self.traffic_light.cycle[0].state:
            color = "tab:green"
        list = [  # green
            (+ length * 0.5, - width * 0.5, top / 3), (+ length * 0.5, + width * 0.5, top / 3),
            (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor=color, edgecolor="k",
                                           rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)
        list = []
        color = "tab:grey"
        if self.find_color()==self.traffic_light.cycle[1].state:
            color = "tab:orange"
        list = [  # orange
            (+ length * 0.5, - width * 0.5, top * 2 / 3), (+ length * 0.5, + width * 0.5, top * 2 / 3),
            (+ length * 0.5, + width * 0.5, top / 3), (+ length * 0.5, - width * 0.5, top / 3)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor=color, edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)
        list = []
        color="tab:grey"
        if self.find_color()==self.traffic_light.cycle[2].state:
            color = "tab:red"
        list = [  # red
            (+ length * 0.5, - width * 0.5, top), (+ length * 0.5, + width * 0.5, top),
            (+ length * 0.5, + width * 0.5, top*2/3), (+ length * 0.5, - width * 0.5, top*2/3)]

        list = add_coor(self.position[0], self.position[1], self.hight, rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor=color, edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)
        list = []
        plt.pause(0.01)



def rotation_z(o, liste: list):
    # rotation vector
    # x'= xcos(o)-ysin(o)
    # y'= xsin(o)+ycos(o)
    list_tempo = []
    for i in range(len(liste)):
        list_tempo.append((liste[i][0] * np.cos(o) - liste[i][1] * np.sin(o),
                           liste[i][0] * np.sin(o) + liste[i][1] * np.cos(o), liste[i][2]))
    return list_tempo


def add_coor(x, y, z, liste: list):
    # rotation vector
    # x'= xcos(o)-ysin(o)
    # y'= xsin(o)+ycos(o)
    list_tempo = []
    for i in range(len(liste)):
        list_tempo.append((liste[i][0] + x, liste[i][1] + y, liste[i][2] + z))
    return list_tempo
