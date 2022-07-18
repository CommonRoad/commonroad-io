from typing import Any

import matplotlib.pyplot as plt
import click
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from commonroad.scenario.scenario import Scenario
import numpy as np
import math
import select
import time
import os
import sys
from fonction import *


class drone3d():
    """
    This drone is a map exploration and visualization tool
    It has the purpose to manage the visualization, to detect obstacles but also to find the nearest power line

    """

    def __init__(self, ax, list_obstacle, list_poweline):
        """
                Constructor of a drone object

        :param ax: the figure where the drone should be constructed
        :param list_poweline: list of electric wires where the drone can go
        :param list_obstacle: list of obstacles that the drone can meet
        """


        self._ax = ax
        self._orientation = 0
        self._speed = 2
        self._position = [0, 0, 5]
        self.length = 1
        self.list_obstacle = list_obstacle
        self.width = 1
        self.r = 0.33
        self.elev = 20
        self.list_patchcollection = []
        self._follow = 1
        self.list_poweline = list_poweline
        self.state_know = []
        self.flag_detect = 0


    def __str__(self):
        traffic_str = "\n"
        traffic_str += "Drone:\n"
        traffic_str += "- position: {}\n".format(self._position)
        traffic_str += "- orientation: {}\n".format(self._orientation)

        return traffic_str



    @property
    def ax(self) :
        return self._ax

    @ax.setter
    def ax(self, ax):
        self._ax = ax

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, speed):
        self._speed = speed

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position):
        self._position = position

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, orientation):
        self._orientation = orientation

    @property
    def follow(self):
        return self._follow

    @follow.setter
    def follow(self, follow):
        self._follow = follow


    def new_drone(self):
        """
        Delete the image of the old drone to build a new one and adapt the view
        """
        if self._follow % 2:
            self._ax.set_xlim([self._position[0] - 5, self._position[0] + 5])
            self._ax.set_ylim([self._position[1] - 5, self._position[1] + 5])
            self._ax.set_zlim([self._position[2] - 5, self._position[2] + 5])
            self._ax.view_init(elev=self.elev, azim=-180 + self._orientation * 180 / 3.1415)

        self.construction()
        self.detect()
        plt.pause(0.01)
        for patch in self.list_patchcollection:
            patch.remove()
        for patch in range(len(self.list_patchcollection)):
            self.list_patchcollection.remove(self.list_patchcollection[0])

    def rotate_plus(self):
        self._orientation += 0.2

    def rotate_minus(self):
        self._orientation -= 0.2

    def go(self):
        """moves the drone forward respecting the orientation"""
        self._position[0] += self._speed * np.cos(self._orientation)
        self._position[1] += self._speed * np.sin(self._orientation)

    def returne(self):
        """move the drone backwards respecting the orientation"""
        self._position[0] += -self._speed * np.cos(self._orientation)
        self._position[1] += -self._speed * np.sin(self._orientation)

    def right(self):
        """move the drone to the right respecting the orientation"""
        self._position[0] += +self._speed * np.sin(self._orientation)
        self._position[1] += -self._speed * np.cos(self._orientation)

    def left(self):
        """move the drone to the left respecting the orientation"""
        self._position[0] += -self._speed * np.sin(self._orientation)
        self._position[1] += +self._speed * np.cos(self._orientation)

    def up(self):
        """increases the altitude of the drone"""
        self._position[2] += self._speed

    def down(self):
        """decrease the altitude of the drone"""
        self._position[2] -= self._speed

    def go_powerline(self):
        self.state_know = []  # init
        self.path = self.Astar()

    def construction(self):
        """
        constructs the image of the drone which is made up of a central square
        and four circles symbolizing the pallets
        """

        accurate = 10
        pi = 3.1415

        list = [(- self.length * 0.5, - self.width * 0.5, 0), (- self.length * 0.5, + self.width * 0.5, 0),
                (+ self.length * 0.5, + self.width * 0.5, 0), (+ self.length * 0.5, - self.width * 0.5, 0)]

        list = add_coor(self._position[0], self._position[1], self._position[2], rotation_z(self._orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)
        list = []

        ###############################################################################################################

        for j in range(accurate):
            theta = -pi + 2 * pi * j / accurate
            list.append((self.r * np.cos(theta) + +self.length / 2, self.r * np.sin(theta) + +self.width / 2, 0))

        list = add_coor(self._position[0], self._position[1], self._position[2], rotation_z(self._orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:red", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)

        ###############################################################################################################
        list = []
        for j in range(accurate):
            theta = -pi + 2 * pi * j / accurate
            list.append((self.r * np.cos(theta) - self.length / 2, self.r * np.sin(theta) + self.width / 2, 0))
        list = add_coor(self._position[0], self._position[1], self._position[2], rotation_z(self._orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)

        ###############################################################################################################

        list = []
        for j in range(accurate):
            theta = -pi + 2 * pi * j / accurate
            list.append((self.r * np.cos(theta) + +self.length / 2, self.r * np.sin(theta) + -self.width / 2, 0))
        list = add_coor(self._position[0], self._position[1], self._position[2], rotation_z(self._orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:red", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)

        ###############################################################################################################

        list = []
        for j in range(accurate):
            theta = -pi + 2 * pi * j / accurate
            list.append((self.r * np.cos(theta) - self.length / 2, self.r * np.sin(theta) - self.width / 2, 0))
        list = add_coor(self._position[0], self._position[1], self._position[2], rotation_z(self._orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self._ax.add_collection3d(patchcollection)

        for k in range(3):
            r = 1 + k

            accurate = 15
            pi = math.pi

            list = []

            patches = []

            list = []
            for i in range(accurate):
                for j in range(accurate):
                    theta = -pi / 2 + pi * i / accurate
                    phi = -pi + 2 * pi * j / accurate

                    list.append((r * np.sin(phi) * np.cos(theta) + self._position[0],
                                 r * np.sin(phi) * np.sin(theta) + self._position[1],
                                 r * np.cos(phi) + self._position[2]))

                    patchcollection = Poly3DCollection([list], linewidth=0, rasterized=True, alpha=0.5 - k * 0.1)
            self.list_patchcollection.append(patchcollection)
            self._ax.add_collection3d(patchcollection)

            patches.append(list)

        plt.pause(0.01)

    def detect(self):
        """
        detect a colition and display a detection message and a visual element
        """
        r = 0.6
        accurate = 10
        pi = 3.1415
        list = []
        patches = []
        for obstacle in self.list_obstacle:
            if self.flag_detect != 1:

                if abs(obstacle[0][2] - self._position[2]) < 0.5:
                    if abs(obstacle[0][1] - self._position[1]) < 1.3:
                        if abs(obstacle[0][0] - self._position[0]) < 1.3:
                            print("BOOM")
                            for i in range(accurate):
                                for j in range(accurate):
                                    phi = -pi / 2 + pi * i / accurate
                                    theta = -pi + 2 * pi * j / accurate
                                    list.append((r * np.sin(theta) * np.cos(phi) + obstacle[0],
                                                 r * np.sin(theta) * np.sin(phi) + obstacle[1],
                                                 r * np.cos(theta) + obstacle[2]))

                            patches.append(list)

                            patchcollection = Poly3DCollection(patches, edgecolor="r", facecolor="tab:red",
                                                               rasterized=True)
                            self._ax.add_collection3d(patchcollection)
                            self.flag_detect = 1

    def on_powerline(self, position):
        """
        calcul pour savoir si le drone touche les ligne electrique

        :param position: la position a tester
        :return: le resultat du test
        """
        for obstacle in self.list_poweline:
            if abs(obstacle[2] - position[2]) < self._speed:
                if abs(obstacle[1] - position[1]) < self._speed:
                    if abs(obstacle[0] - position[0]) < self._speed:
                        return True
        return False

    def position_ok(self, x, y, z):
        """
        calculation to know if the drone touches an obstacle

        :param position: the position to test
        :return: the result of the test
        """
        for obstacle in self.list_obstacle:
            if abs(obstacle[2] - z) < 0.5:
                if abs(obstacle[1] - y) < 1.3:
                    if abs(obstacle[0] - x) < 1.3:
                        return False
        return True

    def construct_path(self, final, camefrom):

        total_path = []
        total_path.append(final)

        current = final

        while current[0] != self._position[0] or current[1] != self._position[1] or current[2] != self._position[2]:
            current = self.state_know[self.state_know.index(camefrom[self.state_know.index(current)])]
            total_path.append(current)

        return total_path

    def Astar(self):
        """
        moves the drone to the nearest power line
        avoiding obstacles thanks to an A star algorithm
        """

        if self.list_poweline == []:
            print("path not found")
            return self._position
        open = []
        directions = [-self._speed, 0, self._speed]
        camefrom = []
        self.state_know = []
        open.append(self._position)

        if self._orientation < 0:
            while self._orientation != 0:
                self.rotate_plus()
        if self._orientation > 0:
            while self._orientation != 0:
                self.rotate_minus()

        while True:
            curent = self.choose_curent(open)
            print("current=")
            print(curent)

            if self.on_powerline(curent):
                print("path find")
                return self.construct_path(curent, camefrom)
            open.remove(curent)

            for direction_x in directions:
                if direction_x:
                    if self._position_ok(curent[0] + direction_x, curent[1], curent[2]):
                        if not [curent[0] + direction_x, curent[1], curent[2]] in open:
                            if not [curent[0] + direction_x, curent[1], curent[2]] in self.state_know:
                                camefrom.append(curent)
                                self.state_know.append([curent[0] + direction_x, curent[1], curent[2]])
                                open.append([curent[0] + direction_x, curent[1], curent[2]])
            for direction_y in directions:
                if direction_y:
                    if self._position_ok(curent[0], curent[1] + direction_y, curent[2]):
                        if not [curent[0], curent[1] + direction_y, curent[2]] in open:
                            if not [curent[0], curent[1] + direction_y, curent[2]] in self.state_know:
                                self.state_know.append([curent[0], curent[1] + direction_y, curent[2]])
                                camefrom.append(curent)
                                open.append([curent[0], curent[1] + direction_y, curent[2]])
            for direction_z in directions:
                if direction_z:
                    if self._position_ok(curent[0], curent[1], curent[2] + direction_z):
                        if not [curent[0], curent[1], curent[2] + direction_z] in open:
                            if not [curent[0], curent[1], curent[2] + direction_z] in self.state_know:
                                camefrom.append(curent)
                                self.state_know.append([curent[0], curent[1], curent[2] + direction_z])

                                open.append([curent[0], curent[1], curent[2] + direction_z])

    def heuristic(self, position):
        """
        the chosen heuristic is the manathan distance

        :retun: the heuristic for a chosen position
        """
        min = 100000000000
        for obstacle in self.list_poweline:
            if abs(obstacle[2] - position[2]) < 0.5:
                if abs(obstacle[1] - position[1]) < 1.3:
                    if abs(obstacle[0] - position[0]) < 1.3:
                        return 0
            if min > abs(obstacle[0] - position[0]) + abs(obstacle[1] - position[1]) + abs(obstacle[2] - position[2]):
                min = abs(obstacle[0] - position[0]) + abs(obstacle[1] - position[1]) + abs(obstacle[2] - position[2])

        return min

    def choose_curent(self, open):
        """
        choose the positionn with the smallest euristic

        :param: open list of positions to test
        :return: the most optimal position
        """

        curent = open[0]

        for position in open:

            if self.heuristic(curent) > self.heuristic(position):
                curent = position

        return curent


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
