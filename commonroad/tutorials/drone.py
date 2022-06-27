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


class drone3d():

    def __init__(self, ax, fig, obstacle, list_poweline):

        self.fig = fig
        self.ax = ax
        self.orientation =0
        self.speed = 2
        self.position = [-110, 470, 250]
        self.length = 1
        self.list_obstacle = obstacle
        self.width = 1
        self.r = 0.33
        self.elev=90
        self.list_patchcollection = []
        self.follow = 1
        self.list_poweline = list_poweline
        self.state_know=[]


        self.flag_detect = 0





    def new_drone(self):
        if self.follow % 2:
            self.ax.set_xlim([self.position[0] - 5, self.position[0] + 5])
            self.ax.set_ylim([self.position[1] - 5, self.position[1] + 5])
            self.ax.set_zlim([self.position[2] - 5, self.position[2] + 5])
            self.ax.view_init(elev=self.elev, azim=180 + self.orientation * 180 / 3.1415)

        self.construction()
        self.detect()
        plt.pause(0.01)
        for patch in self.list_patchcollection:
            patch.remove()
        for patch in range(len(self.list_patchcollection)):
            self.list_patchcollection.remove(self.list_patchcollection[0])

    def rotate_plus(self):
        self.orientation += 0.33

    def rotate_minus(self):
        self.orientation -= 0.33

    def go(self):
        self.position[0] += self.speed * np.cos(self.orientation)
        self.position[1] += self.speed * np.sin(self.orientation)

    def returne(self):
        self.position[0] += -self.speed * np.cos(self.orientation)
        self.position[1] += -self.speed * np.sin(self.orientation)

    def right(self):
        self.position[0] += +self.speed * np.sin(self.orientation)
        self.position[1] += -self.speed * np.cos(self.orientation)

    def left(self):
        self.position[0] += -self.speed * np.sin(self.orientation)
        self.position[1] += +self.speed * np.cos(self.orientation)

    def up(self):
        self.position[2] += self.speed

    def down(self):
        self.position[2] -= self.speed

    def go_powerline(self):
        self.state_know=[]#init
        self.path = self.Astar()


    def construction(self):
        biglist = []
        patches = []
        accurate = 10
        pi = 3.1415

        list = [(- self.length * 0.5, - self.width * 0.5, 0), (- self.length * 0.5, + self.width * 0.5, 0),
                (+ self.length * 0.5, + self.width * 0.5, 0), (+ self.length * 0.5, - self.width * 0.5, 0)]

        list = add_coor(self.position[0], self.position[1], self.position[2], rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)
        list = []

        ###############################################################################################################

        for j in range(accurate):
            theta = -pi + 2 * pi * j / accurate
            list.append((self.r * np.cos(theta) + +self.length / 2, self.r * np.sin(theta) + +self.width / 2, 0))

        list = add_coor(self.position[0], self.position[1], self.position[2], rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:red", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)

        ###############################################################################################################
        list = []
        for j in range(accurate):
            theta = -pi + 2 * pi * j / accurate
            list.append((self.r * np.cos(theta) - self.length / 2, self.r * np.sin(theta) + self.width / 2, 0))
        list = add_coor(self.position[0], self.position[1], self.position[2], rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)

        ###############################################################################################################

        list = []
        for j in range(accurate):
            theta = -pi + 2 * pi * j / accurate
            list.append((self.r * np.cos(theta) + +self.length / 2, self.r * np.sin(theta) + -self.width / 2, 0))
        list = add_coor(self.position[0], self.position[1], self.position[2], rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, facecolor="tab:red", edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)

        ###############################################################################################################

        list = []
        for j in range(accurate):
            theta = -pi + 2 * pi * j / accurate
            list.append((self.r * np.cos(theta) - self.length / 2, self.r * np.sin(theta) - self.width / 2, 0))
        list = add_coor(self.position[0], self.position[1], self.position[2], rotation_z(self.orientation, list))
        patchcollection = Poly3DCollection([list], linewidth=0, edgecolor="k", rasterized=True)
        self.list_patchcollection.append(patchcollection)
        self.ax.add_collection3d(patchcollection)

        plt.pause(0.01)

    def detect(self):
        r = 0.6
        accurate = 10
        pi = 3.1415
        list = []
        patches = []
        for obstacle in self.list_obstacle:
            if self.flag_detect != 1:
                if abs(obstacle[2] - self.position[2]) < 0.5:
                    if abs(obstacle[1] - self.position[1]) < 1.3:
                        if abs(obstacle[0] - self.position[0]) < 1.3:
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
                            self.ax.add_collection3d(patchcollection)
                            self.flag_detect = 1



    def on_powerline(self, position):
        for obstacle in self.list_poweline:
            if abs(obstacle[2] - position[2]) < self.speed:
                if abs(obstacle[1] - position[1]) < self.speed:
                    if abs(obstacle[0] - position[0]) < self.speed:
                        return True
        return False



    def position_ok(self, x, y, z):
        for obstacle in self.list_obstacle:
            if abs(obstacle[2] - z) < 0.5:
                if abs(obstacle[1] - y) < 1.3:
                    if abs(obstacle[0] - x) < 1.3:
                        return False
        return True




    def construct_path(self, final, camefrom):

        total_path = []
        deja=[]
        total_path.append(final)

        current=final


        while current[0] != self.position[0] or current[1] != self.position[1] or current[2] != self.position[2]:


            current=self.state_know[self.state_know.index(camefrom[self.state_know.index(current)])]
            total_path.append(current)

        return total_path

    def Astar(self):

        open = []
        directions = [-self.speed, 0, self.speed]
        curent = self.position
        camefrom = []
        self.state_know = []
        open.append(self.position)


        if self.orientation<0:
            while self.orientation!=0:
                self.rotate_plus()
        if self.orientation>0:
            while self.orientation!=0:
                self.rotate_minus()


        while True:
            curent = self.choose_curent(open)
            print("current=")
            print(curent)

            if self.on_powerline(curent):
                print ("path find")
                return self.construct_path(curent,camefrom)
            open.remove(curent)

            for direction_x in directions:
                if direction_x :
                    if self.position_ok(curent[0] + direction_x, curent[1] ,
                                        curent[2] ):
                        if not [curent[0] + direction_x, curent[1] ,
                                        curent[2]] in open:
                            if not [curent[0] + direction_x, curent[1], curent[2]] in self.state_know:
                                camefrom.append(curent)
                                self.state_know.append([curent[0] + direction_x, curent[1] , curent[2] ])
                                open.append(
                                                [curent[0] + direction_x, curent[1] , curent[2] ])
            for direction_y in directions:
                if direction_y :
                    if self.position_ok(curent[0] , curent[1] + direction_y,
                                        curent[2] ):
                        if not [curent[0] , curent[1] + direction_y,
                                        curent[2] ] in open:
                            if not [curent[0], curent[1] + direction_y, curent[2]] in self.state_know:
                                self.state_know.append([curent[0] , curent[1] + direction_y, curent[2] ])
                                camefrom.append(curent)
                                open.append(
                                                [curent[0] , curent[1] + direction_y, curent[2] ])
            for direction_z in directions:
                if direction_z :
                    if self.position_ok(curent[0] , curent[1] ,
                                        curent[2] + direction_z):
                        if not [curent[0] , curent[1] ,
                                        curent[2] + direction_z] in open:
                            if not [curent[0], curent[1], curent[2] + direction_z] in self.state_know:
                                camefrom.append(curent)
                                self.state_know.append([curent[0], curent[1]  , curent[2]+direction_z])

                                open.append(
                                                [curent[0] , curent[1] , curent[2] + direction_z])

    def heuristic(self, position):
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
