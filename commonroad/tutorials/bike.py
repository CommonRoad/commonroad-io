import matplotlib.pyplot as plt
import click
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from drone import drone3d
from powerline import poweline
import math
from commonroad.scenario.obstacle import ObstacleType






class bike():

    def __init__(self,scenario:Scenario,i:int,ax):
        """
        class to build a bike

        :param: list_return stoke forms
        :param: accurate precision of the shape of the bike
        :param: r radius of the wheel
        """
        self.scenario=scenario
        self.list_return = []
        self.r = 0.5
        pi = math.pi
        self.accurate = 10
        for t in range(self.scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = self.scenario.dynamic_obstacles[i].occupancy_at_time(t).shape

            top = 1.5


            length = shape.length

            biglist = []
            o = shape.orientation

            list = [
                (- length * 0.5, 0, top), (+ length * 0.5, 0, top), (0, 0, top / 2)]

            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)
            first = list[0][0]
            last = list[0][1]

            list = []
            for j in range(self.accurate):
                theta = -pi + 2 * pi * j / self.accurate
                list.append((self.r * np.sin(theta), top / 2 + self.r * np.cos(theta), 0))

            list = rotation_z(o, list)
            list = add_center(last[0], last[1], list)

            biglist.append(list)

            list = []

            for j in range(self.accurate):
                theta = -pi + 2 * pi * j / self.accurate
                list.append((self.r * np.sin(theta), 0, top / 2 + self.r * np.cos(theta)))

            list = rotation_z(o, list)
            list = add_center(first[0], first[1], list)
            biglist.append(list)

            list_patchcollection = []

            for list in biglist:
                colors = ["tab:blue" for patch in list]
                patchcollection = Poly3DCollection(list, linewidth=0, edgecolor="k", facecolor=colors, rasterized=True,
                                                   zorder=10)
                ax.add_collection3d(patchcollection)
                list_patchcollection.append(patchcollection)
            self.list_return.append(list_patchcollection)


def rotation_z(o, liste: list):
    list_tempo = []
    for i in range(len(liste)):
        list_tempo.append((liste[i][0] * np.cos(o) - liste[i][1] * np.sin(o),
                           liste[i][0] * np.sin(o) + liste[i][1] * np.cos(o), liste[i][2]))
    return list_tempo


def rotation_x(o, liste: list):
    list_tempo = []
    for i in range(len(liste)):
        list_tempo.append((liste[i][0], liste[i][1] * np.cos(o) + liste[i][2] * np.sin(o),
                           liste[i][1] * np.sin(o) + liste[i][2] * np.cos(o)))
    return list_tempo


def rotation_y(o, liste: list):
    list_tempo = []
    for i in range(len(liste)):
        list_tempo.append((liste[i][0] * np.cos(o) - liste[i][2] * np.sin(o), liste[i][1],
                           -liste[i][0] * np.sin(o) + liste[i][2] * np.cos(o)))
    return list_tempo


def add_center(x: float, y: float, list: list):
    list_tempo = []
    list_ret = []
    for i in range(len(list)):
        list_tempo.append((list[i][0] + x, list[i][1] + y, list[i][2]))
        list_ret.append(list_tempo)
    return list_ret


def add_centerb(x: float, y: float, z: list, list: list):
    list_tempo = []
    list_ret = []
    for i in range(4):
        list_tempo.append((list[i][0] + y, list[i][1] + x, list[i][2] + z[i]))
        list_ret.append(list_tempo)
    return list_ret


def sign(i: int):
    if i < 0:
        return -1
    return 1
