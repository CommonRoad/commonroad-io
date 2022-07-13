import matplotlib.pyplot as plt
import click
from commonroad.scenario.scenario import Scenario
import matplotlib.pyplot as plt
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from commonroad.scenario.scenario import Scenario
import numpy as np
from drone import drone3d
from powerline import poweline
import math
from commonroad.scenario.obstacle import ObstacleType
from traffic_lights import trafic_light
from bike import bike



class pedestrian():
    def __init__(self ,scenario :Scenario, i :int ,ax):
        """
        class to build the pedestrian

        :param: list_return stoke forms
        :param: accurate precision of the shape of the pedestrian
        :param: r radius of the person
        """

        r = 0.5
        # r=1#american
        pi = math.pi
        self.accurate = 10
        self.scenari o =scenario
        self.list_retur n =[]

        for t in range(self.scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = self.scenario.dynamic_obstacles[i].occupancy_at_time(t).shape
            top = 2

            biglist = []
            orientation = 0
            orientationy = 0
            list = []
            for j in range(self.accurate):
                theta = -pi + 2 * pi * j / self.accurate
                thetap1 = -pi + 2 * pi * (j + 1) / self.accurate
                a = (r * np.sin(theta), top / 2 + r * np.cos(theta), top)
                b = (r * np.sin(thetap1), top / 2 + r * np.cos(thetap1), 0.4)
                c = (r * np.sin(thetap1), top / 2 + r * np.cos(thetap1), top)
                d = (r * np.sin(theta), top / 2 + r * np.cos(theta), 0.4)

                list = [b, c, a, d]

                list = rotation_x(orientationy, rotation_z(orientation, list))
                list = add_center(shape.center[0], shape.center[1], list)

                biglist.append(list)

            colors = ["tab:blue" for patch in list]
            list_patchcollection = []
            for list in biglist:
                patchcollection = Poly3DCollection(list, linewidth=0, edgecolor="k", facecolor=colors, rasterized=True,
                                                   zorder=10)
                ax.add_collection3d(patchcollection)
                list_patchcollection.append(patchcollection)
            self.list_return.append(list_patchcollection)





def fctx(i):
    return 30


def fcty(i):
    return -87 + 20 * i


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


