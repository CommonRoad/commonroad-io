import matplotlib.pyplot as plt
import click
from commonroad.scenario.scenario import Scenario
import matplotlib.pyplot as plt
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from commonroad.scenario.scenario import Scenario
import numpy as np
from drone import drone3d

import math




class Tree():
    """
    classe arcre pour ajouter de la diversiter et des obstacles a l'environement
    """

    def __init__(self, ax, x: int, y: int, z: int, r: float, wr: float, list_obstacle: list):
        pi = math.pi
        """

          Constructor of a tree object

          :param x: position x de l'arbre 
          :param y: position y de l'arbre 
          :param z: hauteur de l'arcle
          :param r: rayon des feuilles
          :param wr: rayon du tronc
          :param accurate: degree de precision
          :param list_obstacle: liste des point que le drone ne dois pas rencontre

          """

        self.x = x
        self.y = y
        self.z = z
        self.r = r
        self.wr = wr
        self.ax=ax
        self.list_obstacle = list_obstacle
        accurate = 50

        self.list = []
        self.list2 = []
        self.patches = []
        self.patches2 = []

        for i in range(accurate):
            for j in range(accurate):
                phi = -pi / 2 + pi * i / accurate
                theta = -pi + 2 * pi * j / accurate

                self.list.append((r * np.sin(theta) * np.cos(phi) + x, r * np.sin(theta) * np.sin(phi) + y,
                                  r * np.cos(theta) + z))
                self.list_obstacle.append([(r * np.sin(theta) * np.cos(phi) + x, r * np.sin(theta) * np.sin(phi) + y,
                                           r * np.cos(theta) + z)])
                self.list2.append((wr * np.cos(theta) + x, wr * np.sin(theta) + y, i * z / accurate))
                self.list_obstacle.append([(wr * np.cos(theta) + x, wr * np.sin(theta) + y, i * z / accurate)])

        self.patches.append(self.list)
        self.patches2.append(self.list2)

        colors = "tab:green"
        patchcollection = Poly3DCollection(self.patches, edgecolor="g", facecolor=colors, rasterized=True)
        self.ax.add_collection3d(patchcollection)

        patchcollection = Poly3DCollection(self.patches2, edgecolor="g", facecolor=colors, rasterized=True)
        self.ax.add_collection3d(patchcollection)



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
