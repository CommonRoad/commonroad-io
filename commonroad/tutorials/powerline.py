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


class poweline():


    def __init__(self,ax,x,y,xp1,yp1,obstable):

        roof = 5
        top = 4
        bottom = 0
        base = 1.5
        midbase = 1
        o=abs(x-xp1)/abs(y-yp1)

        biglist = []
        ################################################################################################################
        list = [  # underneath
            (- base * 0.5, - base * 0.5, bottom), (- base * 0.5, + base * 0.5, bottom),
            (+ base * 0.5, + base * 0.5, bottom), (+ base * 0.5, - base * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])

        biglist.append(list)
        ################################################################################################################
        list = [  # underneath
            (- base * 0.5, - base * 0.5, bottom), (+ base * 0.5, + base * 0.5, bottom),
            (- base * 0.5, + base * 0.5, bottom), (+ base * 0.5, - base * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # top
            (- midbase * 0.5, - midbase * 0.5, top), (- midbase * 0.5, + midbase * 0.5, top),
            (+ midbase * 0.5, + midbase * 0.5, top), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # front
            (+ midbase * 0.5, - midbase * 0.5, top), (+ midbase * 0.5, + midbase * 0.5, top),
            (+ base * 0.5, + base * 0.5, bottom), (+ base * 0.5, - base * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # bottom
            (- midbase * 0.5, - midbase * 0.5, top), (- midbase * 0.5, + midbase * 0.5, top),
            (- base * 0.5, + base * 0.5, bottom), (- base * 0.5, - base * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # right
            (- base * 0.5, + base * 0.5, bottom), (- midbase * 0.5, + midbase * 0.5, top),
            (+ midbase * 0.5, + midbase * 0.5, top), (+ base * 0.5, + base * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # left
            (- base * 0.5, - base * 0.5, bottom), (- midbase * 0.5, - midbase * 0.5, top),
            (+ midbase * 0.5, - midbase * 0.5, top), (+ base * 0.5, - base * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # top
            (- midbase * 0.5, - midbase * 0.5, roof), (- midbase * 0.5, + midbase * 0.5, roof),
            (+ midbase * 0.5, + midbase * 0.5, roof), (+ midbase * 0.5, - midbase * 0.5, roof)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # front
            (+ midbase * 0.5, - midbase * 0.5, roof), (+ midbase * 0.5, + midbase * 0.5, roof),
            (+ midbase * 0.5, + midbase * 0.5, top), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # bottom
            (- midbase * 0.5, - midbase * 0.5, roof), (- midbase * 0.5, + midbase * 0.5, roof),
            (- midbase * 0.5, + midbase * 0.5, top), (- midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # right
            (- midbase * 0.5, + midbase * 0.5, top), (- midbase * 0.5, + midbase * 0.5, roof),
            (+ midbase * 0.5, + midbase * 0.5, roof), (+ midbase * 0.5, + midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # left
            (- midbase * 0.5, - midbase * 0.5, top), (- midbase * 0.5, - midbase * 0.5, roof),
            (+ midbase * 0.5, - midbase * 0.5, roof), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################

        top = roof
        roof = 6

        list = [  # top
            (- midbase * 0.5, - midbase * 0.5, roof), (- midbase * 0.5, + midbase * 0.5, roof),
            (+ midbase * 0.5, + midbase * 0.5, roof), (+ midbase * 0.5, - midbase * 0.5, roof)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])

        biglist.append(list)
        ################################################################################################################
        list = [  # front
            (+ midbase * 0.5, - midbase * 0.5, roof), (+ midbase * 0.5, + midbase * 0.5, roof),
            (+ midbase * 0.5, + midbase * 0.5, top), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])

        biglist.append(list)
        ################################################################################################################
        list = [  # bottom
            (- midbase * 0.5, - midbase * 0.5, roof), (- midbase * 0.5, + midbase * 0.5, roof),
            (- midbase * 0.5, + midbase * 0.5, top), (- midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # right
            (- midbase * 0.5, + midbase * 0.5, top), (- midbase * 0.5, + midbase * 0.5, roof),
            (+ midbase * 0.5, + midbase * 0.5, roof), (+ midbase * 0.5, + midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # left
            (- midbase * 0.5, - midbase * 0.5, top), (- midbase * 0.5, - midbase * 0.5, roof),
            (+ midbase * 0.5, - midbase * 0.5, roof), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################

        top = roof
        roof = 8
        ################################################################################################################
        biglist.append(list)
        list = [  # top
            (- midbase * 0.0, - midbase * 0.0, roof), (- midbase * 0.0, + midbase * 0.0, roof),
            (+ midbase * 0.0, + midbase * 0.0, roof), (+ midbase * 0.0, - midbase * 0.0, roof)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # front
            (+ midbase * 0.0, - midbase * 0.0, roof), (+ midbase * 0.0, + midbase * 0.0, roof),
            (+ midbase * 0.5, + midbase * 0.5, top), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])

        biglist.append(list)
        ################################################################################################################
        list = [  # bottom
            (- midbase * 0.0, - midbase * 0.0, roof), (- midbase * 0.0, + midbase * 0.0, roof),
            (- midbase * 0.5, + midbase * 0.5, top), (- midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # right
            (- midbase * 0.5, + midbase * 0.5, top), (- midbase * 0.0, + midbase * 0.0, roof),
            (+ midbase * 0.0, + midbase * 0.0, roof), (+ midbase * 0.5, + midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [  # left
            (- midbase * 0.5, - midbase * 0.5, top), (- midbase * 0.0, - midbase * 0.0, roof),
            (+ midbase * 0.0, - midbase * 0.0, roof), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        obstable.append(list[0])
        biglist.append(list)
        ################################################################################################################

        colors = ["tab:blue" for patch in list]
        list_patchcollection = []

        patchcollection = Poly3DCollection(biglist, linewidth=0.1, edgecolor="k", facecolor=colors, rasterized=True,
                                               alpha=0)
        ax.add_collection3d(patchcollection)



def rotation_z(o, liste: list):
    # rotation vector
    # x'= xcos(o)-ysin(o)
    # y'= xsin(o)+ycos(o)
    list_tempo = []
    for i in range(len(liste)):
        list_tempo.append((liste[i][0] * np.cos(o) - liste[i][1] * np.sin(o),
                           liste[i][0] * np.sin(o) + liste[i][1] * np.cos(o), liste[i][2]))
    return list_tempo


def add_center(x, y, liste: list):
    # rotation vector
    # x'= xcos(o)-ysin(o)
    # y'= xsin(o)+ycos(o)
    list_tempo = []
    for i in range(len(liste)):
        list_tempo.append((liste[i][0] + x, liste[i][1] + y, liste[i][2] ))
    return list_tempo
