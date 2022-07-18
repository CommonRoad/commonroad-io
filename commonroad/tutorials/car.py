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



class car():

    def __init__(self ,scenario :Scenario, i : int ,ax,flag_bus,flag_truck):
        """
        :param scenario: scenario in which the information is stored
        :param ax: the figure where the drone should be constructed
        :param flag_bus: boolean indicating the class of the vehicle
        :param flag_truck: boolean indicating the class of the vehicle
        :param: i index to find the dinamic obstacle

        """

        beta = 0
        self.list_return=[]
        self.scenario=scenario
        for t in range(self.scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = scenario.dynamic_obstacles[i].occupancy_at_time(t).shape
            top = 2
            if flag_bus:
                top = top * 2

            bottom = 0
            length = shape.length
            width = shape.width
            biglist = []
            o = shape.orientation

            if flag_truck:
                alpha = 0
                length = shape.length
                width = shape.width
                l1 = shape.length * 3 / 25
                m1 = shape.length / 25
                l2 = shape.length * 4 / 5
                v = 1
                if t < self.scenario.dynamic_obstacles[i].prediction.final_time_step - 1:
                    alpha = (shape.orientation - scenario.dynamic_obstacles[i].occupancy_at_time(
                        t + 1).shape.orientation) * 1.5
                beta_point = v * ((np.tan(alpha) / l1) - (np.sin(beta) / l2) + (
                            (m1 * np.cos(beta) * np.tan(alpha)) / (l1 * l2)))
                xabso = - length / 5 / 2
                yabso = 0
                beta = beta_point + beta
                length = length / 5

            ########################################################################################################################

            list = [  # underneath
                (- length * 0.5, - width * 0.5, bottom), (- length * 0.5, + width * 0.5, bottom),
                (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]

            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # top
                (- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
                (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, - width * 0.5, top)]

            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # front
                (+ length * 0.5, - width * 0.5, top), (+ length * 0.5, + width * 0.5, top),
                (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]

            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)
            ########################################################################################################################

            list = [  # bottom
                (- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
                (- length * 0.5, + width * 0.5, bottom), (- length * 0.5, - width * 0.5, bottom)]
            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # right
                (- length * 0.5, + width * 0.5, bottom), (- length * 0.5, + width * 0.5, top),
                (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, + width * 0.5, bottom)]

            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # left
                (- length * 0.5, - width * 0.5, bottom), (- length * 0.5, - width * 0.5, top),
                (+ length * 0.5, - width * 0.5, top), (+ length * 0.5, - width * 0.5, bottom)]

            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)

            ########################################################################################################################
            if flag_truck:
                length = length * 4
                list = [  # underneath
                    (- length * 1, - width * 0.5, bottom), (- length * 1, + width * 0.5, bottom),
                    (+ length * 0, + width * 0.5, bottom), (+ length * 0, - width * 0.5, bottom)]

                x = xabso * np.cos(beta + o) - yabso * np.sin(beta + o)
                y = xabso * np.sin(beta + o) + yabso * np.cos(beta + o)

                list = rotation_z(beta + o, list)
                list = add_center(x + shape.center[0], y + shape.center[1], list)
                biglist.append(list)

            colors = ["tab:blue" for patch in list]
            if flag_truck:
                colors = ["tab:orange" for patch in list]
            if flag_bus:
                colors = ["tab:red" for patch in list]
            list_patchcollection = []
            for list in biglist:
                patchcollection = Poly3DCollection(list, linewidth=0, edgecolor="k", facecolor=colors, rasterized=True)
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


