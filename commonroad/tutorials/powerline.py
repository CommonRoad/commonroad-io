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
import math
from fonction import *
class poweline():



    def __init__(self ,scenario :Scenario ,ax,nb_towers,list_obstacle,list_powerline,tention):

        """
        class to build a bike

        :param: list_powerline
        :param ax: the figure where the drone should be constructed
        :param scenario: scenario in which the information is stored
        :param: accurate precision of the shape of the bike
        :param: nb_tower number of pilones
        :param: tention tention of the son
        :param list_obstacle: storage list of obstacles

        """

        self._scenario=scenario
        self._ax=ax
        self.list_obstacle=list_obstacle
        self.list_return = []
        self.list_powerline=list_powerline

        self.nb_towers = nb_towers
        acurate = 10
        self.first = 6
        self.second = 8
        self.tention = tention
        biglist = []
        o = math.pi

        for i in range(self.nb_towers):
            self.tower( fctx(i), fcty(i), fctx(i + 1), fcty(i + 1))

        for i in range(self.nb_towers):
            distance = math.pow(abs(fctx(i) - fctx(i + 1)) ** 2 + abs(fcty(i) - fcty(i + 1)) ** 2, 0.5)
            list = []

            if fctx(i) - fctx(i + 1):
                o = math.atan((fcty(i) - fcty(i + 1)) / (fctx(i) - fctx(i + 1))) + pi / 2
            for y in range(acurate + 1):
                x = -distance / 2 + y * distance / acurate

                list.append((0.5, x, self.tention * math.cosh(x / self.tention) - self.tention * math.cosh(
                    distance / 2 / self.tention) + self.first ))
            for y in range(acurate + 1):
                x = distance / 2 - y * distance / acurate

                list.append((-0.5, x, self.tention * math.cosh(x / self.tention) - self.tention * math.cosh(
                    distance / 2 / self.tention) + self.first ))

            list = rotation_z(o, list)
            list = add_center((fctx(i) + fctx(i + 1)) / 2, (fcty(i) + fcty(i + 1)) / 2, list)
            biglist.append(list)
            colors = "tab:blue"
        for list in biglist:
            for shape in list:
                for tuple in shape:
                    self.list_powerline.append(tuple)
            patchcollection = Poly3DCollection(list, linewidth=0.3, edgecolor="k", facecolor=colors, rasterized=True,
                                               zorder=10, alpha=0)
            self._ax.add_collection3d(patchcollection)



    def __str__(self):
        traffic_str = "\n"
        traffic_str += "powerline:\n"
        traffic_str += "- tention of the son: {}\n".format(self.tention)
        traffic_str += "- number of pilones: {}\n".format(self.nb_towers)
        return traffic_str

    def __eq__(self, other):
        if  self.list_return == other.list_return and self.list_powerline == other.list_powerline :
            return True
        return False

    @property
    def ax(self) :
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

    def tower(self,x,y,xp1,yp1):


        """

        Constructor of a tree object

        :param x: position x of the pilon
        :param y: position y of the pilon
        :param xp1: x position of the next pilon
        :param yp1: y position of the next pilon
        :param obstacle: list of points that the drone must not encounter

        """

        roof = 5
        top = 4
        bottom = 0
        base = 1.5
        midbase = 1



        o=abs(x-xp1)/abs(y-yp1)

        biglist = []
        ################################################################################################################
        list = [
            (- base * 0.5, - base * 0.5, bottom), (- base * 0.5, + base * 0.5, bottom),
            (+ base * 0.5, + base * 0.5, bottom), (+ base * 0.5, - base * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])

        biglist.append(list)

        ################################################################################################################
        list = [
            (- base * 0.5, - base * 0.5, bottom), (+ base * 0.5, + base * 0.5, bottom),
            (- base * 0.5, + base * 0.5, bottom), (+ base * 0.5, - base * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)

        
        ################################################################################################################
        list = [
            (- midbase * 0.5, - midbase * 0.5, top), (- midbase * 0.5, + midbase * 0.5, top),
            (+ midbase * 0.5, + midbase * 0.5, top), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (+ midbase * 0.5, - midbase * 0.5, top), (+ midbase * 0.5, + midbase * 0.5, top),
            (+ base * 0.5, + base * 0.5, bottom), (+ base * 0.5, - base * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (- midbase * 0.5, - midbase * 0.5, top), (- midbase * 0.5, + midbase * 0.5, top),
            (- base * 0.5, + base * 0.5, bottom), (- base * 0.5, - base * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (- base * 0.5, + base * 0.5, bottom), (- midbase * 0.5, + midbase * 0.5, top),
            (+ midbase * 0.5, + midbase * 0.5, top), (+ base * 0.5, + base * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (- base * 0.5, - base * 0.5, bottom), (- midbase * 0.5, - midbase * 0.5, top),
            (+ midbase * 0.5, - midbase * 0.5, top), (+ base * 0.5, - base * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (- midbase * 0.5, - midbase * 0.5, roof), (- midbase * 0.5, + midbase * 0.5, roof),
            (+ midbase * 0.5, + midbase * 0.5, roof), (+ midbase * 0.5, - midbase * 0.5, roof)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (+ midbase * 0.5, - midbase * 0.5, roof), (+ midbase * 0.5, + midbase * 0.5, roof),
            (+ midbase * 0.5, + midbase * 0.5, top), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (- midbase * 0.5, - midbase * 0.5, roof), (- midbase * 0.5, + midbase * 0.5, roof),
            (- midbase * 0.5, + midbase * 0.5, top), (- midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (- midbase * 0.5, + midbase * 0.5, top), (- midbase * 0.5, + midbase * 0.5, roof),
            (+ midbase * 0.5, + midbase * 0.5, roof), (+ midbase * 0.5, + midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (- midbase * 0.5, - midbase * 0.5, top), (- midbase * 0.5, - midbase * 0.5, roof),
            (+ midbase * 0.5, - midbase * 0.5, roof), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################

        top = roof
        roof = self.first

        ################################################################################################################
        list = [
            (- midbase * 0.5, - midbase * 0.5, roof), (- midbase * 0.5, + midbase * 0.5, roof),
            (+ midbase * 0.5, + midbase * 0.5, roof), (+ midbase * 0.5, - midbase * 0.5, roof)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])

        biglist.append(list)
        ################################################################################################################
        list = [
            (+ midbase * 0.5, - midbase * 0.5, roof), (+ midbase * 0.5, + midbase * 0.5, roof),
            (+ midbase * 0.5, + midbase * 0.5, top), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])

        biglist.append(list)
        ################################################################################################################
        list = [
            (- midbase * 0.5, - midbase * 0.5, roof), (- midbase * 0.5, + midbase * 0.5, roof),
            (- midbase * 0.5, + midbase * 0.5, top), (- midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (- midbase * 0.5, + midbase * 0.5, top), (- midbase * 0.5, + midbase * 0.5, roof),
            (+ midbase * 0.5, + midbase * 0.5, roof), (+ midbase * 0.5, + midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (- midbase * 0.5, - midbase * 0.5, top), (- midbase * 0.5, - midbase * 0.5, roof),
            (+ midbase * 0.5, - midbase * 0.5, roof), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################

        top = roof
        roof = self.second

        ################################################################################################################
        biglist.append(list)
        list = [
            (- midbase * 0.0, - midbase * 0.0, roof), (- midbase * 0.0, + midbase * 0.0, roof),
            (+ midbase * 0.0, + midbase * 0.0, roof), (+ midbase * 0.0, - midbase * 0.0, roof)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (+ midbase * 0.0, - midbase * 0.0, roof), (+ midbase * 0.0, + midbase * 0.0, roof),
            (+ midbase * 0.5, + midbase * 0.5, top), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])

        biglist.append(list)
        ################################################################################################################
        list = [
            (- midbase * 0.0, - midbase * 0.0, roof), (- midbase * 0.0, + midbase * 0.0, roof),
            (- midbase * 0.5, + midbase * 0.5, top), (- midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (- midbase * 0.5, + midbase * 0.5, top), (- midbase * 0.0, + midbase * 0.0, roof),
            (+ midbase * 0.0, + midbase * 0.0, roof), (+ midbase * 0.5, + midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################
        list = [
            (- midbase * 0.5, - midbase * 0.5, top), (- midbase * 0.0, - midbase * 0.0, roof),
            (+ midbase * 0.0, - midbase * 0.0, roof), (+ midbase * 0.5, - midbase * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(x, y, list)
        self.list_obstacle.append(list[0])
        biglist.append(list)
        ################################################################################################################

        colors = "tab:blue"
        list_patchcollection=[]
        for list in biglist:
            patchcollection = Poly3DCollection(list, linewidth=0.1, edgecolor="k", facecolor=colors, rasterized=True,
                                               alpha=0)
            self._ax.add_collection3d(patchcollection)
            list_patchcollection.append(patchcollection)
        self.list_return.append(list_patchcollection)

