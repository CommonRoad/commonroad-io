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
from fonction import *


class bike():

    def __init__(self,scenario:Scenario,i:int,ax):
        """

        class to build a bike

        :param scenario: scenario in which the information is stored
        :param: i index to find the dinamic obstacle
        :param ax: the figure where the drone should be constructed
        :param: r radius of the wheel
        :param: accurate precision of the shape of the bike
        :param: list_return stoke forms
        """
        self._scenario=scenario
        self.list_return = []
        self.r = 0.5
        self.i=i
        pi = math.pi
        self.accurate = 10
        for t in range(self._scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = self._scenario.dynamic_obstacles[i].occupancy_at_time(t).shape

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
                list.append((self.r * np.sin(theta),0, top / 2 + self.r * np.cos(theta)))

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


    def __str__(self):
        traffic_str = "\n"
        traffic_str += "Bike:\n"
        traffic_str += "- scenario: {}\n".format(self._scenario.__str__())
        traffic_str += "- index: {}\n".format(self.i)

        return traffic_str

    def __eq__(self, other):
        if self._scenario == other.scenario and self.list_return == other.list_return  :
            return True
        return False

    @property
    def ax(self) :
        return self._ax

    @ax.setter
    def ax(self, ax):
        self._ax = ax

    @property
    def scenario(self) :
        return self._scenario

    @scenario.setter
    def scenario(self, scenario):
        self._scenario = scenario