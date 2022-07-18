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
from fonction import *


class pedestrian():
    def __init__(self ,scenario :Scenario, i :int ,ax):
        """

        class to build the pedestrian
        :param scenario: scenario in which the information is stored
        :param: i index to find the dinamic obstacle
        :param: list_return stoke forms
        :param ax: the figure where the drone should be constructed
        :param: accurate precision of the shape of the pedestrian
        :param: r radius of the person

        """

        r = 0.5
        # r=1#american
        pi = math.pi
        self.accurate = 10
        self._scenario =scenario
        self.list_retur n =[]
        self.radius=r
        self.i=i


        for t in range(self._scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = self._scenario.dynamic_obstacles[i].occupancy_at_time(t).shape
            top = 2

            biglist = []
            orientation = 0
            orientationy = 0
            list = []
            for j in range(self.accurate):
                theta = -pi + 2 * pi * j / self.accurate
                thetap1 = -pi + 2 * pi * (j + 1) / self.accurate
                a = (self.radius * np.sin(theta), top / 2 + r * np.cos(theta), top)
                b = (self.radius * np.sin(thetap1), top / 2 + r * np.cos(thetap1), 0.4)
                c = (self.radius * np.sin(thetap1), top / 2 + r * np.cos(thetap1), top)
                d = (self.radius * np.sin(theta), top / 2 + r * np.cos(theta), 0.4)

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

    def __str__(self):
        traffic_str = "\n"
        traffic_str += "pedestrian:\n"
        traffic_str += "- scenario: {}\n".format(self._scenario.__str__())
        traffic_str += "- index: {}\n".format(self.i)
        traffic_str += "- radius: {}\n".format(self.radius)
        return traffic_str
    
    

    def __eq__(self, other):
        if  self._scenario == other.scenario and self.radius == other.radius and self.accurate == other.accurate :
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