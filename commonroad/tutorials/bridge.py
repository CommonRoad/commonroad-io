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
from fonction import *



class bridge():



    def __init__(self,scenario:Scenario,list_obstacle,ax):
        """
        :param scenario: scenario in which the information is stored
        :param list_obstacle: storage list of obstacles
        :param ax: the figure where the drone should be constructed
        """
        self._scenario=scenario
        self._ax=ax
        self.bridge=[]
        self.list_return=[]



        for l in range(len(self.scenario.lanelet_network.lanelets)):
            lanelet = self.scenario.lanelet_network.lanelets[l].polygon.vertices
            colors = ["tab:grey"]
            for i in range(int(len(lanelet) / 2)):
                a = (lanelet[i][0], lanelet[i][1], 20 * 2 ** ((-(i - 100) ** 2) / 600))
                b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], 20 * 2 ** ((-((i + 1) - 100) ** 2) / 600))
                c = (lanelet[i + 1][0], lanelet[i + 1][1], 20 * 2 ** ((-((i + 1) - 100) ** 2) / 600))
                d = (lanelet[-i][0], lanelet[-i][1], 20 * 2 ** ((-(i - 100) ** 2) / 600))
                lan = [b, c, a, d]
                lan = rotation_z(3.1415 / 2, lan)
                list = add_center(+100, -100, lan)
                for point in list[0]:
                    list_obstacle.append([point])
                self.bridge.append(list[0])
                lancolect = Poly3DCollection(list, linewidth=0, edgecolor="k", facecolor=colors, rasterized=False,
                                             alpha=1, zorder=1)
                ax.add_collection3d(lancolect)

    def __str__(self):
        traffic_str = "\n"
        traffic_str += "Bridge:\n"
        traffic_str += "- scenario: {}\n".format(self._scenario.__str__())
        traffic_str += "- equation: 20 * 2 ** ((-(i - 100) ** 2) / 600)"

        return traffic_str

    def __eq__(self, other):
        if self._scenario == other.scenario and self.list_return == other.list_return and self.bridge == other.bridge :
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


class car_bridge():

    def __init__(self ,scenario :Scenario, i :int ,ax,bridge):
        pi = math.pi
        self.list_return = []
        self._scenario = scenario
        self._ax=ax
        self.bridge=bridge



        for t in range(self._scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = self._scenario.dynamic_obstacles[i].occupancy_at_time(t).shape

            bottom = 0
            oy=0
            for lan in self.bridge:
                while abs(shape.center[0] - lan[0][1] - 100) < 2 and abs(
                        shape.center[1] - lan[0][0] + 100) < 2 and bottom - 0.5 < lan[3][2]:
                    bottom += 0.1
                    oy = lan[0][2] - lan[3][2]

            length = shape.length

            top = bottom + 2
            width = shape.width
            biglist = []

            o = shape.orientation + pi / 2
            #############################################################################################################

            list = [  # underneath
                (- length * 0.5, - width * 0.5, 0), (- length * 0.5, + width * 0.5, 0),
                (+ length * 0.5, + width * 0.5, 0), (+ length * 0.5, - width * 0.5, 0)]
            z = [bottom, bottom, bottom, bottom]
            list = rotation_x(oy, rotation_z(o, list))

            list = add_centerb(shape.center[0] - 100, shape.center[1] + 100, z, list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # top
                (- length * 0.5, - width * 0.5, 0), (- length * 0.5, + width * 0.5, 0),
                (+ length * 0.5, + width * 0.5, 0), (+ length * 0.5, - width * 0.5, 0)]
            z = [top, top, top, top]
            list = rotation_x(oy, rotation_z(o, list))
            list = add_centerb(shape.center[0] - 100, shape.center[1] + 100, z, list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # front
                (+ length * 0.5, - width * 0.5, 0), (+ length * 0.5, + width * 0.5, 0),
                (+ length * 0.5, + width * 0.5, 0), (+ length * 0.5, - width * 0.5, 0)]
            z = [top, top, bottom, bottom]
            list = rotation_x(oy, rotation_z(o, list))
            list = add_centerb(shape.center[0] - 100, shape.center[1] + 100, z, list)

            biglist.append(list)
            ########################################################################################################################

            list = [  # bottom
                (- length * 0.5, - width * 0.5, 0), (- length * 0.5, + width * 0.5, 0),
                (- length * 0.5, + width * 0.5, 0), (- length * 0.5, - width * 0.5, 0)]
            z = [top, top, bottom, bottom]
            list = rotation_x(oy, rotation_z(o, list))
            list = add_centerb(shape.center[0] - 100, shape.center[1] + 100, z, list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # right
                (- length * 0.5, + width * 0.5, 0), (- length * 0.5, + width * 0.5, 0),
                (+ length * 0.5, + width * 0.5, 0), (+ length * 0.5, + width * 0.5, 0)]
            z = [bottom, top, top, bottom]

            list = rotation_x(oy, rotation_z(o, list))
            list = add_centerb(shape.center[0] - 100, shape.center[1] + 100, z, list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # left
                (- length * 0.5, - width * 0.5, 0), (- length * 0.5, - width * 0.5, 0),
                (+ length * 0.5, - width * 0.5, 0), (+ length * 0.5, - width * 0.5, 0)]
            z = [bottom, top, top, bottom]
            list = rotation_x(oy, rotation_z(o, list))
            list = add_centerb(shape.center[0] - 100, shape.center[1] + 100, z, list)

            biglist.append(list)

            colors = ["tab:blue" for patch in list]
            list_patchcollection = []
            for list in biglist:
                patchcollection = Poly3DCollection(list, linewidth=0, edgecolor="k", facecolor=colors, rasterized=True,
                                                   zorder=10)
                self._ax.add_collection3d(patchcollection)
                list_patchcollection.append(patchcollection)
            self.list_return.append(list_patchcollection)



