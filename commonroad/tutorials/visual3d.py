import matplotlib.pyplot as plt
import click
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from commonroad.scenario.scenario import Scenario
import numpy as np
import math
from commonroad.scenario.obstacle import ObstacleType

from traffic_lights import trafic_light
from bike import bike
from bridge import bridge, car_bridge
from powerline import poweline
from drone import drone3d
from car import car
from tree import Tree
from fonction import *


class visual():
    """
    The visual class aims to store all the information needed for a good visualization
    it also aims to organize the initialization of all the classes and
    it has the algorithm to create the movement

    """

    def __init__(self, scenario: Scenario):
        """
        :param scenario: scenario in which the information is stored
        """
        self._scenario = scenario
        self.list_car = []
        self.list_carb = []
        self.list_obstacle = []
        self.list_powerline = []
        self.list_target=[]
        self.list_traffic_lights = []
        self.list_tree=[]
        self.fig = plt.figure(figsize=plt.figaspect(1), constrained_layout=False)
        self._ax = self.fig.gca(projection='3d')
        self.nb_tower = 10
        self.top = 1.7
        self.bottom = 0.6
        self.nb_car = 0
        self.switch = False
        self.bridge_flag = True
        self.tree_flag = True
        self.powerline_flag = True
        self.list_traffic_signs = []

        self.xmin = 0
        self.xmax = 0
        self.ymin = 0
        self.ymax = 0
        self.zmin = 50
        self.zmax = 50
        self.zoom_factor = 5
        self.midx = (self.xmin + self.xmax) / 2
        self.midy = (self.ymin + self.ymax) / 2

        for i in range(len(self._scenario.lanelet_network.traffic_lights)):
            self.list_traffic_lights.append(
                    trafic_light(self._ax, self._scenario, self._scenario.lanelet_network.traffic_lights[i], 0.5, 3, 1.5))

            self.list_traffic_lights[-1].new_light()

        plt.axis('off')
        if self.powerline_flag:
            self.list_powerline.append(poweline(self._scenario, self._ax, self.nb_tower, self.list_obstacle, self.list_target, 30))
        if self.bridge_flag:
            b = bridge(self._scenario, self.list_obstacle, self._ax)
            for i in range(self._scenario.dynamic_obstacles.__len__()):
                self.list_carb.append(car_bridge(self._scenario, i, self._ax, b.bridge).list_return)

        self.get_lanelet()

        for i in range(self._scenario.dynamic_obstacles.__len__()):
            self.list_car.append(self.construction(i))
            self.nb_car += 1

        for i in range(self._scenario.static_obstacles.__len__()):
            self.get_staic(i)

        self.auto_set_lim()

        if self.tree_flag:
            self.list_tree.append(Tree(self._ax, 24, -8, 8, 2, 0.5, self.list_obstacle))

        self.drone = drone3d(self._ax, self.list_obstacle, self.list_target)
        self.init_show()
        plt.ion()  # interactive on
        plt.show()

    def __str__(self):
        traffic_str = "\n"
        traffic_str += "- scenario: {}\n".format(self._scenario.__str__())
        traffic_str += "- nb of car: {}\n".format(self.nb_car())

        if self.bridge_flag:
            traffic_str += "- equation of bridge: 20 * 2 ** ((-(i - 100) ** 2) / 600)"
        if self.tree_flag:
            for tree in self.list_tree:
                traffic_str+=tree.__str__()
        if self.powerline_flag:
            for poweline in self.list_powerline:
                traffic_str+=poweline.__str__()
        return traffic_str

    @property
    def ax(self):
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



    def auto_set_lim(self):
        """
        find the best view
        """
        xmin = 1000
        ymin = 1000
        for i in range(self._scenario.dynamic_obstacles.__len__()):
            if self._scenario.dynamic_obstacles[i].occupancy_at_time(0).shape.center[0] < xmin:
                xmin = self._scenario.dynamic_obstacles[i].occupancy_at_time(0).shape.center[0]
            if self._scenario.dynamic_obstacles[i].occupancy_at_time(0).shape.center[1] < ymin:
                ymin = self._scenario.dynamic_obstacles[i].occupancy_at_time(0).shape.center[1]

        self.xmin = xmin - 10
        self.xmax = xmin + 90
        self.ymin = ymin - 10
        self.ymax = ymin + 90
        self.zmin = 50
        self.zmax = 50
        self.midx = (self.xmin + self.xmax) / 2
        self.midy = (self.ymin + self.ymax) / 2
        self.set_lim(self.xmin, self.xmax, self.ymin, self.ymax, -50, 50)

        plt.axis('off')

    def zoom_out(self):

        self.zoom_factor += 1
        self.set_view()

    def zoom_in(self):
        self.zoom_factor -= 1
        self.set_view()

    def go_up(self):
        self.midy += 3
        self.set_view()

    def go_down(self):
        self.midy -= 3
        self.set_view()

    def go_left(self):
        self.midx -= 3
        self.set_view()

    def go_right(self):
        self.midx += 3
        self.set_view()

    def set_view(self):
        """
        refreshes the view
        """
        self.xmin = self.midx - self.zoom_factor * 10
        self.xmax = self.midx + self.zoom_factor * 10
        self.ymin = self.midy - self.zoom_factor * 10
        self.ymax = self.midy + self.zoom_factor * 10

        self._ax.set_xlim([self.xmin, self.xmax])
        self._ax.set_ylim([self.ymin, self.ymax])

    def set_lim(self, xmin: int, xmax: int, ymin: int, ymax: int, zmin: int, zmax: int):

        self._ax.set_xlim([xmin, xmax])
        self._ax.set_ylim([ymin, ymax])
        self._ax.set_zlim([zmin, zmax])

    def init_show(self):
        """
        to make all object's oppacity at any moment null
        """

        for j in range(self.nb_car):
            for i in range(self._scenario.dynamic_obstacles[j].prediction.final_time_step):
                for y in range(len(self.list_car[j][i][:])):
                    self.list_car[j][i][y].set_alpha(0)
                    self.list_car[j][i][y].set_linewidth(0)
                    if self.bridge_flag:
                        self.list_carb[j][i][y].set_alpha(0)
                        self.list_carb[j][i][y].set_linewidth(0)

    def show(self):
        """
        shows at every moment
        """
        i = 0
        if self.drone.follow == 1:
            self.drone.ax.set_xlim([self.drone.position[0] - 5, self.drone.position[0] + 5])
            self.drone.ax.set_ylim([self.drone.position[1] - 5, self.drone.position[1] + 5])
            self.drone.ax.set_zlim([self.drone.position[2] - 5, self.drone.position[2] + 5])
            #self.drone.ax.view_init(elev=self.drone.elev, azim=180 + self.drone.orientation * 180 /math.pi)
        while True:
            i += 1
            self.init_show()
            for traffic in self.list_traffic_lights:
                traffic.time = i
                traffic.new_light()
            for k in range(self.nb_car):
                for j in range(self._scenario.dynamic_obstacles[k].prediction.final_time_step):
                    if i == j:
                        for y in range(len(self.list_car[k][i][:])):
                            self.list_car[k][i][y].set_alpha(1)
                            self.list_car[k][i][y].set_linewidth(0.1)

                            if self.bridge_flag:
                                self.list_carb[k][i][y].set_alpha(1)
                                self.list_carb[k][i][y].set_linewidth(0.1)
            plt.pause(0.01)

    def show_at_time(self, i: int):
        """
        shows the visualization at the moment of setting the parameters

        :param: i time at which the display will be changed
        """
        self.init_show()
        for k in range(self.nb_car):
            for j in range(self._scenario.dynamic_obstacles[k].prediction.final_time_step):
                if i == j:
                    for y in range(len(self.list_car[k][i][:])):
                        self.list_car[k][i][y].set_alpha(1)
                        self.list_car[k][i][y].set_linewidth(0)
                        if self.bridge_flag:
                            self.list_carb[k][i][y].set_alpha(1)
                            self.list_carb[k][i][y].set_linewidth(0)

    def get_staic(self, i: int):

        """
        build static obstacles

        :param: i index of the static obstacles
        """
        shape = self._scenario.static_obstacles[i].occupancy_at_time(0).shape
        top = self.top
        bottom = self.bottom
        length = shape.length
        width = shape.width
        biglist = []
        o = shape.orientation
        ################################################################################################################

        list = [(- length * 0.5, - width * 0.5, bottom), (- length * 0.5, + width * 0.5, bottom),
                (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(shape.center[0], shape.center[1], list)

        biglist.append(list)

        ################################################################################################################

        list = [(- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
                (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, - width * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(shape.center[0], shape.center[1], list)
        biglist.append(list)

        ################################################################################################################

        list = [(+ length * 0.5, - width * 0.5, top), (+ length * 0.5, + width * 0.5, top),
                (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(shape.center[0], shape.center[1], list)

        biglist.append(list)
        ################################################################################################################

        list = [(- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
                (- length * 0.5, + width * 0.5, bottom), (- length * 0.5, - width * 0.5, bottom)]

        list = rotation_z(o, list)
        list = add_center(shape.center[0], shape.center[1], list)

        biglist.append(list)

        ################################################################################################################

        list = [(- length * 0.5, + width * 0.5, bottom), (- length * 0.5, + width * 0.5, top),
                (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, + width * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(shape.center[0], shape.center[1], list)
        biglist.append(list)

        ################################################################################################################

        list = [(- length * 0.5, - width * 0.5, bottom), (- length * 0.5, - width * 0.5, top),
                (+ length * 0.5, - width * 0.5, top), (+ length * 0.5, - width * 0.5, bottom)]

        list = rotation_z(o, list)
        list = add_center(shape.center[0], shape.center[1], list)

        biglist.append(list)

        ################################################################################################################

        colors = "tab:green"

        for list in biglist:
            patchcollection = Poly3DCollection(list, linewidth=0.1, edgecolor="k", facecolor=colors, rasterized=True)
            self._ax.add_collection3d(patchcollection)

    def get_lanelet(self):

        """
        build lanelet
        """
        for l in range(len(self._scenario.lanelet_network.lanelets)):

            if self._scenario.lanelet_network.lanelets[l].traffic_lights == set():
                lanelet = self._scenario.lanelet_network.lanelets[l].polygon.vertices
                colors = "tab:grey"
                for i in range(int(len(lanelet) / 2)):
                    a = (lanelet[i][0], lanelet[i][1], 0)
                    b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], 0)
                    c = (lanelet[i + 1][0], lanelet[i + 1][1], 0)
                    d = (lanelet[-i][0], lanelet[-i][1], 0)
                    lan = [[b, c, a, d]]

                    lancolect = Poly3DCollection(lan, linewidth=0, edgecolor="k", facecolor=colors, rasterized=False,
                                                 alpha=1, zorder=1)

                    self._ax.add_collection3d(lancolect)

    def switch_ligth(self):

        """
        turn the front of the car yellow
        """
        if scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.CAR:
            self.switch = not self.switch
            if self.switch:
                for k in range(self.nb_car):
                    for j in range(self._scenario.dynamic_obstacles[k].prediction.final_time_step):
                        self.list_car[k][j][2].set_color("tab:orange")
            else:
                for k in range(self.nb_car):
                    for j in range(self._scenario.dynamic_obstacles[k].prediction.final_time_step):
                        self.list_car[k][j][2].set_color("tab:blue")

    def construction(self, i: int) -> list:

        """
        constructs the shape of the obstacles according to their type

        :param: i index to find the dinamic obstacle
        :return: list shape of the dinamic obstacle
        """

        list_list_patches = []
        flag_truck = False
        flag_bus = False
        scenario = self._scenario

        if scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.TRUCK:
            flag_truck = True
        if scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.BUS:
            flag_bus = True
        if scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.BICYCLE or scenario.dynamic_obstacles[
            i].obstacle_type == ObstacleType.MOTORCYCLE:
            return bike(self._scenario, i, self._ax).list_return
        if scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.PEDESTRIAN:
            return pedestrian(self._scenario, i, self._ax).list_return
        return car(self._scenario, i, self._ax, flag_bus, flag_truck).list_return

    @property
    def x(self) -> float:
        return self.x

    @x.setter
    def orientation(self, x):
        print("")
