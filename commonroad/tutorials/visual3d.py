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


class visual():

    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.list_car = []
        self.list_carb = []
        self.list_obstacle = []
        self.list_powerline = []
        self.list_traffic_lights = []
        self.fig = plt.figure(figsize=plt.figaspect(1), constrained_layout=False)
        self.ax = self.fig.gca(projection='3d')
        self.nb_tower=10
        self.top = 1.7
        self.bottom = 0.6
        self.nb_car = 0
        self.switch = False
        self.xmin = 0
        self.xmax = 0
        self.ymin = 0
        self.ymax = 0
        self.zmin = 50
        self.zmax = 50
        self.zoom_factor = 5
        self.midx = (self.xmin + self.xmax) / 2
        self.midy = (self.ymin + self.ymax) / 2
        self.bridge_flag = True
        self.tree_flag = Tree
        self.powerline_flag = True
        self.list_traffic_signs = []

        for i in range(len(self.scenario.lanelet_network.traffic_lights)):
            self.list_traffic_lights.append(
                trafic_light(self.ax, self.scenario, self.scenario.lanelet_network.traffic_lights[i]))
            self.list_traffic_lights[-1].new_light()


        if self.powerline_flag:
            poweline(self.scenario,self.ax,self.nb_tower,self.list_obstacle,self.list_powerline,30)
        if self.bridge_flag:
            b=bridge(self.scenario,self.list_obstacle,self.ax)
            for i in range(self.scenario.dynamic_obstacles.__len__()):
                self.list_carb.append(car_bridge(self.scenario,i,self.ax,b.bridge).list_return)


        self.get_lanelet()

        for i in range(self.scenario.dynamic_obstacles.__len__()):
            self.list_car.append(self.construction( i))
            self.nb_car += 1

        for i in range(self.scenario.static_obstacles.__len__()):
            self.get_staic(i)

        self.auto_set_lim()

        if self.tree_flag:
            Tree(self.ax, 24, -8, 8, 2, 0.5, self.list_obstacle)





        self.drone = drone3d(self.ax, self.list_obstacle, self.list_powerline)
        self.init_show()
        plt.ion()#interactive on
        plt.show()

    def init_show(self):
        """
        to make all object's oppacity at any moment null
        """

        for j in range(self.nb_car):
            for i in range(self.scenario.dynamic_obstacles[j].prediction.final_time_step):
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
            self.drone.ax.set_zlim([self.drone.position[2] - 5, self.drone.position[
                2] + 5])  # self.drone.ax.view_init(elev=self.drone.elev, azim=180 + self.drone.orientation * 180 /
            # 3.1415)
        while True:
            i += 1
            self.init_show()
            for traffic in self.list_traffic_lights:
                traffic.time = i
                traffic.new_light()
            for k in range(self.nb_car):
                for j in range(self.scenario.dynamic_obstacles[k].prediction.final_time_step):
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
            for j in range(self.scenario.dynamic_obstacles[k].prediction.final_time_step):
                if i == j:
                    for y in range(len(self.list_car[k][i][:])):
                        self.list_car[k][i][y].set_alpha(1)
                        self.list_car[k][i][y].set_linewidth(0)
                        if self.bridge_flag:
                            self.list_carb[k][i][y].set_alpha(1)
                            self.list_carb[k][i][y].set_linewidth(0)



    def get_staic(self, i: int):

        """

        """
        shape = self.scenario.static_obstacles[i].occupancy_at_time(0).shape
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

        colors = ["tab:green" for patch in list]

        for list in biglist:
            patchcollection = Poly3DCollection(list, linewidth=0.1, edgecolor="k", facecolor=colors, rasterized=True)
            self.ax.add_collection3d(patchcollection)

    def get_lanelet(self):
        for l in range(len(self.scenario.lanelet_network.lanelets)):

            if self.scenario.lanelet_network.lanelets[l].traffic_lights == set():
                lanelet = self.scenario.lanelet_network.lanelets[l].polygon.vertices
                colors = ["tab:grey"]
                for i in range(int(len(lanelet) / 2)):
                    a = (lanelet[i][0], lanelet[i][1], 0)
                    b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], 0)
                    c = (lanelet[i + 1][0], lanelet[i + 1][1], 0)
                    d = (lanelet[-i][0], lanelet[-i][1], 0)
                    lan = [[b, c, a, d]]


                    lancolect = Poly3DCollection(lan, linewidth=0, edgecolor="k", facecolor=colors, rasterized=False,
                                                 alpha=1, zorder=1)

                    self.ax.add_collection3d(lancolect)

    def auto_set_lim(self):
        """
        find the best view
        """
        xmin = 1000
        ymin = 1000
        for i in range(self.scenario.dynamic_obstacles.__len__()):
            if self.scenario.dynamic_obstacles[i].occupancy_at_time(0).shape.center[0] < xmin:
                xmin = self.scenario.dynamic_obstacles[i].occupancy_at_time(0).shape.center[0]
            if self.scenario.dynamic_obstacles[i].occupancy_at_time(0).shape.center[1] < ymin:
                ymin = self.scenario.dynamic_obstacles[i].occupancy_at_time(0).shape.center[1]

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

        self.ax.set_xlim([self.xmin, self.xmax])
        self.ax.set_ylim([self.ymin, self.ymax])

    def set_lim(self, xmin: int, xmax: int, ymin: int, ymax: int, zmin: int, zmax: int):

        self.ax.set_xlim([xmin, xmax])
        self.ax.set_ylim([ymin, ymax])
        self.ax.set_zlim([zmin, zmax])

    def switch_ligth(self):

        """
        turn the front of the car yellow
        """
        self.switch = not self.switch
        if self.switch :
            for k in range(self.nb_car):
                for j in range(self.scenario.dynamic_obstacles[k].prediction.final_time_step):
                    self.list_car[k][j][2].set_color("tab:orange")
        else:
            for k in range(self.nb_car):
                for j in range(self.scenario.dynamic_obstacles[k].prediction.final_time_step):
                    self.list_car[k][j][2].set_color("tab:blue")

    def construction(self, i: int) -> list:

        """
        construit la forme des obstacle selon leur type

        :param: i index pour retrouver l'obstacle dinamique
        :return: list shape

        """

        list_list_patches = []
        flag_truck = False
        flag_bus = False
        scenario = self.scenario

        if scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.TRUCK:
            flag_truck = True
        if scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.BUS:
            flag_bus = True
        if scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.BICYCLE or scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.MOTORCYCLE:
            return bike(self.scenario,i,self.ax).list_return
        if scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.PEDESTRIAN:
            return pedestrian(self.scenario,i,self.ax).list_return
        return car(self.scenario,i,self.ax,flag_bus,flag_truck).list_return



    @property
    def x(self) -> float:
        return self.x

    @x.setter
    def orientation(self, x):
        print("parametre protege")

    @property
    def y(self) -> float:
        return self.y

    @y.setter
    def orientation(self, y):
        print("parametre protege")

    @property
    def z(self) -> float:
        return self.z

    @z.setter
    def orientation(self, z):
        print("parametre protege")

    @property
    def r(self) -> float:
        return self.r

    @r.setter
    def orientation(self, r):
        print("parametre protege")

    @property
    def wr(self) -> float:
        return self.wr

    @r.setter
    def orientation(self, wr):
        print("parametre protege")






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
