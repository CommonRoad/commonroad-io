import matplotlib.pyplot as plt
import click
from commonroad.scenario.scenario import Scenario
import matplotlib.pyplot as plt
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from commonroad.scenario.scenario import Scenario
import numpy as np


class visual():

    def __init__(self, scenario: Scenario, Planning_Problem_Set: PlanningProblemSet):
        self.scenario = scenario
        self.pause = 2
        self.list_car = []
        self.list_carb = []
        self.fig = plt.figure(figsize=plt.figaspect(1), constrained_layout=False)
        self.ax = self.fig.gca(projection='3d')
        self.list_list_patchcollection = []
        self.top = 1.7
        self.bottom = 0.6
        self.nb_car = 0
        self.planing = Planning_Problem_Set
        self.previous_time = -1
        self.switch = 1
        self.xmin = 0
        self.xmax = 0
        self.ymin = 0
        self.ymax = 0
        self.zmin = 50
        self.zmax = 50
        self.zoom_factor = 5
        self.midx = (self.xmin + self.xmax) / 2
        self.midy = (self.ymin + self.ymax) / 2
        self.bridge=[]

        self.get_lanelet()
        self.get_bridge()

        for i in range(self.scenario.dynamic_obstacles.__len__()):
            self.list_car.append(self.construction(self.scenario, i))
            self.list_carb.append(self.constructionb(self.scenario, i))
            self.nb_car += 1

        for i in range(self.scenario.static_obstacles.__len__()):
            self.get_staic(i)

        self.auto_set_lim()
        self.get_tree(15,15,6)

        plt.ion()
        plt.show()

    def init_show(self):

        for j in range(self.nb_car):
            for i in range(self.scenario.dynamic_obstacles[j].prediction.final_time_step):
                for y in range(len(self.list_car[0][i][:])):
                    self.list_car[j][i][y].set_alpha(0)
                    self.list_carb[j][i][y].set_alpha(0)

    def show(self ):
        i = 0
        self.ax.view_init(elev=20, azim=-20)
        while True:
            i += 1
            for k in range(self.nb_car):
                for j in range(self.scenario.dynamic_obstacles[k].prediction.final_time_step):
                    if self.scenario.dynamic_obstacles[k].prediction.final_time_step < i:
                        for y in range(len(
                                self.list_car[k][self.scenario.dynamic_obstacles[k].prediction.final_time_step - 1][
                                :])):
                            self.list_car[k][self.scenario.dynamic_obstacles[k].prediction.final_time_step - 1][
                                y].set_alpha(0)
                            self.list_car[k][self.scenario.dynamic_obstacles[k].prediction.final_time_step - 1][
                                y].set_linewidth(0)
                            self.list_carb[k][self.scenario.dynamic_obstacles[k].prediction.final_time_step - 1][
                                y].set_alpha(0)
                            self.list_carb[k][self.scenario.dynamic_obstacles[k].prediction.final_time_step - 1][
                                y].set_linewidth(0)
                    if i == j:
                        for y in range(len(self.list_car[k][i][:])):
                            if i >= 1:
                                self.list_car[k][i - 1][y].set_alpha(0)
                                self.list_car[k][i - 1][y].set_linewidth(0)
                                self.list_carb[k][i - 1][y].set_alpha(0)
                                self.list_carb[k][i - 1][y].set_linewidth(0)
                            self.list_car[k][i][y].set_alpha(1)
                            self.list_car[k][i][y].set_linewidth(0.1)
                            self.list_carb[k][i][y].set_alpha(1)
                            self.list_carb[k][i][y].set_linewidth(0.1)
            plt.pause(0.01)

    def show_at_time(self, i: int):
        for k in range(self.nb_car):
            for j in range(self.scenario.dynamic_obstacles[k].prediction.final_time_step):
                if self.scenario.dynamic_obstacles[k].prediction.final_time_step < i:
                    for y in range(len(
                            self.list_car[k][self.scenario.dynamic_obstacles[k].prediction.final_time_step - 1][:])):
                        self.list_car[k][self.scenario.dynamic_obstacles[k].prediction.final_time_step - 1][
                            y].set_alpha(0)
                        self.list_car[k][self.scenario.dynamic_obstacles[k].prediction.final_time_step - 1][
                            y].set_linewidth(0)
                if i == j:
                    for y in range(len(self.list_car[k][i][:])):
                        if i >= 1:
                            self.list_car[k][self.previous_time][y].set_alpha(0)
                            self.list_car[k][self.previous_time][y].set_linewidth(0)
                        self.list_car[k][i][y].set_alpha(1)
                        self.list_car[k][i][y].set_linewidth(0.1)
        self.previous_time = i
        plt.pause(0.01)

    def get_staic(self, i: int):
        shape = self.scenario.static_obstacles[i].occupancy_at_time(0).shape
        top = self.top
        bottom = self.bottom
        length = shape.length
        width = shape.width
        biglist = []
        o = shape.orientation
        ########################################################################################################################

        list = [(- length * 0.5, - width * 0.5, bottom), (- length * 0.5, + width * 0.5, bottom),
                (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(shape.center[0], shape.center[1], list)

        biglist.append(list)

        ########################################################################################################################

        list = [(- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
                (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, - width * 0.5, top)]
        list = rotation_z(o, list)
        list = add_center(shape.center[0], shape.center[1], list)
        biglist.append(list)

        ########################################################################################################################

        list = [(+ length * 0.5, - width * 0.5, top), (+ length * 0.5, + width * 0.5, top),
                (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(shape.center[0], shape.center[1], list)

        biglist.append(list)
        ########################################################################################################################

        list = [(- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
                (- length * 0.5, + width * 0.5, bottom), (- length * 0.5, - width * 0.5, bottom)]

        list = rotation_z(o, list)
        list = add_center(shape.center[0], shape.center[1], list)

        biglist.append(list)

        ########################################################################################################################

        list = [(- length * 0.5, + width * 0.5, bottom), (- length * 0.5, + width * 0.5, top),
                (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, + width * 0.5, bottom)]
        list = rotation_z(o, list)
        list = add_center(shape.center[0], shape.center[1], list)
        biglist.append(list)

        ########################################################################################################################

        list = [(- length * 0.5, - width * 0.5, bottom), (- length * 0.5, - width * 0.5, top),
                (+ length * 0.5, - width * 0.5, top), (+ length * 0.5, - width * 0.5, bottom)]

        list = rotation_z(o, list)
        list = add_center(shape.center[0], shape.center[1], list)

        biglist.append(list)

        ########################################################################################################################

        colors = ["tab:green" for patch in list]
        list_patchcollection = []
        for list in biglist:
            patchcollection = Poly3DCollection(list, linewidth=0, edgecolor="k", facecolor=colors, rasterized=True)
            self.ax.add_collection3d(patchcollection)

    def get_lanelet(self):
        for l in range(len(self.scenario.lanelet_network.lanelets)):
            lanelet = self.scenario.lanelet_network.lanelets[l].polygon.vertices
            colors = ["tab:grey"]
            for i in range(int(len(lanelet) / 2)):
                a = (lanelet[i][0], lanelet[i][1], 0)
                b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], 0)
                c = (lanelet[i + 1][0], lanelet[i + 1][1], 0)
                d = (lanelet[-i][0], lanelet[-i][1], 0)
                lan = [[b, c, a, d]]
                self.bridge.append(lan[0])
                lancolect = Poly3DCollection(lan, linewidth=0, edgecolor="k", facecolor=colors, rasterized=False,
                                             alpha=1, zorder=1)

                self.ax.add_collection3d(lancolect)

    def auto_set_lim(self):

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

        self.xmin = self.midx - self.zoom_factor * 10
        self.xmax = self.midx + self.zoom_factor * 10
        self.ymin = self.midy - self.zoom_factor * 10
        self.ymax = self.midy + self.zoom_factor * 10

        self.ax.set_xlim([self.xmin, self.xmax])
        self.ax.set_ylim([self.ymin, self.ymax])

    def zoom_in(self):
        self.zoom_factor -= 1

        self.xmin = self.midx - self.zoom_factor * 10
        self.xmax = self.midx + self.zoom_factor * 10
        self.ymin = self.midy - self.zoom_factor * 10
        self.ymax = self.midy + self.zoom_factor * 10

        self.ax.set_xlim([self.xmin, self.xmax])
        self.ax.set_ylim([self.ymin, self.ymax])

    def go_up(self):
        self.midy += 5

        self.xmin = self.midx - self.zoom_factor * 10
        self.xmax = self.midx + self.zoom_factor * 10
        self.ymin = self.midy - self.zoom_factor * 10
        self.ymax = self.midy + self.zoom_factor * 10

        self.ax.set_xlim([self.xmin, self.xmax])
        self.ax.set_ylim([self.ymin, self.ymax])

    def go_down(self):
        self.midy -= 3

        self.xmin = self.midx - self.zoom_factor * 10
        self.xmax = self.midx + self.zoom_factor * 10
        self.ymin = self.midy - self.zoom_factor * 10
        self.ymax = self.midy + self.zoom_factor * 10

        self.ax.set_xlim([self.xmin, self.xmax])
        self.ax.set_ylim([self.ymin, self.ymax])

    def go_left(self):
        self.midx -= 3

        self.xmin = self.midx - self.zoom_factor * 10
        self.xmax = self.midx + self.zoom_factor * 10
        self.ymin = self.midy - self.zoom_factor * 10
        self.ymax = self.midy + self.zoom_factor * 10

        self.ax.set_xlim([self.xmin, self.xmax])
        self.ax.set_ylim([self.ymin, self.ymax])

    def go_right(self):
        self.midx += 3

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
        self.switch += 1
        if self.switch % 2 == 0:
            for k in range(self.nb_car):
                for j in range(self.scenario.dynamic_obstacles[k].prediction.final_time_step):
                    self.list_car[k][j][2].set_color("tab:orange")
        else:
            for k in range(self.nb_car):
                for j in range(self.scenario.dynamic_obstacles[k].prediction.final_time_step):
                    self.list_car[k][j][2].set_color("tab:blue")

    def construction(self, scenario: Scenario, i: int) -> list:

        list_list_patches = []
        for t in range(self.scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = scenario.dynamic_obstacles[i].occupancy_at_time(t).shape
            roof = self.top * 2
            top = self.top
            bottom = self.bottom
            length = shape.length
            width = shape.width
            biglist = []
            o = shape.orientation
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
            """
            ########################################################################################################################

            list = [  # top
                (- length * 0.5, - width * 0.5, roof), (- length * 0.5, + width * 0.5, roof),
                (+ length * 0.0, + width * 0.5, roof), (+ length * 0.0, - width * 0.5, roof)]

            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # front
                (+ length * 0.0, - width * 0.5, roof), (+ length * 0.0, + width * 0.5, roof),
                (+ length * 0.0, + width * 0.0, top), (+ length * 0.0, - width * 0.5, top)]

            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)
            ########################################################################################################################

            list = [  # bottom
                (- length * 0.5, - width * 0.5, roof), (- length * 0.5, + width * 0.5, roof),
                (- length * 0.5, + width * 0.5, top), (- length * 0.5, - width * 0.5, top)]
            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # right
                (- length * 0.5, + width * 0.5, top), (- length * 0.5, + width * 0.5, roof),
                (+ length * 0.0, + width * 0.5, roof), (+ length * 0.0, + width * 0.5, top)]

            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # left
                (- length * 0.5, - width * 0.5, top), (- length * 0.5, - width * 0.5, roof),
                (+ length * 0.0, - width * 0.5, roof), (+ length * 0.0, - width * 0.5, top)]

            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)

            ########################################################################################################################
            """

            colors = ["tab:blue" for patch in list]
            list_patchcollection = []
            for list in biglist:
                patchcollection = Poly3DCollection(list, linewidth=0, edgecolor="k", facecolor=colors, rasterized=True,
                                                   zorder=10)
                self.ax.add_collection3d(patchcollection)
                list_patchcollection.append(patchcollection)
            list_list_patches.append(list_patchcollection)

        return list_list_patches




    def get_bridge(self):

        for l in range(len(self.scenario.lanelet_network.lanelets)):
            lanelet = self.scenario.lanelet_network.lanelets[l].polygon.vertices
            bridge=[]
            colors = ["tab:grey"]
            for i in range(int(len(lanelet) / 2)):
                a = (lanelet[i][0], lanelet[i][1], 20*2**((-(i-100)**2)/600))
                b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], 20*2**((-(i-100)**2)/600))
                c = (lanelet[i + 1][0], lanelet[i + 1][1], 20*2**((-(i-100)**2)/600))
                d = (lanelet[-i][0], lanelet[-i][1], 20*2**((-(i-100)**2)/600))
                lan = [b, c, a, d]
                lan=rotation_z(3.1415/2,lan)
                bridge=add_center(+100,-100,lan)
                self.bridge.append(bridge[0])
                lancolect = Poly3DCollection(bridge, linewidth=0, edgecolor="k", facecolor=colors, rasterized=False,
                                             alpha=1, zorder=1)

                self.ax.add_collection3d(lancolect)





    def get_tree(self,x:int,y:int,z:int ):

        r = 2
        wr=0.5

        accurate = 10
        pi = 3.1415

        list=[]
        list2=[]
        patches = []
        patches2 = []



        for i in range(accurate):
            for j in range(accurate):
                phi = -pi / 2 + pi * i / accurate
                theta = -pi + 2 * pi * j / accurate

                list.append((r * np.sin(theta) * np.cos(phi) + x, r * np.sin(theta) * np.sin(phi) + y,
                                 r * 1.6 * np.cos(theta) + z))
                list2.append((wr * np.cos(theta) + x, wr * np.sin(theta) + y, i * z / accurate))

        patches.append(list)
        patches2.append(list2)

        colors = ["tab:green" for patch in patches]
        patchcollection = Poly3DCollection(patches, edgecolor="g", facecolor=colors, rasterized=True)
        self.ax.add_collection3d(patchcollection)

        colors = ["tab:green" for patch in patches2]
        patchcollection = Poly3DCollection(patches2, edgecolor="g", facecolor=colors, rasterized=True)
        self.ax.add_collection3d(patchcollection)

    def constructionb(self, scenario: Scenario, i: int) -> list:
        pi=3.1415
        list_list_patches = []
        for t in range(self.scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = scenario.dynamic_obstacles[i].occupancy_at_time(t).shape


            bottom=0
            for lan in self.bridge:

                while abs(shape.center[0]-lan[0][1]-100)<5 and abs(shape.center[1]-lan[0][0]+100)<5 and bottom<lan[0][2]:
                    bottom+=0.01


            length = shape.length

            top = bottom+2
            width = shape.width
            biglist = []

            o = shape.orientation+pi/2
            #############################################################################################################

            list = [  # underneath
                (- length * 0.5, - width * 0.5, bottom), (- length * 0.5, + width * 0.5, bottom),
                (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]

            list = rotation_z(o, list)
            list = add_centerb(shape.center[0]-100, shape.center[1]+100, list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # top
                (- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
                (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, - width * 0.5, top)]

            list = rotation_z(o, list)
            list = add_centerb(shape.center[0]-100, shape.center[1]+100, list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # front
                (+ length * 0.5, - width * 0.5, top), (+ length * 0.5, + width * 0.5, top),
                (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]

            list = rotation_z(o, list)
            list = add_centerb(shape.center[0]-100, shape.center[1]+100, list)

            biglist.append(list)
            ########################################################################################################################

            list = [  # bottom
                (- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
                (- length * 0.5, + width * 0.5, bottom), (- length * 0.5, - width * 0.5, bottom)]
            list = rotation_z(o, list)
            list = add_centerb(shape.center[0]-100, shape.center[1]+100, list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # right
                (- length * 0.5, + width * 0.5, bottom), (- length * 0.5, + width * 0.5, top),
                (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, + width * 0.5, bottom)]

            list = rotation_z(o, list)
            list = add_centerb(shape.center[0]-100, shape.center[1]+100, list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # left
                (- length * 0.5, - width * 0.5, bottom), (- length * 0.5, - width * 0.5, top),
                (+ length * 0.5, - width * 0.5, top), (+ length * 0.5, - width * 0.5, bottom)]

            list = rotation_z(o, list)
            list = add_centerb(shape.center[0]-100, shape.center[1]+100, list)

            biglist.append(list)


            colors = ["tab:blue" for patch in list]
            list_patchcollection = []
            for list in biglist:
                patchcollection = Poly3DCollection(list, linewidth=0, edgecolor="k", facecolor=colors, rasterized=True,
                                                   zorder=10)
                self.ax.add_collection3d(patchcollection)
                list_patchcollection.append(patchcollection)
            list_list_patches.append(list_patchcollection)

        return list_list_patches


def rotation_z(o, liste: list):
    # rotation vector
    # x'= xcos(o)-ysin(o)
    # y'= xsin(o)+ycos(o)
    list_tempo = []
    for i in range(4):
        list_tempo.append((liste[i][0] * np.cos(o) - liste[i][1] * np.sin(o),
                           liste[i][0] * np.sin(o) + liste[i][1] * np.cos(o), liste[i][2]))
    return list_tempo


def add_center(x: float, y: float, list: list):
    list_tempo = []
    list_ret = []
    for i in range(4):
        list_tempo.append((list[i][0] + x, list[i][1] + y, list[i][2]))
        list_ret.append(list_tempo)
    return list_ret

def add_centerb(x: float, y: float, list: list):
    list_tempo = []
    list_ret = []
    for i in range(4):
        list_tempo.append((list[i][0] + y, list[i][1] + x, list[i][2]))
        list_ret.append(list_tempo)
    return list_ret


def sign(i: int):
    if i < 0:
        return -1
    return 1


