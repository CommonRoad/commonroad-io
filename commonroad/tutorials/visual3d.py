import matplotlib.pyplot as plt

from commonroad.scenario.scenario import Scenario
import matplotlib.pyplot as plt
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from commonroad.scenario.scenario import Scenario
import numpy as np

class visual():

    def __init__(self, scenario :Scenario ,Planning_Problem_Set : PlanningProblemSet):
        self.scenario=scenario
        self.pause=2
        self.list_car=[]
        self.fig = plt.figure(figsize=plt.figaspect(1) , constrained_layout=False)
        self.ax = self.fig.gca(projection='3d')
        self.list_list_patchcollection=[]
        self.top = 2.1
        self.bottom = 0.3
        self.nb_car=0
        self.planing=Planning_Problem_Set


        self.get_lanelet()

        for i in range(self.scenario.dynamic_obstacles.__len__()):
            self.list_car.append(self.construction(self.scenario,i))
            self.nb_car+=1

        for i in range(self.scenario.static_obstacles.__len__()):
            self.get_staic(i)



        self.auto_set_lim()





    def show(self):
        plt.ion()
        plt.show()

        for j in range(self.nb_car):
            for i in range(self.scenario.dynamic_obstacles[j].prediction.final_time_step):
                for y in range(len(self.list_car[0][i][:])):
                    self.list_car[j][i][y].set_alpha(0)

        for i in range(self.scenario.dynamic_obstacles[0].prediction.final_time_step):
            plt.pause(self.pause)
            for k in range(self.nb_car):
                for j in range(self.scenario.dynamic_obstacles[k].prediction.final_time_step):
                    if self.scenario.dynamic_obstacles[k].prediction.final_time_step < i:
                        for y in range(len(
                                self.list_car[k][self.scenario.dynamic_obstacles[k].prediction.final_time_step - 1][
                                :])):
                            self.list_car[k][self.scenario.dynamic_obstacles[k].prediction.final_time_step - 1][
                                y].set_alpha(0)
                    if i == j:
                        for y in range(len(self.list_car[k][i][:])):
                            if i >= 1:
                                self.list_car[k][i - 1][y].set_alpha(0)
                            self.list_car[k][i][y].set_alpha(1)
        plt.pause(20)



    def get_staic(self,i:int):
        shape = self.scenario.static_obstacles[i].occupancy_at_time(0).shape
        top = self.top
        bottom = self.bottom
        length = shape.length
        width = shape.width
        biglist = []

        ########################################################################################################################

        list = [[(- length * 0.5, - width * 0.5, bottom), (- length * 0.5, + width * 0.5, bottom),
                 (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]]

        list = list[0]
        o = shape.orientation
        # rotation vector
        # x'= xcos(o)-ysin(o)
        # y'= xsin(o)+ycos(o)

        list = [[(list[0][0] * np.cos(o) - list[0][1] * np.sin(o) + shape.center[0],
                  list[0][0] * np.sin(o) + list[0][1] * np.cos(o) + shape.center[1], list[0][2]), (
                     list[1][0] * np.cos(o) - list[1][1] * np.sin(o) + shape.center[0],
                     list[1][0] * np.sin(o) + list[1][1] * np.cos(o) + shape.center[1], list[1][2]), (
                     list[2][0] * np.cos(o) - list[2][1] * np.sin(o) + shape.center[0],
                     list[2][0] * np.sin(o) + list[2][1] * np.cos(o) + shape.center[1], list[2][2]), (
                     list[3][0] * np.cos(o) - list[3][1] * np.sin(o) + shape.center[0],
                     list[3][0] * np.sin(o) + list[3][1] * np.cos(o) + shape.center[1], list[3][2])]]

        biglist.append(list)

        ########################################################################################################################

        list = [[(- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
                 (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, - width * 0.5, top)]]

        list = list[0]
        list = [[(list[0][0] * np.cos(o) - list[0][1] * np.sin(o) + shape.center[0],
                  list[0][0] * np.sin(o) + list[0][1] * np.cos(o) + shape.center[1], list[0][2]), (
                     list[1][0] * np.cos(o) - list[1][1] * np.sin(o) + shape.center[0],
                     list[1][0] * np.sin(o) + list[1][1] * np.cos(o) + shape.center[1], list[1][2]), (
                     list[2][0] * np.cos(o) - list[2][1] * np.sin(o) + shape.center[0],
                     list[2][0] * np.sin(o) + list[2][1] * np.cos(o) + shape.center[1], list[2][2]), (
                     list[3][0] * np.cos(o) - list[3][1] * np.sin(o) + shape.center[0],
                     list[3][0] * np.sin(o) + list[3][1] * np.cos(o) + shape.center[1], list[3][2])]]

        biglist.append(list)

        ########################################################################################################################

        list = [[(+ length * 0.5, - width * 0.5, top), (+ length * 0.5, + width * 0.5, top),
                 (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]]

        list = list[0]
        list = [[(list[0][0] * np.cos(o) - list[0][1] * np.sin(o) + shape.center[0],
                  list[0][0] * np.sin(o) + list[0][1] * np.cos(o) + shape.center[1], list[0][2]), (
                     list[1][0] * np.cos(o) - list[1][1] * np.sin(o) + shape.center[0],
                     list[1][0] * np.sin(o) + list[1][1] * np.cos(o) + shape.center[1], list[1][2]), (
                     list[2][0] * np.cos(o) - list[2][1] * np.sin(o) + shape.center[0],
                     list[2][0] * np.sin(o) + list[2][1] * np.cos(o) + shape.center[1], list[2][2]), (
                     list[3][0] * np.cos(o) - list[3][1] * np.sin(o) + shape.center[0],
                     list[3][0] * np.sin(o) + list[3][1] * np.cos(o) + shape.center[1], list[3][2])]]

        biglist.append(list)
        ########################################################################################################################

        list = [[(- length * 0.5, - width * 0.5, top), (- length * 0.5, + width * 0.5, top),
                 (- length * 0.5, + width * 0.5, bottom), (- length * 0.5, - width * 0.5, bottom)]]

        list = list[0]
        list = [[(list[0][0] * np.cos(o) - list[0][1] * np.sin(o) + shape.center[0],
                  list[0][0] * np.sin(o) + list[0][1] * np.cos(o) + shape.center[1], list[0][2]), (
                     list[1][0] * np.cos(o) - list[1][1] * np.sin(o) + shape.center[0],
                     list[1][0] * np.sin(o) + list[1][1] * np.cos(o) + shape.center[1], list[1][2]), (
                     list[2][0] * np.cos(o) - list[2][1] * np.sin(o) + shape.center[0],
                     list[2][0] * np.sin(o) + list[2][1] * np.cos(o) + shape.center[1], list[2][2]), (
                     list[3][0] * np.cos(o) - list[3][1] * np.sin(o) + shape.center[0],
                     list[3][0] * np.sin(o) + list[3][1] * np.cos(o) + shape.center[1], list[3][2])]]

        biglist.append(list)

        ########################################################################################################################

        list = [[(- length * 0.5, + width * 0.5, bottom), (- length * 0.5, + width * 0.5, top),
                 (+ length * 0.5, + width * 0.5, top), (+ length * 0.5, + width * 0.5, bottom)]]

        list = list[0]
        list = [[(list[0][0] * np.cos(o) - list[0][1] * np.sin(o) + shape.center[0],
                  list[0][0] * np.sin(o) + list[0][1] * np.cos(o) + shape.center[1], list[0][2]), (
                     list[1][0] * np.cos(o) - list[1][1] * np.sin(o) + shape.center[0],
                     list[1][0] * np.sin(o) + list[1][1] * np.cos(o) + shape.center[1], list[1][2]), (
                     list[2][0] * np.cos(o) - list[2][1] * np.sin(o) + shape.center[0],
                     list[2][0] * np.sin(o) + list[2][1] * np.cos(o) + shape.center[1], list[2][2]), (
                     list[3][0] * np.cos(o) - list[3][1] * np.sin(o) + shape.center[0],
                     list[3][0] * np.sin(o) + list[3][1] * np.cos(o) + shape.center[1], list[3][2])]]

        biglist.append(list)

        ########################################################################################################################

        list = [[(- length * 0.5, - width * 0.5, bottom), (- length * 0.5, - width * 0.5, top),
                 (+ length * 0.5, - width * 0.5, top), (+ length * 0.5, - width * 0.5, bottom)]]

        list = list[0]
        list = [[(list[0][0] * np.cos(o) - list[0][1] * np.sin(o) + shape.center[0],
                  list[0][0] * np.sin(o) + list[0][1] * np.cos(o) + shape.center[1], list[0][2]), (
                     list[1][0] * np.cos(o) - list[1][1] * np.sin(o) + shape.center[0],
                     list[1][0] * np.sin(o) + list[1][1] * np.cos(o) + shape.center[1], list[1][2]), (
                     list[2][0] * np.cos(o) - list[2][1] * np.sin(o) + shape.center[0],
                     list[2][0] * np.sin(o) + list[2][1] * np.cos(o) + shape.center[1], list[2][2]), (
                     list[3][0] * np.cos(o) - list[3][1] * np.sin(o) + shape.center[0],
                     list[3][0] * np.sin(o) + list[3][1] * np.cos(o) + shape.center[1], list[3][2])]]

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
            colors = ["tab:red"]  # MWE colors
            for i in range(int(len(lanelet) / 2 )):
                a = (lanelet[i][0], lanelet[i][1], 0)
                b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], 0)
                c = (lanelet[i + 1][0], lanelet[i + 1][1], 0)
                d = (lanelet[-i][0], lanelet[-i][1], 0)
                lan = [[b, c, a, d]]
                lancolect = Poly3DCollection(lan, linewidth=0, edgecolor="k", facecolor=colors, rasterized=False,alpha=1,zorder=2)
                self.ax.add_collection3d(lancolect)



    def auto_set_lim(self):

        xmin = 1000
        ymin = 1000
        for i in range(self.scenario.dynamic_obstacles.__len__()):
            if self.scenario.dynamic_obstacles[i].occupancy_at_time(0).shape.center[0] < xmin:
                xmin = self.scenario.dynamic_obstacles[i].occupancy_at_time(0).shape.center[0]
            if self.scenario.dynamic_obstacles[i].occupancy_at_time(0).shape.center[1] < ymin:
                ymin = self.scenario.dynamic_obstacles[i].occupancy_at_time(0).shape.center[1]


        self.set_lim(xmin - 10, xmin + 90, ymin - 10, ymin + 90, -50, 50)
        plt.axis('off')


        for list_patches in self.list_list_patchcollection:
            self.time += 1
            plt.pause(self.pause)
            for i in range(self.scenario.dynamic_obstacles[0].prediction.final_time_step):
                if i == self.time:
                    for y in range(len(self.list_list_patchcollection[i][:])):
                        if i >= 1:
                            list_patches[i - 1][y].set_alpha(0)
                        list_patches[i][y].set_alpha(1)



    def set_lim(self,xmin:int,xmax:int,ymin:int,ymax:int,zmin:int,zmax:int):
        self.ax.set_xlim([xmin, xmax])
        self.ax.set_ylim([ymin, ymax])
        self.ax.set_zlim([zmin, zmax])


    def construction(self,scenario: Scenario, i: int ) -> list :

        list_list_patches = []
        for t in range(self.scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = scenario.dynamic_obstacles[i].occupancy_at_time(t).shape
            top = self.top
            bottom = self.bottom
            length = shape.length
            width = shape.width
            biglist = []

            ########################################################################################################################

            list = [
                [(- length * 0.5, - width * 0.5, bottom), (- length * 0.5, + width * 0.5, bottom),
                 (+ length * 0.5, + width * 0.5, bottom), (+ length * 0.5, - width * 0.5, bottom)]]

            list = list[0]
            o = shape.orientation
            # rotation vector
            # x'= xcos(o)-ysin(o)
            # y'= xsin(o)+ycos(o)

            list = [[(list[0][0] * np.cos(o) - list[0][1] * np.sin(o) + shape.center[0],
                      list[0][0] * np.sin(o) + list[0][1] * np.cos(o) + shape.center[1], list[0][2]), (
                      list[1][0] * np.cos(o) - list[1][1] * np.sin(o) + shape.center[0],
                      list[1][0] * np.sin(o) + list[1][1] * np.cos(o) + shape.center[1], list[1][2]), (
                      list[2][0] * np.cos(o) - list[2][1] * np.sin(o) + shape.center[0],
                      list[2][0] * np.sin(o) + list[2][1] * np.cos(o) + shape.center[1], list[2][2]), (
                      list[3][0] * np.cos(o) - list[3][1] * np.sin(o) + shape.center[0],
                      list[3][0] * np.sin(o) + list[3][1] * np.cos(o) + shape.center[1], list[3][2])]]

            biglist.append(list)

            ########################################################################################################################

            list = [[(- length * 0.5, - width * 0.5, top),
                     (- length * 0.5, + width * 0.5, top),
                     (+ length * 0.5, + width * 0.5, top),
                     (+ length * 0.5, - width * 0.5, top)]]

            list = list[0]
            list = [[(list[0][0] * np.cos(o) - list[0][1] * np.sin(o) + shape.center[0],
                      list[0][0] * np.sin(o) + list[0][1] * np.cos(o) + shape.center[1], list[0][2]), (
                      list[1][0] * np.cos(o) - list[1][1] * np.sin(o) + shape.center[0],
                      list[1][0] * np.sin(o) + list[1][1] * np.cos(o) + shape.center[1], list[1][2]), (
                      list[2][0] * np.cos(o) - list[2][1] * np.sin(o) + shape.center[0],
                      list[2][0] * np.sin(o) + list[2][1] * np.cos(o) + shape.center[1], list[2][2]), (
                      list[3][0] * np.cos(o) - list[3][1] * np.sin(o) + shape.center[0],
                      list[3][0] * np.sin(o) + list[3][1] * np.cos(o) + shape.center[1], list[3][2])]]

            biglist.append(list)

            ########################################################################################################################

            list = [[(+ length * 0.5, - width * 0.5, top),
                     (+ length * 0.5, + width * 0.5, top),
                     (+ length * 0.5, + width * 0.5, bottom),
                     (+ length * 0.5, - width * 0.5, bottom)]]

            list = list[0]
            list = [[(list[0][0] * np.cos(o) - list[0][1] * np.sin(o) + shape.center[0],
                      list[0][0] * np.sin(o) + list[0][1] * np.cos(o) + shape.center[1], list[0][2]), (
                      list[1][0] * np.cos(o) - list[1][1] * np.sin(o) + shape.center[0],
                      list[1][0] * np.sin(o) + list[1][1] * np.cos(o) + shape.center[1], list[1][2]), (
                      list[2][0] * np.cos(o) - list[2][1] * np.sin(o) + shape.center[0],
                      list[2][0] * np.sin(o) + list[2][1] * np.cos(o) + shape.center[1], list[2][2]), (
                      list[3][0] * np.cos(o) - list[3][1] * np.sin(o) + shape.center[0],
                      list[3][0] * np.sin(o) + list[3][1] * np.cos(o) + shape.center[1], list[3][2])]]

            biglist.append(list)
            ########################################################################################################################

            list = [[(- length * 0.5, - width * 0.5, top),
                     (- length * 0.5, + width * 0.5, top),
                     (- length * 0.5, + width * 0.5, bottom),
                     (- length * 0.5, - width * 0.5, bottom)]]

            list = list[0]
            list = [[(list[0][0] * np.cos(o) - list[0][1] * np.sin(o) + shape.center[0],
                      list[0][0] * np.sin(o) + list[0][1] * np.cos(o) + shape.center[1], list[0][2]), (
                      list[1][0] * np.cos(o) - list[1][1] * np.sin(o) + shape.center[0],
                      list[1][0] * np.sin(o) + list[1][1] * np.cos(o) + shape.center[1], list[1][2]), (
                      list[2][0] * np.cos(o) - list[2][1] * np.sin(o) + shape.center[0],
                      list[2][0] * np.sin(o) + list[2][1] * np.cos(o) + shape.center[1], list[2][2]), (
                      list[3][0] * np.cos(o) - list[3][1] * np.sin(o) + shape.center[0],
                      list[3][0] * np.sin(o) + list[3][1] * np.cos(o) + shape.center[1], list[3][2])]]

            biglist.append(list)

            ########################################################################################################################

            list = [[(- length * 0.5, + width * 0.5, bottom),
                     (- length * 0.5, + width * 0.5, top),
                     (+ length * 0.5, + width * 0.5, top),
                     (+ length * 0.5, + width * 0.5, bottom)]]

            list = list[0]
            list = [[(list[0][0] * np.cos(o) - list[0][1] * np.sin(o) + shape.center[0],
                      list[0][0] * np.sin(o) + list[0][1] * np.cos(o) + shape.center[1], list[0][2]), (
                      list[1][0] * np.cos(o) - list[1][1] * np.sin(o) + shape.center[0],
                      list[1][0] * np.sin(o) + list[1][1] * np.cos(o) + shape.center[1], list[1][2]), (
                      list[2][0] * np.cos(o) - list[2][1] * np.sin(o) + shape.center[0],
                      list[2][0] * np.sin(o) + list[2][1] * np.cos(o) + shape.center[1], list[2][2]), (
                      list[3][0] * np.cos(o) - list[3][1] * np.sin(o) + shape.center[0],
                      list[3][0] * np.sin(o) + list[3][1] * np.cos(o) + shape.center[1], list[3][2])]]

            biglist.append(list)

            ########################################################################################################################

            list = [[(- length * 0.5, - width * 0.5, bottom),
                     (- length * 0.5, - width * 0.5, top),
                     (+ length * 0.5, - width * 0.5, top),
                     (+ length * 0.5, - width * 0.5, bottom)]]

            list = list[0]
            list = [[(list[0][0] * np.cos(o) - list[0][1] * np.sin(o) + shape.center[0],
                      list[0][0] * np.sin(o) + list[0][1] * np.cos(o) + shape.center[1], list[0][2]), (
                      list[1][0] * np.cos(o) - list[1][1] * np.sin(o) + shape.center[0],
                      list[1][0] * np.sin(o) + list[1][1] * np.cos(o) + shape.center[1], list[1][2]), (
                      list[2][0] * np.cos(o) - list[2][1] * np.sin(o) + shape.center[0],
                      list[2][0] * np.sin(o) + list[2][1] * np.cos(o) + shape.center[1], list[2][2]), (
                      list[3][0] * np.cos(o) - list[3][1] * np.sin(o) + shape.center[0],
                      list[3][0] * np.sin(o) + list[3][1] * np.cos(o) + shape.center[1], list[3][2])]]

            biglist.append(list)

            ########################################################################################################################

            colors = ["tab:blue" for patch in list]
            list_patchcollection=[]
            for list in biglist:
                patchcollection = Poly3DCollection(list, linewidth=0, edgecolor="k", facecolor=colors, rasterized=True)
                self.ax.add_collection3d(patchcollection)
                list_patchcollection.append(patchcollection)
            list_list_patches.append(list_patchcollection)

        return list_list_patches
