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


class visual():

    def __init__(self, scenario: Scenario, Planning_Problem_Set: PlanningProblemSet):
        self.scenario = scenario
        self.list_car = []
        self.list_carb = []
        self.list_obstacle = []
        self.list_traffic_lights=[]
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
        self.bridge_flag =0
        self.tree_flag = 0
        self.powerline_flag=0

        for i in range(len(self.scenario.lanelet_network.traffic_lights)):
            self.list_traffic_lights.append(trafic_light(self.ax, self.fig,self.scenario, self.scenario.lanelet_network.traffic_lights[i]))

        for traffic in self.list_traffic_lights:
            traffic.new_light()


        if self.bridge_flag:
            self.bridge = []

            self.get_bridge()
        self.get_lanelet()
        """
        
        for i in range(self.scenario.dynamic_obstacles.__len__()):
            self.list_car.append(self.construction(self.scenario, i))
            if self.bridge_flag:
                self.list_carb.append(self.constructionb(self.scenario, i))
            self.nb_car += 1

        for i in range(self.scenario.static_obstacles.__len__()):
            self.get_staic(i)
        """


        self.auto_set_lim()
        if self.tree_flag:
            self.get_tree(24, -8, 8)
            self.get_tree(27, -10, 8)
            self.get_tree(24, -14, 8)
        self.list_powerline = []
        if self.powerline_flag:

            self.get_powerline()


        self.drone = drone3d(self.ax, self.fig, self.list_obstacle,self.list_powerline)

        plt.ion()
        plt.show()

    def init_show(self):

        for j in range(self.nb_car):
            for i in range(self.scenario.dynamic_obstacles[j].prediction.final_time_step):
                for y in range(len(self.list_car[j][i][:])):
                    self.list_car[j][i][y].set_alpha(0)
                    self.list_car[j][i][y].set_linewidth(0)
                    if self.bridge_flag:
                        self.list_carb[j][i][y].set_alpha(0)
                        self.list_carb[j][i][y].set_linewidth(0)

    def show(self):
        i = 0

        if self.drone.follow==1:

            self.drone.ax.set_xlim([self.drone.position[0] - 5, self.drone.position[0] + 5])
            self.drone.ax.set_ylim([self.drone.position[1] - 5, self.drone.position[1] + 5])
            self.drone.ax.set_zlim([self.drone.position[2] - 5, self.drone.position[2] + 5])
            #self.drone.ax.view_init(elev=self.drone.elev, azim=180 + self.drone.orientation * 180 / 3.1415)
        while True:
            i += 1
            self.init_show()
            for k in range(self.nb_car):
                for j in range(self.scenario.dynamic_obstacles[k].prediction.final_time_step):
                    if i == j:
                        for y in range(len(self.list_car[k][i][:])):
                            self.list_car[k][i][y].set_alpha(1)
                            self.list_car[k][i][y].set_linewidth(0.1)
                            for traffic in self.list_traffic_lights:
                                traffic.time=i
                                traffic.new_light()
                            if self.bridge_flag:
                                self.list_carb[k][i][y].set_alpha(1)
                                self.list_carb[k][i][y].set_linewidth(0.1)
            plt.pause(0.01)



    def show_at_time(self, i: int):
        self.init_show()

        for k in range(self.nb_car):
            for j in range(self.scenario.dynamic_obstacles[k].prediction.final_time_step):
                if i == j:
                    for y in range(len(self.list_car[k][i][:])):
                        self.list_car[k][i][y].set_alpha(1)
                        self.list_car[k][i][y].set_linewidth(0.1)
                        if self.bridge_flag:
                            self.list_carb[k][i][y].set_alpha(1)
                            self.list_carb[k][i][y].set_linewidth(0.1)

        # plt.pause(0.01)

    def get_staic(self, i: int):
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
        list_patchcollection = []
        for list in biglist:
            patchcollection = Poly3DCollection(list, linewidth=0.1, edgecolor="k", facecolor=colors, rasterized=True)
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
                if self.bridge_flag:
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
        #self.ax.view_init(elev=20, azim=-30)
        plt.axis('off')

    def zoom_out(self):
        self.zoom_factor += 1
        self.set_view()

    def zoom_in(self):
        self.zoom_factor -= 1
        self.set_view()

    def go_up(self):
        self.midy += 5
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
        flag_truck=False
        flag_bus=False

        if scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.TRUCK:
            flag_truck=True
        if scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.BUS:
            flag_bus=True
        if scenario.dynamic_obstacles[i].obstacle_type == ObstacleType.BICYCLE:
            return self.construct_bike(i)

        beta = 0

        #TODO switch case


        for t in range(self.scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = scenario.dynamic_obstacles[i].occupancy_at_time(t).shape
            roof = self.top * 2
            top = self.top
            if flag_bus:
                top=top*2
            bottom = self.bottom
            length = shape.length
            width = shape.width
            biglist = []
            o = shape.orientation



            if flag_truck:
                alpha=0
                length = shape.length
                width = shape.width
                l1=shape.length*3/25
                m1=shape.length/25
                l2=shape.length*4/5
                v=1
                if t<self.scenario.dynamic_obstacles[i].prediction.final_time_step-1:
                    alpha=(shape.orientation-scenario.dynamic_obstacles[i].occupancy_at_time(t+1).shape.orientation)*1.5
                beta_point = v * ((np.tan(alpha) / l1) - (np.sin(beta) / l2) + ((m1 * np.cos(beta) * np.tan(alpha)) / (l1 * l2)))
                xabso = - length/5/2
                yabso = 0
                beta = beta_point+beta
                length=length/5

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

            ########################################################################################################################
            if flag_truck:
                length=length*4
                list = [  # underneath
                    (- length * 1, - width * 0.5, bottom), (- length * 1, + width * 0.5, bottom),
                    (+ length * 0, + width * 0.5, bottom), (+ length * 0, - width * 0.5, bottom)]


                x= xabso * np.cos(beta+o) - yabso * np.sin(beta+o)
                y= xabso * np.sin(beta+o) + yabso * np.cos(beta+o)

                list = rotation_z(beta+o, list)
                list = add_center(x+shape.center[0], y+shape.center[1], list)
                biglist.append(list)

            colors = ["tab:blue" for patch in list]
            if flag_truck:
                colors = ["tab:orange" for patch in list]
            if flag_bus:
                colors = ["tab:red" for patch in list]
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
            bridge = []
            colors = ["tab:grey"]
            for i in range(int(len(lanelet) / 2)):
                a = (lanelet[i][0], lanelet[i][1], 20 * 2 ** ((-(i - 100) ** 2) / 600))
                b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], 20 * 2 ** ((-((i + 1) - 100) ** 2) / 600))
                c = (lanelet[i + 1][0], lanelet[i + 1][1], 20 * 2 ** ((-((i + 1) - 100) ** 2) / 600))
                d = (lanelet[-i][0], lanelet[-i][1], 20 * 2 ** ((-(i - 100) ** 2) / 600))
                lan = [b, c, a, d]
                lan = rotation_z(3.1415 / 2, lan)
                bridge = add_center(+100, -100, lan)
                for point in bridge[0]:
                    self.list_obstacle.append(point)
                self.bridge.append(bridge[0])
                lancolect = Poly3DCollection(bridge, linewidth=0, edgecolor="k", facecolor=colors, rasterized=False,
                                             alpha=1, zorder=1)

                self.ax.add_collection3d(lancolect)

    def get_tree(self, x: int, y: int, z: int):

        r = 2
        wr = 0.5

        accurate = 50
        pi = 3.1415

        list = []
        list2 = []
        patches = []
        patches2 = []

        for i in range(accurate):
            for j in range(accurate):
                phi = -pi / 2 + pi * i / accurate
                theta = -pi + 2 * pi * j / accurate

                list.append((r * np.sin(theta) * np.cos(phi) + x, r * np.sin(theta) * np.sin(phi) + y,
                             r * 1.6 * np.cos(theta) + z))
                self.list_obstacle.append((r * np.sin(theta) * np.cos(phi) + x, r * np.sin(theta) * np.sin(phi) + y,
                                           r * 1.6 * np.cos(theta) + z))
                list2.append((wr * np.cos(theta) + x, wr * np.sin(theta) + y, i * z / accurate))
                self.list_obstacle.append((wr * np.cos(theta) + x, wr * np.sin(theta) + y, i * z / accurate))

        patches.append(list)
        patches2.append(list2)

        colors = ["tab:green" for patch in patches]
        patchcollection = Poly3DCollection(patches, edgecolor="g", facecolor=colors, rasterized=True)
        self.ax.add_collection3d(patchcollection)

        colors = ["tab:green" for patch in patches2]
        patchcollection = Poly3DCollection(patches2, edgecolor="g", facecolor=colors, rasterized=True)
        self.ax.add_collection3d(patchcollection)

    def constructionb(self, scenario: Scenario, i: int) -> list:
        pi = 3.1415
        list_list_patches = []
        for t in range(self.scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = scenario.dynamic_obstacles[i].occupancy_at_time(t).shape

            bottom = 0
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
                self.ax.add_collection3d(patchcollection)
                list_patchcollection.append(patchcollection)
            list_list_patches.append(list_patchcollection)

        return list_list_patches




    def get_powerline(self):
        nb_towers=10
        acurate=10
        alpha=30
        biglist=[]
        for i in range(nb_towers):
            poweline(self.ax,fctx(i),fcty(i),fctx(i+1),fcty(i+1),self.list_obstacle)

        for i in range(nb_towers):
            distance= math.pow(abs(fctx(i)-fctx(i+1))**2+abs(fcty(i)-fcty(i+1))**2,0.5)
            list=[]
            o=3.1415
            if fctx(i)-fctx(i+1):
                o=math.atan((fcty(i)-fcty(i+1))/(fctx(i)-fctx(i+1)))+3.1415/2
            for y in range(acurate+1):
                x=-distance/2+y*distance/acurate


                list.append((0.5, x , alpha*math.cosh(x/alpha)-alpha*math.cosh(distance/2/alpha)+6))#6 = top powerline
            for y in range(acurate+1):
                x=distance/2-y*distance/acurate


                list.append((-0.5, x , alpha*math.cosh(x/alpha)-alpha*math.cosh(distance/2/alpha)+6))#6 = top powerline


            list = rotation_z(o, list)
            list = add_center((fctx(i)+fctx(i+1))/2, (fcty(i)+fcty(i+1))/2, list)
            biglist.append(list)
            colors = ["tab:blue" for patch in list]
        for list in biglist:
            for shape in list:
                for tuple in shape:
                    self.list_powerline.append(tuple)
            patchcollection = Poly3DCollection(list, linewidth=0.3, edgecolor="k", facecolor=colors, rasterized=True,
                                                   zorder=10,alpha=0)
            self.ax.add_collection3d(patchcollection)

    def construct_bike(self,i):
        list_list_patches = []
        r=0.5
        pi=3.1415
        accurate=10
        for t in range(self.scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = self.scenario.dynamic_obstacles[i].occupancy_at_time(t).shape
            roof = self.top * 2
            top = self.top

            bottom = self.bottom
            length = shape.length
            width = shape.width
            biglist = []
            o = shape.orientation

            list = [  # underneath
                (- length * 0.5, 0, top), (+ length * 0.5, 0, top)
                , (0, 0, top/2)]

            list = rotation_z(o, list)
            list = add_center(shape.center[0], shape.center[1], list)

            biglist.append(list)
            first=list[0][0]
            last=list[0][1]

            list = []
            for j in range(accurate):
                theta = -pi + 2 * pi * j / accurate
                list.append(( r * np.sin(theta),0, top/2+r * np.cos(theta)))

            list = rotation_z(o, list)
            list = add_center(last[0], last[1], list)

            biglist.append(list)

            list = []


            for j in range(accurate):
                theta = -pi + 2 * pi * j / accurate
                list.append(( r * np.sin(theta),0, top/2+r * np.cos(theta)))

            list = rotation_z(o, list)
            list = add_center(first[0], first[1], list)
            biglist.append(list)

            list_patchcollection = []

            for list in biglist:
                colors = ["tab:blue" for patch in list]
                patchcollection = Poly3DCollection(list, linewidth=0, edgecolor="k", facecolor=colors, rasterized=True,
                                                   zorder=10)
                self.ax.add_collection3d(patchcollection)
                list_patchcollection.append(patchcollection)
            list_list_patches.append(list_patchcollection)

        return list_list_patches







def fctx(i):
    return 30
def fcty(i):
    return -87+20*i


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
