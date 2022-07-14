import math

import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt




class obstacles:

    def __init__(self,ax ,scenario,information,car_time):

        self.ax = ax
        self.car_time=car_time
        self.scenario = scenario
        self.information =information
        self.list_car = []
        self.nb_car=0
        self.listz=[]

        self.before=[0,0,0]
        for i in range(self.scenario.static_obstacles.__len__()):
            self.get_static(i)

        for i in range(len(scenario.dynamic_obstacles)):
            self.list_car.append(self.get_dynamic(i))
            self.nb_car+=1




    def init_show(self):

        for j in range(self.nb_car):
            for i in range(self.scenario.dynamic_obstacles[j].prediction.final_time_step):
                for y in range(len(self.list_car[j][i][:])):
                    self.list_car[j][i][y].set_alpha(0)
                    self.list_car[j][i][y].set_linewidth(0)


    def show(self):
        i = 0
        self.init_show()

        while True:
            i += 1
            self.init_show()

            for k in range(self.nb_car):
                for j in range(self.scenario.dynamic_obstacles[k].prediction.final_time_step):
                    if i == j:
                        for y in range(len(self.list_car[k][i][:])):
                            self.list_car[k][i][y].set_alpha(1)
                            self.list_car[k][i][y].set_linewidth(0.1)

            plt.pause(0.01)

    def get_dynamic(self, i: int) -> list:

        list_list_patches = []
        for t in range(self.scenario.dynamic_obstacles[i].prediction.final_time_step):

            shape = self.scenario.dynamic_obstacles[i].occupancy_at_time(t).shape
            cur_lan_id=self.car_time[i][t]

            cur_lan =self.scenario.lanelet_network.find_lanelet_by_id(cur_lan_id)
            point=self.choose_point(cur_lan.polygon.vertices,shape.center)
            bottom=self.choose_z(cur_lan_id,point)


            #print("car : "+ist+" at time : "+tst+" z = "+bst+" and lan : "+lst)
            length = shape.length

            top = bottom + 2
            width = shape.width
            biglist = []

            o = shape.orientation

            oy=0
            if t!=0:
                oy=(self.before[2]-bottom)/math.pow(math.pow(self.before[1]-shape.center[0],2)+math.pow(self.before[1]-shape.center[0],2),0.5)
                oy=math.atan(oy)*20# I can't explain the  *20 for the moment



            self.before[0] = shape.center[0]
            self.before[2] = shape.center[1]
            self.before[2] = bottom


            """
            cube
            
            
               8########7
              #       # #
             #       #  #
            5########6  #
            #        #  #
            #  4########3
            # #      # #
            ##       ##
            1########2
            """
            #############################################################################################################
            list = [  # underneath
                (- length * 0.5, - width * 0.5, 0), (- length * 0.5, + width * 0.5, 0),
                (+ length * 0.5, + width * 0.5, 0), (+ length * 0.5, - width * 0.5, 0)]
            z = [bottom, bottom, bottom, bottom]
            list = rotation_x(oy, rotation_z(o, list))
            list = add_center(shape.center[0], shape.center[1], z, list)
            v1 = list[0][0]
            v2 = list[0][1]
            v3 = list[0][2]
            v4 = list[0][3]


            trans=plus(vector(minus(v2,v3),minus(v1,v2)),vector(minus(v2,v3),minus(v1,v2)))#hight=2
            v5 = plus(v1, trans)
            v6 = plus(v2, trans)
            v7 = plus(v3, trans)
            v8 = plus(v4, trans)
            biglist.append([(v1, v2, v3, v4)])
            biglist.append([(v1, v2, v6, v5)])
            biglist.append([(v2, v3, v7, v6)])
            biglist.append([(v3, v4, v8, v7)])
            biglist.append([(v4, v1, v5, v8)])
            biglist.append([(v5, v6, v7, v8)])


            colors = ["tab:blue" for patch in list]
            list_patchcollection = []
            for list in biglist:
                patchcollection = Poly3DCollection(list, linewidth=0, edgecolor="k", facecolor=colors, rasterized=True,
                                                   zorder=10)
                self.ax.add_collection3d(patchcollection)
                list_patchcollection.append(patchcollection)
            list_list_patches.append(list_patchcollection)

        return list_list_patches


    def get_static(self,i):

        if True:
            shape = self.scenario.static_obstacles[i].occupancy_at_time(0).shape
            bottom = 0
            length = shape.length
            top = bottom + 2
            width = shape.width
            biglist = []

            o = shape.orientation
            oy=0
            #############################################################################################################

            list = [  # underneath
                (- length * 0.5, - width * 0.5, 0), (- length * 0.5, + width * 0.5, 0),
                (+ length * 0.5, + width * 0.5, 0), (+ length * 0.5, - width * 0.5, 0)]
            z = [bottom, bottom, bottom, bottom]

            list = rotation_x(oy, rotation_z(o, list))

            list = add_center(shape.center[0] , shape.center[1] , z, list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # top
                (- length * 0.5, - width * 0.5, 0), (- length * 0.5, + width * 0.5, 0),
                (+ length * 0.5, + width * 0.5, 0), (+ length * 0.5, - width * 0.5, 0)]
            z = [top, top, top, top]
            list = rotation_x(oy, rotation_z(o, list))
            list = add_center(shape.center[0], shape.center[1] , z, list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # front
                (+ length * 0.5, - width * 0.5, 0), (+ length * 0.5, + width * 0.5, 0),
                (+ length * 0.5, + width * 0.5, 0), (+ length * 0.5, - width * 0.5, 0)]
            z = [top, top, bottom, bottom]
            list = rotation_x(oy, rotation_z(o, list))
            list = add_center(shape.center[0] , shape.center[1] , z, list)

            biglist.append(list)
            ########################################################################################################################

            list = [  # bottom
                (- length * 0.5, - width * 0.5, 0), (- length * 0.5, + width * 0.5, 0),
                (- length * 0.5, + width * 0.5, 0), (- length * 0.5, - width * 0.5, 0)]
            z = [top, top, bottom, bottom]
            list = rotation_x(oy, rotation_z(o, list))
            list = add_center(shape.center[0] , shape.center[1] , z, list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # right
                (- length * 0.5, + width * 0.5, 0), (- length * 0.5, + width * 0.5, 0),
                (+ length * 0.5, + width * 0.5, 0), (+ length * 0.5, + width * 0.5, 0)]
            z = [bottom, top, top, bottom]

            list = rotation_x(oy, rotation_z(o, list))
            list = add_center(shape.center[0] , shape.center[1] , z, list)

            biglist.append(list)

            ########################################################################################################################

            list = [  # left
                (- length * 0.5, - width * 0.5, 0), (- length * 0.5, - width * 0.5, 0),
                (+ length * 0.5, - width * 0.5, 0), (+ length * 0.5, - width * 0.5, 0)]
            z = [bottom, top, top, bottom]
            list = rotation_x(oy, rotation_z(o, list))
            list = add_center(shape.center[0] , shape.center[1] , z, list)

            biglist.append(list)

            colors = ["tab:blue" for patch in list]
            list_patchcollection = []
            for list in biglist:
                patchcollection = Poly3DCollection(list, linewidth=0, edgecolor="k", facecolor=colors, rasterized=True,
                                                   zorder=10)
                self.ax.add_collection3d(patchcollection)


    def choose_point(self,lan,center):
        result=0
        for i in range (int(len(lan))):
            if math.pow(math.pow(lan[i][0]-center[0],2)+math.pow(lan[i][1]-center[1],2),0.5)<math.pow(math.pow(lan[result][0]-center[0],2)+math.pow(lan[result][1]-center[1],2),0.5):
                result=i
        return result

    def choose_z(self,lan_id,i):
        for j in range(len(self.information)):
            if self.information[j][0]==lan_id:
                if self.information[j][1]==i:
                    return self.information[j][2]



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

def add_center(x: float, y: float,z ,list: list):
    list_tempo = []
    list_ret = []
    for i in range(len(list)):
        list_tempo.append((list[i][0] + x, list[i][1] + y, list[i][2]+z[i] ))
        list_ret.append(list_tempo)
    return list_ret

#product = (u2v3-u3v2)(u3v1-u1v3)(u1v2-u2v1)

def minus(u:tuple,v:tuple):
    return (v[0]-u[0],v[1]-u[1],v[2]-u[2])

def vector(u:tuple,v:tuple):
    n=math.pow(math.pow(u[1]*v[2]-u[2]*v[1],2)+math.pow(u[2]*v[0]-u[0]*v[2],2)+math.pow(u[0]*v[1]-u[1]*v[0],2),0.5)
    return ((u[1]*v[2]-u[2]*v[1])/n,(u[2]*v[0]-u[0]*v[2])/n,(u[0]*v[1]-u[1]*v[0])/n)

def plus(u:tuple,v:tuple):
    return (v[0]+u[0],v[1]+u[1],v[2]+u[2])

