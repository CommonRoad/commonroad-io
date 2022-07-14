from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math

from commonroad.scenario.scenario import Scenario

class lanelet:
    def __init__(self, ax, scenario):
        self.ax = ax
        self.scenario = scenario
        self.hight=5
        self.information=[]#id and after z coordinat of thr first point
        self.zi=0
        self.zip1=0




        finish_at = [0 for i in range(self.idmax())]
        check_liste = []
        colors = ["tab:grey"]

        for l in range(len(scenario.lanelet_network.lanelets)):
            lanelet = scenario.lanelet_network.lanelets[l]
            if lanelet.predecessor == []:
                check_liste.append(lanelet.lanelet_id)



        while check_liste != []:
            current = self.get_lan(check_liste[0])


            check_liste.remove(check_liste[0])
            if current.predecessor == []:
                lanelet = current.polygon.vertices
                for i in range(len(current.successor)):
                    check_liste.append(current.successor[i])

                for i in range(int(len(lanelet) / 2)):
                    a = (lanelet[i][0], lanelet[i][1], self.hight * current.layer)
                    b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], self.hight * current.layer)
                    c = (lanelet[i + 1][0], lanelet[i + 1][1], self.hight * current.layer)
                    d = (lanelet[-i][0], lanelet[-i][1], self.hight * current.layer)
                    finish_at.append(self.hight * current.layer)
                    self.information.append((current.lanelet_id,i,self.hight * current.layer))

                    lan = [[b, c, a, d]]

                    lancolect = Poly3DCollection(lan, linewidth=0, edgecolor="k", facecolor=colors, rasterized=False,
                                                 alpha=1, zorder=1)
                    self.ax.add_collection3d(lancolect)

                    a = (lanelet[i][0], lanelet[i][1], self.hight * current.layer-0.1)
                    b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], self.hight * current.layer)
                    c = (lanelet[i + 1][0], lanelet[i + 1][1], self.hight * current.layer-0.1)
                    d = (lanelet[-i][0], lanelet[-i][1], self.hight * current.layer)
                    finish_at.append(self.hight * current.layer)
                    lan = [[b, c, a, d]]

                    lancolect = Poly3DCollection(lan, linewidth=0, edgecolor="k", facecolor=colors, rasterized=False,
                                                 alpha=1, zorder=1)
                    self.ax.add_collection3d(lancolect)

                finish_at[current.lanelet_id] = (self.hight * current.layer)

            if current.predecessor != []:
                layer=current.layer*self.hight
                lanelet = current.polygon.vertices
                for i in range(len(current.successor)):
                    check_liste.append(current.successor[i])

                begin = finish_at[current.predecessor[0]]
                scale = self.get_scale(begin, current)

                for i in range(int(len(lanelet) / 2)):
                    self.zi=begin + scale * i
                    self.zip1=begin + scale * (i + 1)
                    self.zi=self.modif(self.zi,layer,scale)
                    self.zip1=self.modif(self.zip1,layer,scale)

                    a = (lanelet[i][0], lanelet[i][1], self.zi)
                    b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], self.zip1)
                    c = (lanelet[i + 1][0], lanelet[i + 1][1], self.zip1)
                    d = (lanelet[-i][0], lanelet[-i][1], self.zi)
                    lan = [[b, c, a, d]]
                    self.information.append((current.lanelet_id, i, self.zi))

                    lancolect = Poly3DCollection(lan, linewidth=0, edgecolor="k", facecolor=colors,
                                                     rasterized=False, alpha=1, zorder=1)
                    self.ax.add_collection3d(lancolect)


                finish_at[current.lanelet_id] = self.zip1
               
            """
            if current.predecessor != []:

                lanelet = current.polygon.vertices
                for i in range(len(current.successor)):
                    check_liste.append(current.successor[i])


                begin = finish_at[current.predecessor[0]]
                scale = self.get_scale(begin, current)

                for i in range(int(len(lanelet) / 2)):
                    a = (lanelet[i][0], lanelet[i][1], self.hight * current.layer)
                    b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], self.hight * current.layer)
                    c = (lanelet[i + 1][0], lanelet[i + 1][1], self.hight * current.layer)
                    d = (lanelet[-i][0], lanelet[-i][1], self.hight * current.layer)
                    lan = [[b, c, a, d]]
                    self.information.append(current.lanelet_id)
                    self.information.append(self.hight * current.layer)


                    lancolect = Poly3DCollection(lan, linewidth=0, edgecolor="k", facecolor=colors,
                                                     rasterized=False, alpha=1, zorder=1)
                    self.ax.add_collection3d(lancolect)

                for i in range(int(len(lanelet) / 2)):
                    a = (lanelet[i][0], lanelet[i][1], self.hight * current.layer -0.1)
                    b = (lanelet[-(i + 1)][0], lanelet[-(i + 1)][1], self.hight * current.layer)
                    c = (lanelet[i + 1][0], lanelet[i + 1][1], self.hight * current.layer-0.1)
                    d = (lanelet[-i][0], lanelet[-i][1],self.hight * current.layer)
                    lan = [[b, c, a, d]]

                    lancolect = Poly3DCollection(lan, linewidth=0, edgecolor="k", facecolor=colors,
                                                     rasterized=False, alpha=1, zorder=1)
                    self.ax.add_collection3d(lancolect)
                finish_at[current.lanelet_id] = (begin + scale * int(len(lanelet) / 2))

    """
    def get_lan(self, id):
        for lan in self.scenario.lanelet_network.lanelets:
            if lan.lanelet_id == id:
                return lan

    def get_scale(self, begin, curent):
        scalemax = 0.5
        scalemin = 0.0
        best_scale=(curent.layer * self.hight - begin / int((len(curent.polygon.vertices)/2)))


        if curent.layer * self.hight - begin == 0:
            return 0

        if begin + best_scale * int(len(curent.polygon.vertices) / 2)!=curent.layer*self.hight:
            return 0.3 * (curent.layer * self.hight - begin) / abs(curent.layer * self.hight - begin)
        if best_scale.__abs__() < scalemax and best_scale.__abs__() > scalemin:
            return best_scale


        return 0.1 * (curent.layer * self.hight - begin) / abs(curent.layer * self.hight - begin)

    def size_of_lanelet(self, current):
        end = int(len(current.polygon.vertices) / 2)
        return math.pow(math.pow(current.polygon.vertices[end][0] - current.polygon.vertices[0][0], 2) + math.pow(current.polygon.vertices[end][1] - current.polygon.vertices[0][1], 2), 0.5)

    def idmax(self):
        idmax = 0
        for lan in self.scenario.lanelet_network.lanelets:
            if lan.lanelet_id > idmax:
                idmax = lan.lanelet_id
        return idmax

    def modif(self,z,layer,scale):

        if z < layer and scale < 0 :
            return layer
        if z > layer and scale > 0 :
            return layer
        return z

