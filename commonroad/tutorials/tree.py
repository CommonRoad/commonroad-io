import matplotlib.pyplot as plt
from commonroad.scenario.scenario import Scenario
import matplotlib.pyplot as plt
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from commonroad.scenario.scenario import Scenario
import numpy as np
from drone import drone3d
from fonction import *
import math


class Tree():
    """
    tree class to add diversity and barriers to the environment
    """

    def __init__(self, ax, x: int, y: int, z: int, r: float, wr: float, list_obstacle: list):
        pi = math.pi
        """

        Constructor of a tree object
    
        :param x: x position of the tree 
        :param y: y position of the tree 
        :param z: height of the arch
        :param r: radius of the leaves
        :param wr: radius of the trunk
        :param accurate: degree of accuracy
        :param ax: the figure where the drone should be constructed
        :param list_obstacle: storage list of obstacles
    
        """

        self._x = x
        self._y = y
        self._z = z
        self._r = r
        self._wr = wr
        self._ax = ax
        self.list_obstacle = list_obstacle
        accurate = 50

        self.list = []
        self.list2 = []
        self.patches = []
        self.patches2 = []

        for i in range(accurate):
            for j in range(accurate):
                phi = -pi / 2 + pi * i / accurate
                theta = -pi + 2 * pi * j / accurate

                self.list.append((r * np.sin(theta) * np.cos(phi) + x, r * np.sin(theta) * np.sin(phi) + y,
                                  r * np.cos(theta) + z))
                self.list_obstacle.append([(r * np.sin(theta) * np.cos(phi) + x, r * np.sin(theta) * np.sin(phi) + y,
                                            r * np.cos(theta) + z)])
                self.list2.append((wr * np.cos(theta) + x, wr * np.sin(theta) + y, i * z / accurate))
                self.list_obstacle.append([(wr * np.cos(theta) + x, wr * np.sin(theta) + y, i * z / accurate)])

        self.patches.append(self.list)
        self.patches2.append(self.list2)

        colors = "tab:green"
        patchcollection = Poly3DCollection(self.patches, edgecolor="g", facecolor=colors, rasterized=True)
        self._ax.add_collection3d(patchcollection)

        patchcollection = Poly3DCollection(self.patches2, edgecolor="g", facecolor=colors, rasterized=True)
        self._ax.add_collection3d(patchcollection)

    def __str__(self):
        traffic_str = "\n"
        traffic_str += "Tree:\n"
        traffic_str += "- x position of the tree : {}\n".format(self._x)
        traffic_str += "- y position of the tree : {}\n".format(self._y)
        traffic_str += "- index: {}\n".format(self.i)
        traffic_str += "- radius: {}\n".format(self._radius)
        return traffic_str

    def __eq__(self, other):
        if  self._r == other.r and self._x == other.x and self._y == other.y and self._z == other.z and self._wr == other.wr :
            return True
        return False


    @property
    def ax(self) :
        return self._ax

    @ax.setter
    def ax(self, ax):
        self._ax = ax

    @property
    def x(self) :
        return self._x

    @x.setter
    def x(self, x):
        self._x = x

    @property
    def y(self) :
        return self._y

    @y.setter
    def y(self, y):
        self._y = y

    @property
    def z(self) :
        return self._z

    @z.setter
    def z(self, z):
        self._z = z

    @property
    def wr(self) :
        return self._wr

    @wr.setter
    def wr(self, wr):
        self._wr = wr


