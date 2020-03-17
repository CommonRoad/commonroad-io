from matplotlib.lines import Line2D
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib as mpl
import matplotlib.collections as collections
from typing import List

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = [""]
__version__ = "2020.1"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


class LineDataUnits(Line2D):
    def __init__(self, *args, **kwargs):
        _lw_data = kwargs.pop("linewidth", 1)
        # _dashes_data = kwargs.pop("dashes", 1)
        super().__init__(*args, **kwargs)
        self._lw_data = _lw_data
        # self._dashes_data = _dashes_data

    def _get_lw(self):
        if self.axes is not None:
            ppd = 72./self.axes.figure.dpi
            trans = self.axes.transData.transform
            return ((trans((1, self._lw_data))-trans((0, 0)))*ppd)[1]
        else:
            return 1

    def _set_lw(self, lw):
        self._lw_data = lw

    # def _get_dashes(self):
    #     if self.axes is not None:
    #         ppd = 72. / self.axes.figure.dpi
    #         trans = self.axes.transData.transform
    #         if not None in self._dashes_data:
    #             scale0 = ((trans((1, self._dashes_data[0])) - trans((0, 0))) * ppd)[1]
    #             scale1 = ((trans((1, self._dashes_data[1])) - trans((0, 0))) * ppd)[1]
    #             return (scale0, scale1)
    #         else:
    #             return self._dashes_data
    #     else:
    #         return (1,1)
    #
    # def _set_dashes(self, dashes):
    #     self._dashes_data = dashes


    _linewidth = property(_get_lw, _set_lw)
    # _dashes = property(_get_dashes, _set_dashes)


def draw_polygon_as_patch(vertices, ax, zorder=5, facecolor='#ffffff',
                          edgecolor='#000000', lw=0.5, alpha=1.0) -> mpl.patches.Patch:
    """
    vertices are no closed polygon (first element != last element)
    """
    verts = []
    codes = [Path.MOVETO]
    for p in vertices:
        verts.append(p)
        codes.append(Path.LINETO)
    del codes[-1]
    codes.append(Path.CLOSEPOLY)
    verts.append((0, 0))

    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=facecolor, edgecolor=edgecolor,
                              lw=lw, zorder=zorder, alpha=alpha)
    ax.add_patch(patch)

    return patch


def draw_polygon_collection_as_patch(vertices: List[list], ax, zorder=5, facecolor='#ffffff',
                          edgecolor='#000000', lw=0.5, alpha=1.0, antialiased=True) -> mpl.collections.Collection:
    """
    vertices are no closed polygon (first element != last element)
    """
    path_list = list()
    for v in vertices:
        verts = []
        codes = [Path.MOVETO]
        for p in v:
            verts.append(p)
            codes.append(Path.LINETO)
        del codes[-1]
        codes.append(Path.CLOSEPOLY)
        verts.append((0, 0))

        path_list.append(Path(verts, codes))
        collection_tmp = collections.PathCollection(path_list, facecolor=facecolor, edgecolor=edgecolor,
                                  lw=lw, zorder=zorder, alpha=alpha, antialiaseds=antialiased)
        collection = ax.add_collection(collection_tmp)

    return collection