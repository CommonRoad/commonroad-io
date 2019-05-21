from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib as mpl
import matplotlib.collections as collections
from typing import List

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = [""]
__version__ = "2019.1"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad@in.tum.de"
__status__ = "Released"


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