import matplotlib as mpl
import numpy as np
from matplotlib.path import Path


def create_origin_patch(origin: np.ndarray, scale=0.1):
    vertices = origin + scale * np.array(
        [(-0.5, -0.5), (0.5, 0.5), (0, 0), (0.5, -0.5), (-0.5, 0.5)]
    )
    codes = [Path.MOVETO, Path.LINETO, Path.MOVETO, Path.LINETO, Path.LINETO]
    path = Path(vertices, codes)
    return mpl.patches.PathPatch(path, facecolor="none", edgecolor="black", lw=1, zorder=200)
