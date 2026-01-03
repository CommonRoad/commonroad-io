from typing import TypeVar

import shapely

T = TypeVar("T", bound=shapely.Geometry)


def translate_and_rotate(geom: T, xoff: float, yoff: float, angle: float) -> T:
    """Translates a shapely geometry, and then rotates it around the origin.

    :param geom: shapely geometry to be transformed
    :param xoff: translation offset in x-direction
    :param yoff: translation offset in y-direction
    :param angle: rotation angle in radians (counter-clockwise)
    :return: transformed shapely geometry
    """
    translated_geom = shapely.affinity.translate(geom, xoff=xoff, yoff=yoff)
    rotated_geom = shapely.affinity.rotate(
        translated_geom, angle=angle, origin=(0, 0), use_radians=True
    )
    return rotated_geom


def rotate_and_translate(geom: T, angle: float, xoff: float, yoff: float) -> T:
    """Rotates a shapely geometry around the origin, and translates it.

    :param geom: shapely geometry to be transformed
    :param angle: rotation angle in radians (counter-clockwise)
    :param xoff: translation offset in x-direction
    :param yoff: translation offset in y-direction
    :return: transformed shapely geometry
    """
    rotated_geom = shapely.affinity.rotate(geom, angle=angle, origin=(0, 0), use_radians=True)
    translated_geom = shapely.affinity.translate(rotated_geom, xoff=xoff, yoff=yoff)
    return translated_geom
