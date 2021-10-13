import numpy as np
from shapely.geometry import LineString


def compute_polyline_lengths(polyline: np.ndarray) -> np.ndarray:
    """
    Computes the path lengths of a given polyline in steps travelled
    from initial to final coordinate.

    :param polyline: Polyline with 2D points
    :return: Path lengths of the polyline for each coordinate
    """
    assert_valid_polyline(polyline, 2)

    distance = [0]
    for i in range(1, len(polyline)):
        distance.append(distance[i - 1] + np.linalg.norm(polyline[i] - polyline[i - 1]))

    return np.array(distance)


def compute_polyline_length(polyline: np.ndarray) -> float:
    """
    Computes the complete path length of a given polyline.

    :param polyline: Polyline with 2D points
    :return: Path length of the polyline
    """
    lengths = compute_polyline_lengths(polyline)

    return float(lengths[-1])


def compute_polyline_curvatures(polyline: np.ndarray) -> np.ndarray:
    """
    Computes the curvatures along a given polyline travelled from initial
    to final coordinate.

    :param polyline: Polyline with 2D points
    :return: Curvatures of the polyline for each coordinate
    """
    assert_valid_polyline(polyline, 3)

    x_d = np.gradient(polyline[:, 0])
    x_dd = np.gradient(x_d)
    y_d = np.gradient(polyline[:, 1])
    y_dd = np.gradient(y_d)

    return (x_d * y_dd - x_dd * y_d) / ((x_d ** 2 + y_d ** 2) ** (3. / 2.))


def compute_polyline_orientations(polyline: np.ndarray) -> np.ndarray:
    """
    Computes the orientation of a given polyline travelled from initial
    to final coordinate. The orientation of the last coordinate is always
    assigned with the computed orientation of the penultimate one. The
    orientation is given by degree.

    :param polyline: Polyline with 2D points
    :return: Orientations of the polyline for each coordinate
    """
    assert_valid_polyline(polyline, 2)

    orientation = []
    for i in range(0, len(polyline) - 1):
        pt_1 = polyline[i]
        pt_2 = polyline[i + 1]
        tmp = pt_2 - pt_1
        orient = np.arctan2(tmp[1], tmp[0]) * 180 / np.pi
        orientation.append(orient)
        if i == len(polyline) - 2:
            orientation.append(orient)

    return np.array(orientation)


def compute_polyline_orientation(polyline: np.array) -> float:
    """
    Computes the orientation of the initial coordinate with respect to the succeeding
    coordinate. The orientation is given by degree.

    :param polyline: Polyline with 2D points
    :return: Orientation of the initial coordinate
    """
    orientations = compute_polyline_orientations(polyline)

    return orientations[0]


def compute_polyline_self_intersection(polyline: np.array) -> bool:
    """
    Computes whether the given polyline contains self-intersection. Intersection
    at boundary points are considered as self-intersection.

    :param: Polyline with 2D points
    :return: Self-intersection or not
    """
    assert_valid_polyline(polyline, 2)

    line = [(x, y) for x, y in polyline]
    line_string = LineString(line)

    return not line_string.is_simple


def assert_valid_polyline(polyline: np.array, min_size=2) -> None:
    """
    Makes assertions for a valid polyline. A valid polyline is instanced from the type np.ndarray,
    is constructed of at least a specified number of coordinates, and is two-dimensional.

    :param: Polyline with 2D points
    """
    assert isinstance(polyline, np.ndarray), 'Polyline p={} is not instanced from np.ndarray'.format(polyline)
    assert len(polyline) > min_size - 1, 'Polyline p={} is not constructed of at least two coordinates'.format(polyline)
    for i in range(0, len(polyline)):
        assert polyline.ndim == 2 and len(polyline[i, :]) == 2, 'Polyline p={} is not two-dimensional'.format(polyline)
