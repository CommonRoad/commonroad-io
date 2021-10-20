import math

import numpy as np
import numpy.linalg
from shapely.geometry import LineString


def compute_polyline_lengths(polyline: np.ndarray) -> np.ndarray:
    """
    Computes the path lengths of a given polyline in steps travelled
    from initial to final coordinate.

    :param polyline: Polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    :return: Path lengths of the polyline for each coordinate in m
    """
    assert_valid_polyline(polyline, 2)

    distance = [0]
    for i in range(1, len(polyline)):
        distance.append(distance[i - 1] + np.linalg.norm(polyline[i] - polyline[i - 1]))

    return np.array(distance)


def compute_polyline_complete_length(polyline: np.ndarray) -> float:
    """
    Computes the complete path length of a given polyline.

    :param polyline: Polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    :return: Path length of the polyline in m
    """
    lengths = compute_polyline_lengths(polyline)

    return float(lengths[-1])


def compute_polyline_curvatures(polyline: np.ndarray) -> np.ndarray:
    """
    Computes the curvatures along a given polyline travelled from initial
    to final coordinate.

    :param polyline: Polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
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
    assigned with the computed orientation of the penultimate one.

    :param polyline: Polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    :return: Orientations of the polyline for each coordinate in rad
    """
    assert_valid_polyline(polyline, 2)

    orientation = []
    for i in range(0, len(polyline) - 1):
        pt_1 = polyline[i]
        pt_2 = polyline[i + 1]
        tmp = pt_2 - pt_1
        orient = np.arctan2(tmp[1], tmp[0])
        orientation.append(orient)
        if i == len(polyline) - 2:
            orientation.append(orient)

    return np.array(orientation)


def compute_polyline_initial_orientation(polyline: np.ndarray) -> float:
    """
    Computes the orientation of the initial coordinate with respect to the succeeding
    coordinate.

    :param polyline: Polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    :return: Orientation of the initial coordinate in rad
    """
    orientations = compute_polyline_orientations(polyline)

    return orientations[0]


def is_point_on_polyline(polyline: np.ndarray, point: np.ndarray) -> bool:
    """
    Computes whether a given point lies on a polyline. That means, the point is between the starting
    and ending point of the polyline.

    :param polyline: Polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    :param point: 2D point [x, y]
    :return: Lies on polyline or not
    """
    assert_valid_polyline(polyline, 2)

    for i in range(0, len(polyline) - 1):
        l_x_1 = polyline[i][0]
        l_y_1 = polyline[i][1]
        l_x_2 = polyline[i + 1][0]
        l_y_2 = polyline[i + 1][1]

        cross_product = (point[1] - l_y_1) * (l_x_2 - l_x_1) - (point[0] - l_x_1) * (l_y_2 - l_y_1)

        epsilon = 1e-12
        if abs(cross_product) > epsilon:
            continue

        dot_product = (point[0] - l_x_1) * (l_x_2 - l_x_1) + (point[1] - l_y_1) * (l_y_2 - l_y_1)
        if dot_product < 0:
            continue

        squared_length = (l_x_2 - l_x_1) * (l_x_2 - l_x_1) + (l_y_2 - l_y_1) * (l_y_2 - l_y_1)
        if dot_product > squared_length:
            continue

        return True

    return False


def compute_polyline_intersections(polyline_1: np.ndarray, polyline_2: np.ndarray) -> np.ndarray:
    """
    Computes the intersection points of two polylines.

    :param polyline_1: First polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    :param polyline_2: Second polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    :return: Intersection points
    """
    assert_valid_polyline(polyline_1, 2)
    assert_valid_polyline(polyline_2, 2)

    intersection_points = []

    for i in range(0, len(polyline_1) - 1):
        for j in range(0, len(polyline_2) - 1):

            x_diff = [polyline_1[i][0] - polyline_1[i + 1][0], polyline_2[j][0] - polyline_2[j + 1][0]]
            y_diff = [polyline_1[i][1] - polyline_1[i + 1][1], polyline_2[j][1] - polyline_2[j + 1][1]]

            div = np.linalg.det(np.array([x_diff, y_diff]))
            if math.isclose(div, 0.0):
                continue

            d_1 = np.linalg.det(
                np.array([[polyline_1[i][0], polyline_1[i][1]], [polyline_1[i + 1][0], polyline_1[i + 1][1]]]))
            d_2 = np.linalg.det(
                np.array([[polyline_2[j][0], polyline_2[j][1]], [polyline_2[j + 1][0], polyline_2[j + 1][1]]]))
            d = [d_1, d_2]
            x = np.linalg.det(np.array([d, x_diff])) / div
            y = np.linalg.det(np.array([d, y_diff])) / div

            point = np.array([x, y])
            between_polyline_1 = is_point_on_polyline(np.array([polyline_1[i], polyline_1[i + 1]]), point)
            between_polyline_2 = is_point_on_polyline(np.array([polyline_2[j], polyline_2[j + 1]]), point)
            if [x, y] not in intersection_points and between_polyline_1 and between_polyline_2:
                intersection_points.append([x, y])

    return np.array(intersection_points)


def is_polyline_self_intersection(polyline: np.ndarray) -> bool:
    """
    Computes whether the given polyline contains self-intersection. Intersection
    at boundary points are considered as self-intersection.

    :param polyline: Polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    :return: Self-intersection or not
    """
    assert_valid_polyline(polyline, 2)

    line = [(x, y) for x, y in polyline]
    line_string = LineString(line)

    return not line_string.is_simple


def compare_polylines_equality(polyline_1: np.ndarray, polyline_2: np.ndarray, threshold=1e-10) -> bool:
    """
    Compares two polylines for equality. For equality of the values a threshold can be given.

    :param polyline_1: First polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    :param polyline_2: Second polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    :param threshold: Threshold for equality of values
    :return: Equality of both polylines or not
    """
    assert_valid_polyline(polyline_1, 2)
    assert_valid_polyline(polyline_2, 2)

    return np.allclose(polyline_1, polyline_2, rtol=threshold, atol=threshold)


def resample_polyline_with_number(polyline: np.ndarray, number: int) -> np.ndarray:
    """
    Resamples the given polyline with a fixed number of points. The number
    of coordinates can be resampled down or up.

    :param polyline: Polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    :param number: Fixed number of 2D points
    :return: Resampled polyline
    """
    assert_valid_polyline(polyline, 2)
    assert number > 1, 'Number n={} has to be at least two'.format(number)

    line = LineString(polyline)
    dists = np.linspace(0, line.length, number)
    polyline_resampled = []
    for d in dists:
        point = line.interpolate(d)
        polyline_resampled.append([point.x, point.y])

    return np.array(polyline_resampled)


def resample_polyline_with_distance(polyline: np.ndarray, distance: float) -> np.ndarray:
    """
    Resamples the given polyline with a specific distance. For a higher distance than the length
    of the given polyline the polyline is not resampled.

    :param polyline: Polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    :param distance: Specific distance
    :return: Resampled polyline
    """
    assert_valid_polyline(polyline, 2)
    assert distance > 0, 'Distance d={} has to be greater than 0'.format(distance)

    line = LineString(polyline)
    dists = np.arange(0, line.length, distance)
    polyline_resampled = []
    for d in dists:
        point = line.interpolate(d)
        polyline_resampled.append([point.x, point.y])

    last_point = polyline[-1]
    polyline_resampled.append(last_point)

    return np.array(polyline_resampled)


def assert_valid_polyline(polyline: np.ndarray, min_size=2) -> None:
    """
    Makes assertions for a valid polyline. A valid polyline is instanced from the type np.ndarray,
    is constructed of at least a specified number of coordinates, and is two-dimensional.

    :param: Polyline with 2D points [[x_0, y_0], [x_1, y_1], ...]
    """
    assert polyline is not None and isinstance(polyline, np.ndarray), \
        'Polyline p={} is not instanced from np.ndarray'.format(polyline)
    assert len(polyline) > min_size - 1, 'Polyline p={} is not constructed of at least two coordinates'.format(polyline)
    for i in range(0, len(polyline)):
        assert polyline.ndim == 2 and len(polyline[i, :]) == 2, 'Polyline p={} is not two-dimensional'.format(polyline)
