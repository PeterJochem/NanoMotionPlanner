import numpy as np
from typing import Optional


def normal_to_points_in_plane(points: np.ndarray) -> np.ndarray:
    """Computes the normal vector to the plane in which the three provided points lie.

    Args:
        points: numpy.ndarray
            3x3 vector of points.

    Returns:
        numpy.ndarray:
            3x1 vector. The normal to the plane in which the points lie.
    """
    point_1, point_2, point_3 = points
    return np.cross((point_3 - point_1), (point_3 - point_2))


def approximately_equal(a: float, b: float, epsilon: float = 1e-8) -> bool:
    """Checks if a and b are approximately equal.

    Args:
        a: float
        b: float
        epsilon: float

    Returns:
        bool:
            True iff the absolute difference between a and b is less than epsilon.
    """
    return abs(a - b) < epsilon


def same_sign(a: float, b: float) -> bool:
    """Checks if the two numbers have the same sign.

    Returns:
        bool:
            True iff both numbers are positive or both numbers are negative.
    """
    return (a > 0. and b > 0.) or (a < 0. and b < 0.)


def intersection_of_a_line_and_a_plane(point_1: np.ndarray,
                                       point_2: np.ndarray,
                                       plane: "PlaneEquation") -> Optional[float]:
    """Computes the intersection of a line formed by two points and a plane in terms of the location on the line.

    Notes:
        Let the line be parameterized by a point and a direction.
        Then all the points on the line have a unique distance from the point.
        Line(T) = Point + (Direction * T)
        See the pdf in this directory for a derivation.
        See the image in this directory for an image from the Wolfram Alpha equation solver.
        Useful video: https://www.youtube.com/watch?v=_W3aVWsMp14

    Args:
        point_1: numpy.ndarray
        point_2: numpy.ndarray
        plane: PlaneEquation

    Returns:
        Optional[float]:
            Parameter T above. Line(T) = Point + (Direction * T)
    """

    numerator = plane.d + np.dot(point_1, plane.n)
    denominator = np.dot(point_2 - point_1, plane.n)

    if approximately_equal(denominator, 0.):
        return None

    return numerator / denominator
