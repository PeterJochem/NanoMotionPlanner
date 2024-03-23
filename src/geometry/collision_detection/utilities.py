import numpy as np


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
    np.cross((point_3 - point_1), (point_3 - point_2))


def approximately_equal(a: float, b: float, epsilon: float = 1e-8) -> bool:
    """Checks if a and b are approximately equal.

    Args:
        a: float
        b: float
        epsilon: float

    Returns:
        True iff the absolute difference between a and b is less than epsilon.
    """
    return abs(a - b) < epsilon
