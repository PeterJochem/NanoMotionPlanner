from typing import List
import numpy as np
from geometry.primitives.transformation import Transformation


def multiply(transformations: List[Transformation]):
    """Multiplies or composes all the Transformations in the given list.

    Args:
        transformations: List[Transformation]

    Returns:
        Transformation:
            The product of all the transformations.
    """
    left = Transformation.identity()
    for transformation in transformations:
        left = left * transformation
    return left


def pose(transformation: Transformation) -> np.ndarray:
    """Extracts the pose ([x, y, z, roll, pitch, yaw] from the provided transformation.

    Args:
        transformation: Transformation
            Extract the pose from this transformation.

    Returns:
        numpy.ndarray:
            [x, y, z, roll, pitch, yaw]
    """

    x = transformation.matrix[0][3]
    y = transformation.matrix[1][3]
    z = transformation.matrix[2][3]

    roll, pitch, yaw = transformation.euler_angles()

    return np.array([x, y, z, roll, pitch, yaw])
