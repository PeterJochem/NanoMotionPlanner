from typing import List
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
