import numpy as np


class TriangleTriangleCollisionDetector:
    """
    Detects collisions between two triangles in 3D space.
    """
    def __init__(self, triangle_1: np.ndarray, triangle_2: np.ndarray):
        """Constructor.

        Args:
            triangle_1: numpy.ndarray
                3 point in 3D space.
            triangle_2: numpy.ndarray
                3 points in 3D space.
        """
        self.triangle_1 = triangle_1
        self.triangle_2 = triangle_2

    def detect(self) -> bool:
        """Detects collisions between two triangles in 3D space.

        Returns:
            bool:
                True iff there is a collision between the two triangles in 3D space.
        """
        raise RuntimeError("Need to implement this!")
