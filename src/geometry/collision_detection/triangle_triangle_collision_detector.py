import numpy as np
from geometry.collision_detection.utilities import normal_to_points_in_plane, approximately_equal


class PlaneEquation:
    """
    ...
    Plane: (n * X) + d = 0, where x is a 3 vector.
    """

    def __init__(self, points: np.ndarray):
        """Constructor.

        Args:
            points: numpy.ndarray
                Represents a triangle. 3 points in 3D space.
        """
        self.n = normal_to_points_in_plane(points)
        self.d = np.dot(-1 * self.n, points[0])

    def signed_distance(self, point: np.ndarray) -> float:
        """Computes the signed distance from the provided point in 3D space to the plane.

        Args:
            point: numpy.ndarray
                Measures the distance from this point to the plane.

        Returns:
            float:
                The signed distance from the point to the plane.
        """
        return np.dot(self.n, point) + self.d


    def all_vertices_lie_on_one_side_of_plane(self, vertices: np.ndarray) -> bool:
        """..."""
        distances = self.signed_distances(vertices)
        all_zero = all([approximately_equal(distance, 0.) for distance in distances])
        all_positive = all([distance > 0. for distance in distances])
        all_negative = all([distance < 0. for distance in distances])

        if all_zero:
            return False # All the points lie in the plane.
        elif all_positive or all_negative:
            return True
        return False

    def signed_distances(self, points: np.ndarray) -> np.ndarray:
        """..."""

        n = len(points)
        distances = np.zeros(n)
        for i, point in enumerate(points):
            distances[i] = self.signed_distance(point)
        return distances


class TriangleTriangleCollisionDetector:
    """
    Detects collisions between two triangles in 3D space.
    Implements the algorithm detailed here:
        https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tritri_tam.pdf
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

        self.triangle_1_plane_equation = PlaneEquation(triangle_1)
        self.triangle_2_plane_equation = PlaneEquation(triangle_2)

    def detect(self) -> bool:
        """Detects collisions between two triangles in 3D space.

        Returns:
            bool:
                True iff there is a collision between the two triangles in 3D space.
        """
        if self.triangle_1_plane_equation.all_vertices_lie_on_one_side_of_plane(self.triangle_2):
            return False
