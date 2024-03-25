import numpy as np
from geometry.collision_detection.utilities import approximately_equal, normal_to_points_in_plane


class PlaneEquation:
    """
    Holds the information about a plane.
        Plane: (n * X) + d = 0, where x is a 3 vector.
    """

    def __init__(self, points: np.ndarray):
        """Constructor.

        Args:
            points: numpy.ndarray
                Represents a triangle. 3 points in 3D space.
        """
        self.n = normal_to_points_in_plane(points)
        self.n = self.n / np.linalg.norm(self.n)
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
        """Checks if all the vertices lie on one side of the plane.

        Args:
            vertices: numpy.ndarray
                Set of points in 3D space.

        Returns:
            bool:
                True iff all the vertices lie on one side of the plane.
        """
        distances = self.signed_distances(vertices)
        all_zero = all([approximately_equal(distance, 0.) for distance in distances])
        all_positive = all([distance > 0. for distance in distances])
        all_negative = all([distance < 0. for distance in distances])

        if all_zero:
            return False  # All the points lie in the plane.
        elif all_positive or all_negative:
            return True
        return False

    def signed_distances(self, points: np.ndarray) -> np.ndarray:
        """Computes the signed distance from each point to the plane.

        Args:
            points: numpy.ndarray
                A set of points in 3D space.

        Returns:
            numpy.ndarray:
                The signed distance from each point to the plane.
        """

        n = len(points)
        distances = np.zeros(n)
        for i, point in enumerate(points):
            distances[i] = self.signed_distance(point)
        return distances
