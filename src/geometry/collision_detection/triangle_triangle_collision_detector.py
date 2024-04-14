import numpy as np
from geometry.collision_detection.utilities import same_sign, intersection_of_a_line_and_a_plane
from geometry.primitives.interval import Interval
from geometry.primitives.plane_equation import PlaneEquation
from numerical_algorithms.utilities import normalized_cosine_similarity


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

    def triangles_lie_in_the_same_plane(self, epsilon: float = 0.05) -> bool:
        """Checks if the two triangles lie in the same plane.

        Args:
            epsilon: float:
                Threshold for how close the two planes are to parallel.

        Returns:
            bool:
                True iff the two triangles lie in the same plane.
        """
        ncs = normalized_cosine_similarity(self.triangle_1_plane_equation.n, self.triangle_2_plane_equation.n)
        return abs(ncs - 1.) < epsilon


    def seperate_points_by_signed_distance(self, points: np.ndarray, plane: PlaneEquation) -> np.ndarray:
        """Separates the points based on which side of the plane they lie on.

        Args:
            points: numpy.ndarray
            plane: PlaneEquation

        Returns:
            numpy.ndarray:
                The first two points are the same side of the plane. The last point is on the other side.
        """

        separated_points = np.zeros((3, 3))

        point_1_distance = plane.signed_distance(points[0])
        point_2_distance = plane.signed_distance(points[1])
        point_3_distance = plane.signed_distance(points[2])

        if same_sign(point_1_distance, point_2_distance):
            separated_points[0] = points[0]
            separated_points[1] = points[1]
            separated_points[2] = points[2]
        elif same_sign(point_1_distance, point_3_distance):
            separated_points[0] = points[0]
            separated_points[1] = points[2]
            separated_points[2] = points[1]
        elif same_sign(point_2_distance, point_3_distance):
            separated_points[0] = points[1]
            separated_points[1] = points[2]
            separated_points[2] = points[0]
        else:
            raise RuntimeError("All the points share the same sign or all are zero.")

        return separated_points

    def detect(self) -> bool:
        """Detects collisions between two triangles in 3D space.

        Returns:
            bool:
                True iff there is a collision between the two triangles in 3D space.
        """

        if self.triangle_1_plane_equation.all_vertices_lie_on_one_side_of_plane(self.triangle_2):
            return False
        elif self.triangle_2_plane_equation.all_vertices_lie_on_one_side_of_plane(self.triangle_1):
            return False
        elif self.triangles_lie_in_the_same_plane():
            return False

        triangle_1_separated_points = self.seperate_points_by_signed_distance(self.triangle_1,
                                                                              self.triangle_2_plane_equation)
        triangle_2_separated_points = self.seperate_points_by_signed_distance(self.triangle_2,
                                                                              self.triangle_1_plane_equation)

        # Calculate the two locations where triangle 1 intersects plane 2.
        intersection_point_1 = intersection_of_a_line_and_a_plane(triangle_1_separated_points[0],
                                                                  triangle_1_separated_points[2],
                                                                  self.triangle_2_plane_equation)

        intersection_point_2 = intersection_of_a_line_and_a_plane(triangle_1_separated_points[1],
                                                                  triangle_1_separated_points[2],
                                                                  self.triangle_2_plane_equation)

        direction = np.cross(self.triangle_1_plane_equation.n, self.triangle_2_plane_equation.n)
        # line(T) = zero_point_on_line + (D * T)
        # intersection_point_2 - zero_point_on_line = D * T
        # We know the ^ point is on the line +- floating point rounding.
        # T is a scalar. We have three linear equations in T. We know they are satisfiable +- floating point rounding.
        # Use just the equation in x to solve for T.

        t1 = 0.
        t2 = (intersection_point_2[0] - intersection_point_1[0]) / direction[0]

        # Calculate the two locations where triangle 2 intersects plane 1.
        intersection_point_3 = intersection_of_a_line_and_a_plane(triangle_2_separated_points[0],
                                                                  triangle_2_separated_points[2],
                                                                  self.triangle_1_plane_equation)

        intersection_point_4 = intersection_of_a_line_and_a_plane(triangle_2_separated_points[1],
                                                                  triangle_2_separated_points[2],
                                                                  self.triangle_1_plane_equation)

        t3 = (intersection_point_3[0] - intersection_point_1[0]) / direction[0]
        t4 = (intersection_point_4[0] - intersection_point_1[0]) / direction[0]

        interval_1 = Interval(t1, t2)
        interval_2 = Interval(t3, t4)

        return interval_1.intersects(interval_2)
