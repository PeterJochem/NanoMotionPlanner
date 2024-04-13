import numpy as np
from geometry.primitives.utilities import construct_matrix, euler_angles
from utilities import row_to_string


class Transformation:
    """
    Represents a transformation (position and rotation) in 3D space.
    """

    def __init__(self, matrix: np.ndarray):
        """Constructor.

        matrix: numpy.ndarray
            The (4, 4) transformation matrix.
        """
        self.matrix = matrix

    def euler_angles(self) -> np.ndarray:
        """Gets the underlying Euler angles.

        Returns:
            numpy.ndarray:
                [roll, pitch, yaw] (in radians)
        """
        return euler_angles(self.matrix)

    def roll(self) -> float:
        """Gets the rotation about the x-axis, in radians.

        Returns:
            float:
                The rotation about the x-axis, in radians.
        """
        return self.euler_angles()[0]

    def pitch(self) -> float:
        """Gets the rotation about the y-axis, in radians.

        Returns:
            float:
                The rotation about the y-axis, in radians.
        """
        return self.euler_angles()[1]

    def yaw(self) -> float:
        """Gets the rotation about the z-axis, in radians.

        Returns:
            float:
                The rotation about the z-axis, in radians.
        """
        return self.euler_angles()[2]

    def transform(self, right: "Transformation") -> "Transformation":
        """Computes the product of this transformation and the provided one.

        Args:
            right: "Transformation"

        Returns:
            Transformation:
                The product of this transformation and the provided one.
        """
        return Transformation(self.matrix @ right)

    def transform_point(self, point: np.ndarray) -> np.ndarray:
        """Applies the transformation to the point.

        Args:
            point: numpy.ndarray
                Apply the transformation to this point.

        Returns:
            numpy.ndarray
                The transformed point.
        """
        x, y, z = point
        homogenous_point = np.array([x, y, z, 1.])
        transformed_homogenous_point = self.matrix @ homogenous_point
        return transformed_homogenous_point[0:3]

    def transform_triangle(self, triangle: np.ndarray) -> np.ndarray:
        """Applies the transformation to all the points in the triangle.

        Args:
            triangle: numpy.ndarray
                3x3 array. 3 vertices of a triangle in 3D space.

        Returns:
            numpy.ndarray:
                3x3 array. The 3 vertices of the triangle after application of the transformation.
        """

        transformed_triangle = np.zeros((3, 3))
        for i, point in enumerate(triangle):
            transformed_triangle[i] = self.transform_point(point)
        return transformed_triangle

    def __mul__(self, right: "Transformation") -> "Transformation":
        """Defines the behavior of the * operator.

        Args:
            right: Transformation

        Returns:
            Transformation:
                The result of composing the two transformations.
        """

        if isinstance(right, Transformation):
            return Transformation(self.matrix @ right.matrix)
        else:
            raise TypeError(f"Unsupported operand type(s) for *: {type(self).__name__} and {type(right).__name__}")

    def __str__(self) -> str:
        string = "["
        string += row_to_string(self.matrix[0]) + '\n'
        string += row_to_string(self.matrix[1]) + '\n'
        string += row_to_string(self.matrix[2]) + '\n'
        string += row_to_string(self.matrix[3])
        string += "]"
        return string

    @staticmethod
    def construct(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> "Transformation":
        """Constructs a new Transformation from the provided matrix.

        Returns:
            Transformation:
                Represents the provided matrix.
        """
        matrix = construct_matrix(x, y, z, roll, pitch, yaw)
        return Transformation(matrix)

    @staticmethod
    def identity() -> "Transformation":
        """Constructs the identity Transformation.

        Returns:
            Transformation:
        """
        return Transformation.construct(0., 0., 0., 0., 0., 0.)
