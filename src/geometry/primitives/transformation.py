import numpy as np
from geometry.primitives.utilities import construct_matrix


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
