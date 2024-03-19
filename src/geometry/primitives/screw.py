import numpy as np

from geometry.primitives.transformation import Transformation
from geometry.primitives.utilities import little_so3


class Screw:
    """
    Represents a rotation about and around a screw in 3D space.
    See https://hades.mech.northwestern.edu/images/7/7f/MR.pdf page 123.
    """

    def __init__(self, q: np.ndarray, s: np.ndarray, h: float):
        """Constructor.

        Args:
            q: numpy.ndarray
                A 3D point on the screw.
            s: numpy.ndarray
                Direction of the screw in 3D space.
            h: float
                Ratio of the linear velocity along the screw axis to the angular
                velocity θ̇_dot about the screw axis
        """

        self.q = q
        self.s = s
        self.h = h

        self.w = self.s
        self.v = np.cross(-self.s, self.q) + (self.h * self.s)

    def as_6_vector(self) -> np.ndarray:
        """..."""
        return np.append(self.w, self.v)

    def twist(self, theta_dot: float) -> np.ndarray:
        """Calculates the twist associated when rotating at the provided rate.

        Args:
            theta_dot: float
                The angular velocity about the screw axis.

        Returns:
            numpy.ndarray:
                The twist which occurs when rotating about the screw axis at the provided velocity.
        """
        return self.as_6_vector() * theta_dot

    def bracket_s(self) -> np.ndarray:
        """..."""
        return little_so3(self.as_6_vector())

    def rotation_matrix(self, theta: float) -> np.ndarray:
        """Computes the rotation matrix transformation matrix corresponding to rotating by theta radians
           about the screw axis.

        Notes:
            MR Link ...
            I + sin θ[ωˆ] + (1 − cos θ)[ωˆ]2

        Returns:
            numpy.ndarray:
                3x3 rotation matrix.
        """
        matrix = np.eye(3)

        bracket_w = little_so3(self.w)
        bracket_w_squared = bracket_w @ bracket_w

        return matrix + (bracket_w * np.sin(theta)) + ((1 - np.cos(theta)) * bracket_w_squared)

    def translation(self, theta: float) -> np.ndarray:
        """Computes the translation component of the transformation matrix corresponding to rotating by theta radians
           about the screw axis.

        Notes:
            MR link ...

            (Iθ + (1 − cos θ)[ω] + (θ − sin θ)[ω]2)v

        Returns:
            numpy.ndarray:
                [x, y, z]
        """
        matrix = np.eye(3)

        bracket_w = little_so3(self.w)
        bracket_w_squared = bracket_w @ bracket_w

        matrix += bracket_w * (1. - np.cos(theta))
        matrix += bracket_w_squared * (theta - np.sin(theta))

        return matrix @ self.v

    def transformation(self, theta: float) -> Transformation:
        """Computes the translation matrix corresponding to rotating by theta radians about the screw axis.

        Args:
            theta: float
                Angular distance (radians) rotated about the screw axis.

        Returns:
            Transformation:
                The translation matrix corresponding to rotating by theta radians about the screw axis.
        """
        matrix = np.eye(4)

        matrix[0:3, 0:3] = self.rotation_matrix(theta)
        x, y, z = self.translation(theta)

        matrix[0][3] = x
        matrix[1][3] = y
        matrix[2][3] = z

        return matrix
