import numpy as np
import math


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Constructs a rotation matrix from the roll, pitch, yaw (in radians).

    See the equations presented here: https://msl.cs.uiuc.edu/planning/node102.html

    Args:
        roll: float
            Rotation about the x-axis, in radians.
        pitch: float
            Rotation about the y-axis, in radians.
        yaw: float
            Rotation about the x-axis, in radians.

    Returns:
        numpy.ndarray
            The rotation matrix.
    """

    Rx = np.zeros((3, 3))
    Ry = np.zeros((3, 3))
    Rz = np.zeros((3, 3))

    Rx[0][0] = 1.
    Rx[1][1] = math.cos(roll)
    Rx[1][2] = -math.sin(roll)
    Rx[2][1] = math.sin(roll)
    Rx[2][2] = math.cos(roll)

    Ry[0][0] = math.cos(pitch)
    Ry[0][2] = math.sin(pitch)
    Ry[1][1] = 1.
    Ry[2][0] = -math.sin(pitch)
    Ry[2][2] = math.cos(pitch)

    Rz[0][0] = math.cos(yaw)
    Rz[0][1] = -math.sin(yaw)
    Rz[1][0] = math.sin(yaw)
    Rz[1][1] = math.cos(yaw)
    Rz[2][2] = 1.

    return Rx @ Ry @ Rz


def construct_matrix(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Constructs the underlying (4, 4) transformation matrix.

    Args:
        x: float
            X position, in meters.
        y: float
            Y position, in meters.
        z: float
            Z position, in meters.
        roll: float
            Rotation about the x-axis, in meters.
        pitch: float
            Rotation about the y-axis, in meters.
        yaw: float
            Rotation about the z-axis, in meters.

    Returns:
        numpy.ndarray:
            The underlying (4, 4) transformation matrix.
    """
    matrix = np.zeros((4, 4))
    matrix[0:3, 0:3] = rotation_matrix(roll, pitch, yaw)
    matrix[0][3] = x
    matrix[1][3] = y
    matrix[2][3] = z
    matrix[3][3] = 1.
    return matrix
