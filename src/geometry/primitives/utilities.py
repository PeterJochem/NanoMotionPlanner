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

    return (Rx @ Ry @ Rz).T
    #return (Rz @ Ry @ Rx).T


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


def euler_angles(matrix) -> np.ndarray:
    """Extracts the Euler angles (XYZ convention).

        Notes:
            https://msl.cs.uiuc.edu/planning/node103.html

    Args:
        matrix: numpy.ndarray

    Returns:
        numpy.ndarray:
            Euler angles (roll, pitch, yaw).
    """

    y = matrix[1][0]
    x = matrix[0][0]
    alpha = -math.atan2(y, x)

    y = -matrix[2][0]
    x = math.sqrt((matrix[2][1] ** 2) + (matrix[2][2] ** 2))
    beta = -math.atan2(y, x)

    y = matrix[2][1]
    x = matrix[2][2]
    gamma = -math.atan2(y, x)

    return gamma, beta, alpha


def little_so3(v: np.ndarray) -> np.ndarray:
    """Constructs the so(3) matrix of the provided 3 vector.

    Notes:
        https://hades.mech.northwestern.edu/images/7/7f/MR.pdf

    Args:
        v: numpy.ndarray
            3 vector.

    Returns:
        numpy.ndarray:
            so(3) representation of v.
    """
    matrix = np.zeros((3, 3))

    # First Row
    matrix[0][1] = -v[2]
    matrix[0][2] = v[1]

    # Second Row
    matrix[1][0] = v[2]
    matrix[1][2] = -v[0]

    # Third Row
    matrix[2][0] = -v[1]
    matrix[2][1] = v[0]

    return matrix
