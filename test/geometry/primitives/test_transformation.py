from typing import List
import pytest
import numpy as np
from nano_motion_planner.src.geometry.primitives.transformation import Transformation

matrix_1 = np.eye(4)
matrix_1[0][3] = 1.
matrix_1[1][3] = 2.
matrix_1[2][3] = 3.

# Rotation about the x-axis by 1.57 radians.
matrix_2 = np.array([[1., 0., 0., 0.],
                     [0., 0.0007963, -1, 0.],
                     [0., 1, 0.0007963, 0.],
                     [0., 0., 0., 1.]]).T

# https://www.andre-gaschler.com/rotationconverter/
matrix_3 = np.array([[-0.0357114, -0.3980418, -0.9166719, 0.],
                     [-0.5035815, 0.7994576, -0.3275261, 0.],
                     [0.8632094, 0.4499226, -0.2289962, 0.],
                     [0., 0., 0., 1.]])

matrix_4_t = np.array([[-0.0357114, -0.5035815, -0.8632094, 12.5],
                       [0.9961295, 0.0514817, -0.0712439, 25.5],
                       [0.0803166, -0.8624125, 0.4997939, 55.5],
                       [0., 0., 0., 1.]])

matrix_4 = np.array([[-0.0357114, 0.9961295, 0.0803166, 12.5],
                     [-0.5035815, 0.0514817, -0.8624125, 25.5],
                     [-0.8632094, -0.0712439, 0.4997939, 55.5],
                     [0., 0., 0., 1.]])


@pytest.mark.parametrize("values, expected_matrix", [([0., 0., 0., 0., 0., 0.], np.eye(4)),
                                                     ([1., 2., 3., 0., 0., 0.], matrix_1),
                                                     ([0., 0., 0., 1.57, 0., 0.], matrix_2),
                                                     ([0., 0., 0., 1.1, 2.1, -1.5], matrix_3),
                                                     ([12.5, 25.5, 55.5, -3., -2.1, -1.5], matrix_4)])
def test_construct_from_matrix(values: List[float], expected_matrix: np.ndarray):
    transformation = Transformation.construct(*values)
    assert np.allclose(transformation.matrix, expected_matrix, atol=1e-5)


@pytest.mark.parametrize("left, right, expected", [(Transformation.identity(),
                                                    Transformation.identity(),
                                                    Transformation.identity()),

                                                   (Transformation.construct(1., 2., 3., 0., 0., 0.),
                                                    Transformation.construct(4., 5., 10., 0., 0., 0.),
                                                    Transformation.construct(5., 7., 13., 0., 0., 0.)),

                                                   (Transformation.construct(0., 0., 0., 0., 0., 0.),
                                                    Transformation.construct(0., 0., 0., 1., 1.1, 1.2),
                                                    Transformation.construct(0., 0., 0., 1., 1.1, 1.2))])
def test_multiplication_operator(left: Transformation, right: Transformation, expected: Transformation):
    result = left * right
    assert np.allclose(result.matrix, expected.matrix, atol=1e-5)


@pytest.mark.parametrize("transformation, point, expected", [(Transformation.identity(),
                                                              np.array([1., 2., 3.]),
                                                              np.array([1., 2., 3.]))])
def test_transform_point(transformation: Transformation, point: np.ndarray, expected: np.ndarray):
    result = transformation.transform_point(point)
    assert np.allclose(result, expected, atol=1e-5)


@pytest.mark.parametrize("transformation, expected_angles", [
    (Transformation.construct(0., 0., 0., 0., 0., 0.),
     np.array([0., 0., 0.])),
    (Transformation.construct(0., 0., 0., 0., 0., -2.1),
     np.array([0., 0.0, -2.1])),
    (Transformation.construct(0., 0., 0., 1., 2., 3.),
     np.array([-2.14159, 1.14159, -0.14159])),
    (Transformation.construct(0., 0., 0., -1., 2., -3.),
     np.array([2.1415926, 1.1415926, 0.1415927]))])
def test_extracting_euler_angles_from_transformation(transformation, expected_angles: np.ndarray):
    roll, pitch, yaw = transformation.euler_angles()

    assert np.isclose(roll, expected_angles[0], atol=1e-5)
    assert np.isclose(pitch, expected_angles[1], atol=1e-5)
    assert np.isclose(yaw, expected_angles[2], atol=1e-5)
