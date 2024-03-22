import numpy as np
import pytest
from geometry.primitives.screw import Screw

w1 = np.array([1., 2., 3.])
v1 = np.array([4., 5., 6.])
s1 = np.vstack((w1, v1))

q1 = np.array([1., 2., 3.])
s1 = np.array([0., 0., 1.])
h1 = 1
angle_1 = 2.
normalized_screw_axis_1 = np.array([0., 0., 1., 2., -1., 1.])
transformation_1 = np.array([[-0.41614684, -0.90929743, 0., 3.23474169],
                            [0.90929743, -0.41614684,  0., 1.92299625],
                            [0.,  0., 1., 2.],
                            [0., 0.,  0., 1.]])

q2 = np.array([0., 0., 0.])
s2 = np.array([0., 0., 1.])
h2 = 1
angle_2 = 3.5
transformation_2 = np.array([[-0.93645669, 0.35078323, 0., 0.],
                             [-0.35078323, -0.93645669, 0., 0.],
                             [0., 0., 1., 3.5],
                             [0., 0., 0., 1.]])
normalized_screw_axis_2 = np.array([0., 0., 1., 0., 0., 1.])

q3 = np.array([0., 0., 0.])
s3 = np.array([0., 1., 0.])
h3 = 2
angle_3 = -1.
transformation_3 = np.array([[0.54030231, 0., -0.84147098, 0.],
                             [0., 1., 0., -2.],
                             [0.84147098, 0., 0.54030231, 0.],
                             [0., 0., 0., 1.]])
normalized_screw_axis_3 = np.array([0., 1., 0., 0., 2., 0.])

q4 = np.array([0., 0., 0.])
s4 = np.array([0., 1., 0.])
h4 = 0
angle_4 = 2.5
transformation_4 = np.array([[-0.80114362, 0., 0.59847214, 0.],
                             [0., 1., 0., 0.],
                             [-0.59847214, 0., -0.80114362, 0.],
                             [0., 0., 0., 1.]])
normalized_screw_axis_4 = np.array([0., 1., 0., 0., 0., 0.])

q5 = np.array([4., 5., 6.])
s5 = np.array([0., 1., 0.])
h5 = 0
angle_5 = 2.5
transformation_5 = np.array([[-0.80114362, 0., 0.59847214, 3.6137416],
                             [0., 1., 0.,  0.],
                             [-0.59847214, 0., -0.80114362, 13.20075027],
                             [0., 0., 0., 1.]])
normalized_screw_axis_5 = np.array([0., 1., 0., -6., 0., 4.])


@pytest.mark.parametrize("q, s, h, expected_6_vector", [(q1, s1, h1, normalized_screw_axis_1),
                                                        (q2, s2, h2, normalized_screw_axis_2),
                                                        (q3, s3, h3, normalized_screw_axis_3),
                                                        (q4, s4, h4, normalized_screw_axis_4),
                                                        (q5, s5, h5, normalized_screw_axis_5)])
def test_as_6_vector(q: np.ndarray, s: np.ndarray, h: float, expected_6_vector: np.ndarray):

    screw = Screw.construct_from_q_s_h(q, s, h)
    assert np.allclose(screw.as_6_vector(), expected_6_vector)


@pytest.mark.parametrize("q, s, h, theta, expected_twist", [(q1, s1, h1, 2., normalized_screw_axis_1 * 2),
                                                            (q2, s2, h2, 3.5, normalized_screw_axis_2 * 3.5),
                                                            (q3, s3, h3, -1., normalized_screw_axis_3 * -1.),
                                                            (q4, s4, h4, 2., normalized_screw_axis_4 * 2.),
                                                            (q5, s5, h5, 3., normalized_screw_axis_5 * 3.)])
def test_twist(q: np.ndarray, s: np.ndarray, h: float, theta: float, expected_twist: np.ndarray):

    screw = Screw.construct_from_q_s_h(q, s, h)
    twist = screw.twist(theta)
    assert np.allclose(twist, expected_twist)


@pytest.mark.parametrize("q, s, h, theta, expected_transformation", [(q1, s1, h1, angle_1, transformation_1),
                                                                     (q2, s2, h2, angle_2, transformation_2),
                                                                     (q3, s3, h3, angle_3, transformation_3),
                                                                     (q4, s4, h4, angle_4, transformation_4),
                                                                     (q5, s5, h5, angle_5, transformation_5)])
def test_transformation(q: np.ndarray, s: np.ndarray, h: float, theta: float, expected_transformation: np.ndarray):

    screw = Screw.construct_from_q_s_h(q, s, h)
    transformation = screw.transformation(theta)
    assert np.allclose(transformation.matrix, expected_transformation)


