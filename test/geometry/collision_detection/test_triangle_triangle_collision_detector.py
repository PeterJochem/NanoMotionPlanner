import numpy as np
import pytest
from geometry.collision_detection.triangle_triangle_collision_detector import TriangleTriangleCollisionDetector

# The triangles don't overlap but they lie in the same plane.
triangle_1 = np.array([[0., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
triangle_2 = np.array([[0., 2., 0.], [0., 2., 1.], [0., 2. + 1., 0.]])
collision_test_case_1 = (TriangleTriangleCollisionDetector(triangle_1, triangle_2), False)


triangle_3 = np.array([[0., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
triangle_4 = np.array([[1., 2., 0.], [2., 2., 1.], [3., 2. + 1., 0.]])
collision_test_case_2 = (TriangleTriangleCollisionDetector(triangle_3, triangle_4), False)


@pytest.mark.parametrize("detector, expected", [collision_test_case_2])
def test_collision_detector(detector: TriangleTriangleCollisionDetector, expected: bool):

    assert detector.detect() == expected
