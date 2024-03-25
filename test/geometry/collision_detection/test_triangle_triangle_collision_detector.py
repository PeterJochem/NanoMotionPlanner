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

triangle_5 = np.array([[0., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
triangle_6 = np.array([[1., 0., 0.], [2., 0., 1.], [3., 2. + 1., 0.]])
collision_test_case_3 = (TriangleTriangleCollisionDetector(triangle_5, triangle_6), False)

triangle_7 = np.array([[0., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
triangle_8 = np.array([[1., 0., 0.], [2., 0., 1.], [0., 2. + 1., 0.]])
collision_test_case_4 = (TriangleTriangleCollisionDetector(triangle_7, triangle_8), False)

triangle_9 = np.array([[1.1, 1.4, 1.6], [3., 1., 1.], [2., 3., 1.]])
triangle_10 = np.array([[0., 2., 0.], [4., 2., 2.2], [3., 4., 2.]])
collision_test_case_5 = (TriangleTriangleCollisionDetector(triangle_9, triangle_10), True)
collision_test_case_6 = (TriangleTriangleCollisionDetector(triangle_10, triangle_9), True)

triangle_11 = np.array([[1., 1., 1.], [3., 1., 1.], [2., 3., 1.]])
triangle_12 = np.array([[0., 1.5, -1.], [5., 1.5, 3.], [4., 3., 5.]])
collision_test_case_7 = (TriangleTriangleCollisionDetector(triangle_11, triangle_12), True)
collision_test_case_8 = (TriangleTriangleCollisionDetector(triangle_12, triangle_11), True)

triangle_13 = np.array([[1., 1., 1.], [3., 1., 1.], [2., 3., 1.]])
triangle_14 = np.array([[2., 2., 0.], [4., 2., 0.], [3., 4., 0.]])
collision_test_case_9 = (TriangleTriangleCollisionDetector(triangle_13, triangle_14), False)
collision_test_case_10 = (TriangleTriangleCollisionDetector(triangle_14, triangle_13), False)

triangle_15 = np.array([[1., 1., 1.], [3., 1., 1.], [2., 3., 1.]])
triangle_16 = np.array([[2., 2., 0.], [4., 2., 0.], [3., 4., 0.]])
collision_test_case_11 = (TriangleTriangleCollisionDetector(triangle_15, triangle_16), False)
collision_test_case_12 = (TriangleTriangleCollisionDetector(triangle_16, triangle_15), False)

triangle_17 = np.array([[1., 1., 2.2], [3., 1., 1.2], [2., 3., 1.]])
triangle_18 = np.array([[2., 2., 0.], [4., 2., 0.], [3., 4., 0.]])
collision_test_case_13 = (TriangleTriangleCollisionDetector(triangle_17, triangle_18), False)
collision_test_case_14 = (TriangleTriangleCollisionDetector(triangle_18, triangle_17), False)

triangle_19 = np.array([[1., 1., 2.2], [3., 1., 1.2], [2., 3., 1.]])
triangle_20 = np.array([[2., 2., 0.], [4., 2., 5.], [3., 4., 9.]])
collision_test_case_15 = (TriangleTriangleCollisionDetector(triangle_19, triangle_20), True)

test_cases = []
test_cases = [collision_test_case_1]
test_cases += [collision_test_case_2, collision_test_case_3, collision_test_case_4,
              collision_test_case_5, collision_test_case_6, collision_test_case_7,
              collision_test_case_8, collision_test_case_9, collision_test_case_10,
              collision_test_case_11, collision_test_case_12, collision_test_case_13,
              collision_test_case_14, collision_test_case_15]

@pytest.mark.parametrize("detector, expected", test_cases)
def test_collision_detector(detector: TriangleTriangleCollisionDetector, expected: bool):

    assert detector.detect() == expected
