import pytest
from test.geometry.collision_detection.constant_triangles import (triangle_1, triangle_2, triangle_3, triangle_4,
                                                             triangle_5, triangle_6, triangle_7, triangle_8,
                                                             triangle_9, triangle_10, triangle_11, triangle_12,
                                                             triangle_13, triangle_14, triangle_15, triangle_16,
                                                             triangle_17, triangle_18, triangle_20, triangle_19,
                                                                  triangle_21, triangle_22)
from geometry.collision_detection.triangle_triangle_collision_detector import TriangleTriangleCollisionDetector

collision_test_case_1 = (TriangleTriangleCollisionDetector(triangle_1, triangle_2), False)
collision_test_case_2 = (TriangleTriangleCollisionDetector(triangle_3, triangle_4), False)
collision_test_case_3 = (TriangleTriangleCollisionDetector(triangle_5, triangle_6), False)
collision_test_case_4 = (TriangleTriangleCollisionDetector(triangle_7, triangle_8), False)
collision_test_case_5 = (TriangleTriangleCollisionDetector(triangle_9, triangle_10), True)
collision_test_case_6 = (TriangleTriangleCollisionDetector(triangle_10, triangle_9), True)
collision_test_case_7 = (TriangleTriangleCollisionDetector(triangle_11, triangle_12), True)
collision_test_case_8 = (TriangleTriangleCollisionDetector(triangle_12, triangle_11), True)
collision_test_case_9 = (TriangleTriangleCollisionDetector(triangle_13, triangle_14), False)
collision_test_case_10 = (TriangleTriangleCollisionDetector(triangle_14, triangle_13), False)
collision_test_case_11 = (TriangleTriangleCollisionDetector(triangle_15, triangle_16), False)
collision_test_case_12 = (TriangleTriangleCollisionDetector(triangle_16, triangle_15), False)
collision_test_case_13 = (TriangleTriangleCollisionDetector(triangle_17, triangle_18), False)
collision_test_case_14 = (TriangleTriangleCollisionDetector(triangle_18, triangle_17), False)
collision_test_case_15 = (TriangleTriangleCollisionDetector(triangle_19, triangle_20), True)
collision_test_case_16 = (TriangleTriangleCollisionDetector(triangle_21, triangle_22), False)


triangle_triangle_collision_test_cases = [collision_test_case_1, collision_test_case_2, collision_test_case_3,
                                          collision_test_case_4, collision_test_case_5, collision_test_case_6,
                                          collision_test_case_7, collision_test_case_8, collision_test_case_9,
                                          collision_test_case_10, collision_test_case_11, collision_test_case_12,
                                          collision_test_case_13, collision_test_case_14, collision_test_case_15,
                                          collision_test_case_16
                                          ]

triangle_triangle_collision_test_cases = [collision_test_case_16]
triangle_triangle_collision_test_cases = []

@pytest.mark.parametrize("detector, expected", triangle_triangle_collision_test_cases)
def test_collision_detector(detector: TriangleTriangleCollisionDetector, expected: bool):

    assert detector.detect() == expected
