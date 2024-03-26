import pytest
import numpy as np
from geometry.collision_detection.mesh_mesh_collision_detector import MeshMeshCollisionDetector
from geometry.mesh import Mesh
from test.geometry.collision_detection.constant_triangles import (triangle_1, triangle_2, triangle_3, triangle_4,
                                                                  triangle_5, triangle_6, triangle_7, triangle_8,
                                                                  triangle_9, triangle_10, triangle_11, triangle_12,
                                                                  triangle_13, triangle_14, triangle_15, triangle_16,
                                                                  triangle_17, triangle_18, triangle_19, triangle_20
                                                                  )


test_case_1 = (Mesh(np.array([triangle_1])), Mesh(np.array([triangle_2])), False)
test_case_2 = (Mesh(np.array([triangle_3])), Mesh(np.array([triangle_4])), False)
test_case_3 = (Mesh(np.array([triangle_5])), Mesh(np.array([triangle_6])), False)
test_case_4 = (Mesh(np.array([triangle_7])), Mesh(np.array([triangle_8])), False)
test_case_5 = (Mesh(np.array([triangle_9])), Mesh(np.array([triangle_10])), True)
test_case_6 = (Mesh(np.array([triangle_10])), Mesh(np.array([triangle_9])), True)
test_case_7 = (Mesh(np.array([triangle_11])), Mesh(np.array([triangle_12])), True)
test_case_8 = (Mesh(np.array([triangle_12])), Mesh(np.array([triangle_11])), True)
test_case_9 = (Mesh(np.array([triangle_13])), Mesh(np.array([triangle_14])), False)
test_case_10 = (Mesh(np.array([triangle_14])), Mesh(np.array([triangle_13])), False)
test_case_11 = (Mesh(np.array([triangle_15])), Mesh(np.array([triangle_16])), False)
test_case_12 = (Mesh(np.array([triangle_16])), Mesh(np.array([triangle_15])), False)
test_case_13 = (Mesh(np.array([triangle_17])), Mesh(np.array([triangle_18])), False)
test_case_14 = (Mesh(np.array([triangle_18])), Mesh(np.array([triangle_17])), False)
test_case_15 = (Mesh(np.array([triangle_19])), Mesh(np.array([triangle_20])), True)
test_case_16 = (Mesh(np.array([triangle_20])), Mesh(np.array([triangle_19])), True)


mesh_mesh_collision_test_cases = [test_case_1, test_case_2, test_case_3, test_case_4, test_case_5, test_case_6,
                                  test_case_7, test_case_8, test_case_9, test_case_10, test_case_11, test_case_12,
                                  test_case_13, test_case_14, test_case_15, test_case_16]


@pytest.mark.parametrize("mesh_1, mesh_2, expected", mesh_mesh_collision_test_cases)
def test_dummy(mesh_1: Mesh, mesh_2: Mesh, expected: bool):

    detector = MeshMeshCollisionDetector(mesh_1, mesh_2)
    assert detector.detect() == expected

