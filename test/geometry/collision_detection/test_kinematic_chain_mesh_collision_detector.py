from typing import List
import pytest
import numpy as np
from geometry.collision_detection.kinematic_chain_mesh_collision_detector import KinematicChainMeshCollisionDetector
from geometry.kinematic_chain import KinematicOpenChain
from geometry.mesh import Mesh
from geometry.primitives.transformation import Transformation
from geometry.utilities import multiply
from robots.ur5 import define_ur5_kinematic_chain, UR5Meshes, UR5ZeroAngleTransformations

import open3d as o3d
import open3d as mylib

ur5_kinematic_chain = define_ur5_kinematic_chain()
ur5_meshes = UR5Meshes().ordered_meshes

test_case_1 = (ur5_kinematic_chain, ur5_meshes, np.zeros(6), False)

kinematic_chain_mesh_collision_test_cases = [test_case_1]


def visualize_mesh(vertices, faces):
    """
    Visualize a mesh using Open3D.

    Parameters:
        vertices (numpy array): Vertices of the mesh.
        faces (numpy array): Faces of the mesh.
    """
    # Create Open3D triangle mesh
    mesh = mylib.geometry.TriangleMesh()
    mesh.vertices = mylib.utility.Vector3dVector(vertices)
    #mesh.triangles = mylib.utility.Vector3iVector(faces)

    # Visualize the mesh
    #mylib.visualization.draw_geometries([mesh])

    pcd = o3d.geometry.PointCloud()
    pcd.points = mylib.utility.Vector3dVector(vertices)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])

@pytest.mark.parametrize("kinematic_chain, meshes, joint_angles, expected", kinematic_chain_mesh_collision_test_cases)
def test_kinematic_chain_mesh_collision_detector(kinematic_chain: KinematicOpenChain,
                                                 meshes: List[Mesh],
                                                 joint_angles: np.ndarray,
                                                 expected: bool):

    detector = KinematicChainMeshCollisionDetector(kinematic_chain, meshes)
    #assert detector.detect(joint_angles) == expected
    ...


def test_dummy():

    meshes = ur5_meshes

    base_mesh = meshes[0]
    link_2_mesh = meshes[1]
    link_3_mesh = meshes[2]
    link_4_mesh = meshes[3]
    link_5_mesh = meshes[4]
    link_6_mesh = meshes[5]


    Ts = UR5ZeroAngleTransformations().transformations

    base_vertices = []
    base_to_mesh_1 = Transformation.identity()  # multiply(Ts[:0])
    for triangle in base_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_1.transform_point(point)
            base_vertices.append(transformed_point)

    link_2_vertices = []
    base_to_mesh_2 = multiply(Ts[:5])
    for triangle in link_2_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_2.transform_point(point)
            link_2_vertices.append(transformed_point)

    link_3_vertices = []
    base_to_mesh_3 = multiply(Ts[:7])
    for triangle in link_3_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_3.transform_point(point)
            link_3_vertices.append(transformed_point)

    link_4_vertices = []
    base_to_mesh_4 = multiply(Ts[:9])
    for triangle in link_4_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_4.transform_point(point)
            link_4_vertices.append(transformed_point)

    link_5_vertices = []
    base_to_mesh_5 = multiply(Ts[:11])
    for triangle in link_5_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_5.transform_point(point)
            link_5_vertices.append(transformed_point)

    link_6_vertices = []
    base_to_mesh_6 = multiply(Ts[:13])
    for triangle in link_6_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_6.transform_point(point)
            link_6_vertices.append(transformed_point)

    robot_vertices = np.array(base_vertices + link_2_vertices)  # + link_3_vertices + link_4_vertices)
    robot_vertices = np.array(base_vertices + link_2_vertices + link_3_vertices + link_4_vertices + link_5_vertices + link_6_vertices)

    base_vertices = np.array(base_vertices)
    link_2_vertices = np.array(link_2_vertices)
    link_3_vertices = np.array(link_3_vertices)
    link_4_vertices = np.array(link_4_vertices)
    link_5_vertices = np.array(link_5_vertices)

    #breakpoint()
    base_mesh_triangles = o3d.utility.Vector3dVector(base_vertices)
    link_2_mesh_triangles = o3d.utility.Vector3dVector(link_2_vertices)
    link_3_mesh_triangles = o3d.utility.Vector3dVector(link_3_vertices)

    robot_mesh_triangles = o3d.utility.Vector3dVector(robot_vertices)

    visualize_mesh(base_mesh_triangles, None)
    visualize_mesh(link_2_mesh_triangles, None)
    visualize_mesh(robot_mesh_triangles, None)


