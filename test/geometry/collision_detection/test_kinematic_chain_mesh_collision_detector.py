from typing import List
import pytest
import numpy as np
from geometry.collision_detection.kinematic_chain_mesh_collision_detector import KinematicChainMeshCollisionDetector
from geometry.kinematic_chain import KinematicOpenChain
from geometry.mesh import Mesh
from robots.ur5 import define_ur5_kinematic_chain, UR5Meshes

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

    return

    meshes = ur5_meshes

    base_mesh = meshes[0]
    link_2_mesh = meshes[1]


    base_vertices = []
    for triangle in base_mesh.triangles:
        for point in triangle:
            base_vertices.append(point)

    link_2_vertices = []
    for triangle in link_2_mesh.triangles:
        for point in triangle:
            transformed_point = ...
            link_2_vertices.append(transformed_point)

    base_vertices = np.array(base_vertices)
    link_2_vertices = np.array(link_2_vertices)

    #breakpoint()
    base_mesh_triangles = o3d.utility.Vector3dVector(base_vertices)
    link_2_mesh_triangles = o3d.utility.Vector3dVector(link_2_vertices)
    visualize_mesh(base_mesh_triangles, None)
