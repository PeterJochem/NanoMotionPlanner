from typing import List
import pytest
import numpy as np
import open3d as o3d
import open3d as mylib
from geometry.collision_detection.kinematic_chain_mesh_collision_detector import KinematicChainMeshCollisionDetector
from geometry.kinematic_chain import KinematicOpenChain
from geometry.mesh import Mesh
from geometry.primitives.transformation import Transformation
from geometry.utilities import multiply
from robots.ur5 import define_ur5_kinematic_chain, UR5Meshes, UR5ZeroAngleTransformations

ur5_kinematic_chain = define_ur5_kinematic_chain()
ur5_meshes = UR5Meshes().ordered_meshes

test_case_1 = (ur5_kinematic_chain, ur5_meshes, np.zeros(6), False)

kinematic_chain_mesh_collision_test_cases = [test_case_1]


def visualize_mesh(vertices, origin_vertices):
    """
    Visualize a mesh using Open3D.

    Parameters:
        vertices (numpy array): Vertices of the mesh.
        faces (numpy array): Faces of the mesh.
    """
    # Create Open3D triangle mesh
    mesh = mylib.geometry.TriangleMesh()
    mesh.vertices = mylib.utility.Vector3dVector(vertices)

    faces = np.array([np.array([3 * i, ((3 * i) + 1), ((3 * i) + 2)]) for i in range(len(mesh.vertices))])
    mesh.triangles = mylib.utility.Vector3iVector(faces)

    pcd = o3d.geometry.PointCloud()
    pcd.points = mylib.utility.Vector3dVector(origin_vertices)
    origins = np.asarray(origin_vertices)
    link_3_origin = origins[2]


    # Visualize the point cloud
    #o3d.visualization.draw_geometries([pcd])
    #mylib.visualization.draw_geometries([mesh, pcd])

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for i in range(7):
        g = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.16, origin=origins[i])
        vis.add_geometry(g)

    vis.add_geometry(mesh)
    vis.run()



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
    ur5_kinematic_chain = define_ur5_kinematic_chain()

    base_mesh = meshes[0]
    link_2_mesh = meshes[1]
    link_3_mesh = meshes[2]
    link_4_mesh = meshes[3]
    link_5_mesh = meshes[4]
    link_6_mesh = meshes[5]
    link_7_mesh = meshes[6]

    angle = np.pi/3
    np.array([0., 0., angle, 0., 0., 0.])
    joint_angles = np.array([angle, angle, angle, angle, 2.5, 0.])
    #joint_angles = np.array([0., 0., angle, 0., 0., 0.])

    Ts = UR5ZeroAngleTransformations().transformations

    transformed_origins = []
    origin = np.array([0., 0., 0.])

    base_vertices = []
    base_to_mesh_1 = Transformation.identity()  # multiply(Ts[:0])
    for triangle in base_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_1.transform_point(point)
            base_vertices.append(transformed_point)
    transformed_origins.append(base_to_mesh_1.transform_point(origin))

    link_2_vertices = []
    base_to_mesh_2 = ur5_kinematic_chain.forward_kinematics_base_to_nth_joint(joint_angles[:1])
    for triangle in link_2_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_2.transform_point(point)
            link_2_vertices.append(transformed_point)
    transformed_origins.append(base_to_mesh_2.transform_point(origin))

    link_3_vertices = []
    base_to_mesh_3 = ur5_kinematic_chain.forward_kinematics_base_to_nth_joint(joint_angles[:2])
    for triangle in link_3_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_3.transform_point(point)
            link_3_vertices.append(transformed_point)
    transformed_origins.append(base_to_mesh_3.transform_point(origin))

    link_4_vertices = []
    base_to_mesh_4 = ur5_kinematic_chain.forward_kinematics_base_to_nth_joint(joint_angles[:3])
    for triangle in link_4_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_4.transform_point(point)
            link_4_vertices.append(transformed_point)
    transformed_origins.append(base_to_mesh_4.transform_point(origin))

    link_5_vertices = []
    base_to_mesh_5 = ur5_kinematic_chain.forward_kinematics_base_to_nth_joint(joint_angles[:4])
    for triangle in link_5_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_5.transform_point(point)
            link_5_vertices.append(transformed_point)
    transformed_origins.append(base_to_mesh_5.transform_point(origin))

    link_6_vertices = []
    base_to_mesh_6 = ur5_kinematic_chain.forward_kinematics_base_to_nth_joint(joint_angles[:5])
    for triangle in link_6_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_6.transform_point(point)
            link_6_vertices.append(transformed_point)
    transformed_origins.append(base_to_mesh_6.transform_point(origin))

    link_7_vertices = []
    base_to_mesh_7 = ur5_kinematic_chain.forward_kinematics_base_to_nth_joint(joint_angles)
    for triangle in link_7_mesh.triangles:
        for point in triangle:
            transformed_point = base_to_mesh_7.transform_point(point)
            link_7_vertices.append(transformed_point)
    transformed_origins.append(base_to_mesh_7.transform_point(origin))

    robot_vertices = np.array(base_vertices +
                              link_2_vertices +
                              link_3_vertices +
                              link_4_vertices +
                              link_5_vertices +
                              link_6_vertices +
                              link_7_vertices
                              )
    robot_mesh_triangles = o3d.utility.Vector3dVector(robot_vertices)
    breakpoint()
    transformed_origins = o3d.utility.Vector3dVector(transformed_origins)
    visualize_mesh(robot_mesh_triangles, transformed_origins)
