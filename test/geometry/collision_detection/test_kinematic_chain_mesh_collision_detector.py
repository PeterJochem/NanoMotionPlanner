from typing import List
import pytest
import numpy as np
import open3d as o3d
from geometry.collision_detection.kinematic_chain_mesh_collision_detector import KinematicChainMeshCollisionDetector
from geometry.collision_detection.my_visualize import visualize_mesh
from geometry.kinematic_chain import KinematicOpenChain
from geometry.mesh import Mesh
from geometry.primitives.transformation import Transformation
from robots.ur5 import define_ur5_kinematic_chain, UR5Meshes, UR5ZeroAngleTransformations


angle = np.pi / 1.1


@pytest.mark.skip(reason="Requires Open3D")
@pytest.mark.parametrize("joint_angles", [np.array([0., 0., 0., 0., 0., 0.])])
def test_dummy(joint_angles: np.ndarray):

    for i in range(1):

        joint_angles[2] = joint_angles[2] + (0.2 * i)

        meshes = ur5_meshes
        ur5_kinematic_chain = define_ur5_kinematic_chain()

        base_mesh = meshes[0]
        link_2_mesh = meshes[1]
        link_3_mesh = meshes[2]
        link_4_mesh = meshes[3]
        link_5_mesh = meshes[4]
        link_6_mesh = meshes[5]
        link_7_mesh = meshes[6]

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
        transformed_origins = o3d.utility.Vector3dVector(transformed_origins)
        visualize_mesh(robot_mesh_triangles, transformed_origins)


ur5_kinematic_chain = define_ur5_kinematic_chain()
ur5_meshes = UR5Meshes().ordered_meshes

test_case_1 = (ur5_kinematic_chain, ur5_meshes, np.zeros(6), False)
test_case_2 = (ur5_kinematic_chain, ur5_meshes, np.array([0., np.pi / 1.1, np.pi / 1.1, np.pi / 1.1, np.pi / 1.1, 0.]),
               True)

test_cases = [test_case_1, test_case_2]


#@pytest.mark.skip(reason="Requires Open3D")
@pytest.mark.parametrize("kinematic_chain, meshes, joint_angles, expected", test_cases)
def test_mesh_mesh_collision_detector(kinematic_chain: KinematicOpenChain,
                                      meshes: List[Mesh],
                                      joint_angles: np.ndarray,
                                      expected: bool):

    detector = KinematicChainMeshCollisionDetector(kinematic_chain, meshes)
    assert detector.detect(joint_angles) == expected
