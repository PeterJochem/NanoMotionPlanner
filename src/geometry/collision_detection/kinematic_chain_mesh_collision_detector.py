from typing import List
import numpy as np
from geometry.collision_detection.mesh_mesh_collision_detector import MeshMeshCollisionDetector
from geometry.collision_detection.my_visualize import visualize_two_meshes
from geometry.kinematic_chain import KinematicOpenChain
from geometry.mesh import Mesh


class KinematicChainMeshCollisionDetector:
    """
    Detects collisions between the links of a kinematic chain.
    """

    def __init__(self, kinematic_chain: KinematicOpenChain, meshes: List[Mesh]):
        """Constructor.

        Args:
            kinematic_chain: KinematicOpenChain
                A series of link and joint representing a robots kinematics.
            meshes: List[Mesh]
                Meshes of the chain's links. Follows the order of the chain's links.
        """
        self.kinematic_chain = kinematic_chain
        self.meshes = meshes

    def detect(self, joint_angles: np.ndarray) -> bool:
        """Detects collisions between the robot's links.

        Args:
            joint_angles: numpy.ndarray

        Returns:
            bool:
                True iff there is at least one collision between the meshes of the kinematic chain.
        """

        n = len(self.meshes)
        transformed_meshes = [self.meshes[0]]

        for i in range(1, n):
            base_to_joint_i = self.kinematic_chain.forward_kinematics_base_to_nth_joint(joint_angles[:i])
            transformed_mesh = self.meshes[i].transformed_mesh(base_to_joint_i)
            transformed_meshes.append(transformed_mesh)

        mesh_idx_pairs = []
        for i in range(n):
            for j in range(i + 1, n):
                mesh_idx_pairs.append((i, j))

        for mesh_1_idx, mesh_2_idx in mesh_idx_pairs:
            detector = MeshMeshCollisionDetector(transformed_meshes[mesh_1_idx], transformed_meshes[mesh_2_idx])
            if detector.detect():
                visualize_two_meshes(transformed_meshes[mesh_1_idx].triangles,
                                     transformed_meshes[mesh_2_idx].triangles)
                return True

        return False
