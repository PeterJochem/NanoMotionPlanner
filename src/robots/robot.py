from typing import List
import numpy as np
from abc import ABC
from geometry.collision_detection.kinematic_chain_mesh_collision_detector import KinematicChainMeshCollisionDetector
from geometry.kinematic_chain import KinematicOpenChain
from geometry.mesh import Mesh


class Robot(ABC):
    """
    Base class for representing a robot's physical attributes.
    """

    def __init__(self, kinematic_chain: KinematicOpenChain, meshes: List[Mesh]):
        self.kinematic_chain = kinematic_chain
        self.meshes = meshes

    def is_legal(self, joint_angles: np.ndarray) -> bool:
        """Checks if the provided joint angles are free of self collisions.

        Args:
            joint_angles: numpy.ndarray

        Returns:
            bool:
                True iff the provided joint angles are free of self collisions.
        """
        detector = KinematicChainMeshCollisionDetector(self.kinematic_chain, self.meshes)
        return not detector.detect(joint_angles)
