from typing import List
import pytest
import numpy as np
from geometry.collision_detection.kinematic_chain_mesh_collision_detector import KinematicChainMeshCollisionDetector
from geometry.kinematic_chain import KinematicOpenChain
from geometry.mesh import Mesh
from robots.ur5 import define_ur5_kinematic_chain

kinematic_chain_mesh_collision_test_cases = []
ur5_kinematic_chain = define_ur5_kinematic_chain()


@pytest.mark.parametrize("kinematic_chain, meshes, joint_angles, expected", kinematic_chain_mesh_collision_test_cases)
def test_kinematic_chain_mesh_collision_detector(kinematic_chain: KinematicOpenChain,
                                                 meshes: List[Mesh],
                                                 joint_angles: np.ndarray,
                                                 expected: bool):

    detector = KinematicChainMeshCollisionDetector(kinematic_chain, meshes)
    assert detector.detect(joint_angles) == expected
