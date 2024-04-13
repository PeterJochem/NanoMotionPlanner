import pytest
import numpy as np
from geometry.kinematic_chain import KinematicOpenChain
from geometry.primitives.transformation import Transformation
from geometry.utilities import multiply
from robots.ur5 import define_ur5_kinematic_chain, define_ur5_home_transformation, \
    define_ur5_base_to_end_effector_transformations

# See page 147 of Modern Robotics.
ur5_kinematic_chain = define_ur5_kinematic_chain()

joint_angles_1 = np.array([0., -np.pi / 2., 0., 0., np.pi / 2., 0.])
joint_angles_2 = np.array([0., 0., 0., 0., 0., 0.])


transformation_matrix_1 = np.array([[0., -1., 0., 9.4491e-02],
                                    [1., 0., 0., 1.74325],
                                    [0., 0., 1., 9.8845e-01],
                                    [0., 0., 0., 1.]])

transformation_1 = Transformation(transformation_matrix_1)
ur5_case_1 = (ur5_kinematic_chain, joint_angles_1, transformation_1)
ur5_case_2 = (ur5_kinematic_chain, joint_angles_2, multiply(define_ur5_base_to_end_effector_transformations()))


@pytest.mark.parametrize("kinematic_chain, joint_angles, expected_transformation", [ur5_case_2, ur5_case_2])
def test_forward_kinematics(kinematic_chain: KinematicOpenChain,
                            joint_angles: np.ndarray,
                            expected_transformation: Transformation):

    transformation = kinematic_chain.forward_kinematics(joint_angles)
    assert np.allclose(transformation.matrix, expected_transformation.matrix, atol=1e-5)


def test_inverse_kinematics_from_transformation():

    ee_transformation_at_original_joint_angles = ur5_kinematic_chain.forward_kinematics(joint_angles_1)
    computed_joint_angles = ur5_kinematic_chain.inverse_kinematics_from_transformation(ee_transformation_at_original_joint_angles)
    ee_transformation_at_new_joint_angles = ur5_kinematic_chain.forward_kinematics(computed_joint_angles)

    assert np.allclose(ee_transformation_at_original_joint_angles.matrix,
                       ee_transformation_at_new_joint_angles.matrix, atol=2e-1)


