import pytest
import numpy as np
from geometry.kinematic_chain import KinematicOpenChain
from geometry.primitives.screw import Screw
from geometry.primitives.transformation import Transformation


def mm_to_meters(n: float) -> float:
    return n / 1000.


class URDimensions:

    def __init__(self):
        """Constructor."""
        self.W1 = mm_to_meters(109.)
        self.W2 = mm_to_meters(82.)
        self.L1 = mm_to_meters(425.)
        self.L2 = mm_to_meters(392.)
        self.H1 = mm_to_meters(89.)
        self.H2 = mm_to_meters(95.)


def define_ur5_home_transformation() -> Transformation:
    """See page 147 of Modern Robotics."""
    dimensions = URDimensions()
    matrix = np.zeros(4)
    # Set the rotation components.
    matrix[0][0] = -1.
    matrix[1][2] = 1.
    matrix[2][1] = 1.

    # Set the translation components.
    matrix[0][3] = dimensions.L1 + dimensions.L2
    matrix[1][3] = dimensions.W1 + dimensions.W2
    matrix[2][3] = dimensions.H1 - dimensions.H2

    # Homogenous coordinate.
    matrix[3][3] = 1.

    return Transformation(matrix)


def define_ur5_kinematic_chain() -> KinematicOpenChain:
    """See page 146 of Modern Robotics."""
    dimensions = URDimensions()

    w1 = np.array([0., 0., 1.])
    v1 = np.array([0., 0., 0.])

    w2 = np.array([0., 1., 0.])
    v2 = np.array([-dimensions.H1, 0., 0.])

    w3 = np.array([0., 1., 0.])
    v3 = np.array([-dimensions.H1, 0., dimensions.L1])

    w4 = np.array([0., 1., 0.])
    v4 = np.array([-dimensions.H1, 0., dimensions.L1 + dimensions.L2])

    w5 = np.array([0., 0., -1.])
    v5 = np.array([-dimensions.W1, dimensions.L1 + dimensions.L2, 0.])

    w6 = np.array([0., 1., 0.])
    v6 = np.array([dimensions.H2 - dimensions.H1, 0., dimensions.L1 + dimensions.L2])

    screw1 = Screw(w1, v1)
    screw2 = Screw(w2, v2)
    screw3 = Screw(w3, v3)
    screw4 = Screw(w4, v4)
    screw5 = Screw(w5, v5)
    screw6 = Screw(w6, v6)
    screws = [screw1, screw2, screw3, screw4, screw5, screw6]
    return KinematicOpenChain(screws)





# See page 147 of Modern Robotics.
ur5_kinematic_chain = define_ur5_kinematic_chain()

joint_angles_1 = np.ndarray([0., -np.pi / 2., 0., 0., np.pi / 2., 0.])
transformation_1 = Transformation()

ur5_case_1 = (ur5_kinematic_chain, joint_angles_1, transformation_1)


@pytest.mark.parametrize("chain, joint_angles, expected_transformation", [ur5_case_1, ])
def test_forward_kinematics():
    assert 1 == 2
