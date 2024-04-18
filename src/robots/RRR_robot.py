from typing import List
import numpy as np
from geometry.kinematic_chain import KinematicOpenChain
from geometry.mesh import Mesh
from geometry.primitives.screw import Screw
from geometry.primitives.transformation import Transformation
from robots.robot import Robot

LINK_1_LENGTH = 1.
LINK_2_LENGTH = 1.
LINK_3_LENGTH = 1.


def define_RRR_robot_kinematic_chain() -> KinematicOpenChain:
    """Defines the 3xRevolute robot.

    Returns:
        KinematicOpenChain:
            Represents the kinematics of the 3xRevolute robot.
    """

    z_axis = np.array([0., 0., 1.])
    origin = np.zeros(3)
    h = 0.

    joint_one_screw = Screw.construct_from_q_s_h(origin, z_axis, h)
    joint_two_screw = Screw.construct_from_q_s_h(origin + np.array([LINK_1_LENGTH, 0., 0.]), z_axis, h)
    joint_three_screw = Screw.construct_from_q_s_h(origin + np.array([LINK_2_LENGTH, 0., 0.]), h)

    link_one_to_two_transformation = Transformation.construct()
    link_two_to_three_transformation = Transformation.construct()
    link_three_to_end_effector = Transformation.construct()

    screws = [joint_one_screw, joint_two_screw, joint_three_screw]
    transformations = [link_one_to_two_transformation, link_two_to_three_transformation, link_three_to_end_effector]

    return KinematicOpenChain(screws, transformations)


def define_RRR_robot_meshes() -> List[Mesh]:
    """Defines the 3xRevolute joint robot meshes.

    Returns:
        List[Mesh]:
            The meshes for the 3xRevolute joint robot.
    """

    y_delta = 0.25
    link_1_triangle = np.zeros((3, 3))
    link_2_triangle = np.zeros((3, 3))
    link_3_triangle = np.zeros((3, 3))

    link_1_triangle[0] = np.array([0., y_delta, 0.])
    link_1_triangle[1] = np.array([0., -y_delta, 0.])
    link_1_triangle[2] = np.array([LINK_1_LENGTH, 0., 0.])

    link_2_triangle[0] = np.array([0., y_delta, 0.])
    link_2_triangle[1] = np.array([0., -y_delta, 0.])
    link_2_triangle[2] = np.array([LINK_2_LENGTH, 0., 0.])

    link_3_triangle[0] = np.array([0., y_delta, 0.])
    link_3_triangle[1] = np.array([0., -y_delta, 0.])
    link_3_triangle[2] = np.array([LINK_3_LENGTH, 0., 0.])

    return [Mesh(link_1_triangle), Mesh(link_2_triangle), Mesh(link_2_triangle)]


class PlanarRRRRobot(Robot):
    """
    A very simple, 3xRevolute joint robot for testing purposes.
    """

    def __init__(self):
        """Constructor."""

        super().__init__(define_RRR_robot_kinematic_chain(), define_RRR_robot_meshes())

    def is_legal(self, joint_angles: np.ndarray) -> bool:
        """Checks if the provided joint angles are free of self collisions.

        Args:
            joint_angles: numpy.ndarray

        Returns:
            bool:
                True iff the provided joint angles are free of self collisions.
        """
        return True
