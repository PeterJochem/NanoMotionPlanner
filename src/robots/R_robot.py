from typing import List
import numpy as np
from geometry.kinematic_chain import KinematicOpenChain
from geometry.mesh import Mesh
from geometry.primitives.screw import Screw
from geometry.primitives.transformation import Transformation
from robots.robot import Robot


def define_r_robot_kinematic_chain(link_length: float) -> KinematicOpenChain:
    """Defines a robot with one joint which rotates about the z-axis.

    Args:
        link_length: float
            Length of the robot's only link in meters.

    Returns:
        KinematicOpenChain:
            Represents the open chain of a single revolute joint robot.
    """

    z_axis = np.array([0., 0., 1.])
    origin = np.zeros(3)
    h = 0.

    joint_one_screw = Screw.construct_from_q_s_h(origin, z_axis, h)
    link_one_to_end_effector_transformation = Transformation.construct(link_length, 0., 0., 0., 0., 0.)

    screws = [joint_one_screw]
    transformations = [link_one_to_end_effector_transformation]

    return KinematicOpenChain(screws, transformations)


def define_r_robot_mesh(link_length: float) -> List[Mesh]:
    """Defines the mesh for a single revolute joint robot.

    Returns:
        List[Mesh]:
            The mesh for a single revolute joint robot.
    """

    y_delta = 0.25
    link_1_triangle = np.zeros((3, 3))

    link_1_triangle[0] = np.array([0., y_delta, 0.])
    link_1_triangle[1] = np.array([0., -y_delta, 0.])
    link_1_triangle[2] = np.array([link_length, 0., 0.])

    return [Mesh(link_1_triangle)]


class PlanarRRobot(Robot):
    """
    A very simple, single revolute joint robot for testing purposes.
    """

    def __init__(self, link_length: float = 1.):
        """Constructor.

        Args:
            link_length: float
                The length of the single link in meters.
        """

        super().__init__(define_r_robot_kinematic_chain(link_length), define_r_robot_mesh(link_length))
