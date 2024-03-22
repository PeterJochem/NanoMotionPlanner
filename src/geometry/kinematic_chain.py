from typing import List
import numpy as np
from geometry.primitives.screw import Screw
from geometry.primitives.transformation import Transformation
from geometry.utilities import multiply
from numerical_algorithms.utilities import gradient


class KinematicOpenChain:
    """
    Represents a kinematic open chain, a series of links joined together.
    """

    def __init__(self, screws: List[Screw], zero_angle_transformation: Transformation):
        """Constructor.

        Args:
            screws: List[Screw]
                The screw axes for each joint in the kinematic chain.
            zero_angle_transformation: Transformation
                The transformation from the robot's base to the end effector when all the joint angles are zero.
        """
        self.screws = screws
        self.zero_angle_transformation = zero_angle_transformation
        self.num_joints = len(screws)

    def transformations(self, joint_angles: np.ndarray) -> List[Transformation]:
        """Computes the transformation from the base to the end effector at the provided joint angles.

        Args:
            joint_angles: numpy.ndarray

        Returns:
            List[Transformation]:
                The transformation for each joint in the open chain.
        """
        return [screw.transformation(theta) for screw, theta in zip(self.screws, joint_angles)]

    def forward_kinematics(self, joint_angles: np.ndarray) -> Transformation:
        """Computes the transformation from the base link to the end effector link.

        Args:
            joint_angles: numpy.ndarray

        Returns:
            Transformation:
                The transformation from the base link to the end effector link.
        """

        transformations = self.transformations(joint_angles) + [self.zero_angle_transformation]
        return multiply(transformations)

    def inverse_kinematics(self, desired_base_to_ee: Transformation) -> np.ndarray:
        """..."""
        raise RuntimeError("Not implemented yet.")

    def jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        """Computes the Jacobian at the provided joint angles.

        Notes:
            Modern Robotics page 150.

        Args:
            joint_angles: numpy.ndarray
                The robot's joint angles where the jacobian is evaluated.

        Returns:
            numpy.ndarray:
                The Jacobian matrix expressed in the robot's base frame.
        """

        num_rows = self.num_links
        num_columns = 6
        jacobian = np.zeros((num_rows, num_columns))

        for i in range(num_rows):
            jacobian[i] = gradient(self.forward_kinematics, joint_angles, i)

        return jacobian
