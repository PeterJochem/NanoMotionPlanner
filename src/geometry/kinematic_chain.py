from typing import List
import numpy as np
from geometry.primitives.screw import Screw
from geometry.primitives.transformation import Transformation
from geometry.utilities import multiply


class KinematicOpenChain:
    """
    Represents a kinematic open chain, a series of links joined together.
    """

    def __init__(self, screws: List[Screw]):
        """Constructor.

        Args:
            screws: List[Screw]
                The screw axes for each joint in the kinematic chain.
        """
        self.screws = screws
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

    def forward_kinematics(self, joint_angles: np.ndarray, home_position: Transformation) -> Transformation:
        """Computes the transformation from the base link to the end effector link.

        Args:
            joint_angles: numpy.ndarray
            home_position: Transformation
                The pose of the end effector when all the joint angles are zero.

        Returns:
            Transformation:
                The transformation from the base link to the end effector link.
        """

        transformations = self.transformations(joint_angles) + [home_position]
        return multiply(transformations)

    def inverse_kinematics(self, desired_base_to_ee: Transformation) -> np.ndarray:
        """..."""
        raise RuntimeError("Not implemented yet.")

    def jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        """...

        Notes:
            Modern Robotics page 150.

        Args:
            joint_angles: numpy.ndarray
                ...

        Returns:
            numpy.ndarray:
                The jacobian matrix expressed in the robot's base frame.
        """

        num_rows = self.num_links
        num_columns = 6
        jacobian = np.zeros((num_rows, num_columns))

        base_to_ee = ...

        # Compute the forward kinematics at these joint angles.
        # Alter each joint angle slightly to compute the derivative.
        # Need to be able to extract the (roll, pitch, yaw) from the transformation.
        for row_index in range(num_rows):
            ...


        return jacobian
