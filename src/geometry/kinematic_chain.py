from typing import List
import numpy as np
from geometry.primitives.transformation import Transformation


class KinematicOpenChain:
    """
    Represents a kinematic open chain, a series of links joined together.
    """

    def __init__(self, transformations: List[Transformation]):
        """Constructor.

        Args:
            transformations: List[Transformation]
                The transformations from each link to its parent.
        """
        self.transformations = transformations
        self.num_links = len(transformations)

    def forward_kinematics(self, home_position: Transformation) -> Transformation:
        """Computed the transformation from the base link to the end effector link.

        Args:
            home_position: Transformation
                The pose of the end effector when all the joint angles are zero.

        Returns:
            Transformation:
                The transformation from the base link to the end effector link.
        """

        left = Transformation.identity()
        for transformation in self.transformations:
            left = left * transformation
        return left * home_position

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

        # Compute the forward kinematics at these joint angles.
        # Alter each joint angle slightly to compute the derivative.
        # Need to be able to extract the (roll, pitch, yaw) from the transformation.

        for row_index in range(num_rows):


        return jacobian
