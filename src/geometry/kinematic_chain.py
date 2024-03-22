from typing import List
import numpy as np
from geometry.primitives.screw import Screw
from geometry.primitives.transformation import Transformation
from geometry.utilities import multiply, pose
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

    def end_effector_pose(self, joint_angles: np.ndarray) -> np.ndarray:
        """Computes the pose of the end effector frame in the base frame at the provided joint angles.

        Args:
            joint_angles: numpy.ndarray

        Returns:
            numpy.ndarray:
                The pose of the end effector frame in the base frame at the provided joint angles.
        """

        base_to_ee = self.forward_kinematics(joint_angles)
        return pose(base_to_ee)

    def inverse_kinematics(self, desired_base_to_ee: Transformation) -> np.ndarray:
        """..."""
        print(desired_base_to_ee)
        termination_error_magnitude = 1e-5
        max_num_iterations = 1000  # 10000
        step_size = 1e-1

        current_iteration = 0

        # current_joint_angles = np.zeros(self.num_joints)
        current_joint_angles = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        current_pose = self.end_effector_pose(current_joint_angles)
        desired_pose = pose(desired_base_to_ee)

        current_error = np.linalg.norm(current_pose - desired_pose)
        history = []

        while current_error > termination_error_magnitude:

            if current_iteration > max_num_iterations:
                a = np.round(current_pose, 2)
                b = np.round(desired_pose, 2)
                return current_joint_angles
                #raise RuntimeError("Unable to solve the inverse kinematics problem.")

            delta_theta_direction = self.jacobian(current_joint_angles).T @ (desired_pose - current_pose)
            delta_theta = step_size * current_error * delta_theta_direction / np.linalg.norm(delta_theta_direction)
            current_joint_angles += delta_theta

            # breakpoint()
            # history.append(current_joint_angles)
            history.append(current_error)

            current_pose = self.end_effector_pose(current_joint_angles)
            current_error = np.linalg.norm(current_pose - desired_pose)
            current_iteration += 1


        return current_joint_angles


    def jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        """Computes the jacobian at the provided joint angles.

        Notes:
            Modern Robotics page 150.

        Args:
            joint_angles: numpy.ndarray
                The robot's joint angles where the jacobian is evaluated.

        Returns:
            numpy.ndarray:
                The jacobian matrix expressed in the robot's base frame.
        """

        num_rows = self.num_joints
        num_columns = 6
        matrix = np.zeros((num_rows, num_columns))

        for i in range(num_rows):
            forward_kinematics = lambda x: pose(self.forward_kinematics(x))
            matrix[i] = gradient(forward_kinematics, joint_angles, i, 1e-8)

        return matrix
