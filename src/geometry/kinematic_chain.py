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

    def __init__(self, screws: List[Screw], zero_angle_transformations: List[Transformation]):
        """Constructor.

        Args:
            screws: List[Screw]
                The screw axes for each joint in the kinematic chain.
            zero_angle_transformations: List[Transformation]
                The transformations from each frame on the robot ordered from the base to the end effector when all
                the joint angles are zero.
        """
        self.screws = screws
        self.zero_angle_transformations = zero_angle_transformations
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

        transformations = self.transformations(joint_angles) + self.zero_angle_transformations
        return multiply(transformations)

    def forward_kinematics_base_to_nth_joint(self, joint_angles: np.ndarray) -> Transformation:
        """Computes the transformation of the nth joint in the base frame at the provided joint angles.

        Args:
            joint_angles: numpy.ndarray

        Returns:
            Transformation:
                Transformation in the base frame of the Nth joint at the provided joint angles.
        """

        m = len(joint_angles)
        n = (2 * m)
        via_rotation = self.transformations(joint_angles)
        base_to_joint_home_pose = self.zero_angle_transformations[:n]
        return multiply(via_rotation + base_to_joint_home_pose)

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

    def inverse_kinematics(self,
                           desired_base_to_end_effector_pose: np.ndarray,
                           termination_error_magnitude: float = 1.5e-1,
                           max_num_iterations: float = 1000,
                           step_size: float = 1e-1) -> np.ndarray:
        """Computes a set of joint angles which makes the end effector have desired transformation in the base frame.

        Args:
            desired_base_to_end_effector_pose: numpy.ndarray
            termination_error_magnitude: float
            max_num_iterations: float
            step_size: float

        Returns:
            numpy.ndarray:
                The joint angles which put the end effector in at the provided transformation in the base frame.
        """

        history = []
        current_joint_angles = np.zeros(self.num_joints)
        current_pose = self.end_effector_pose(current_joint_angles)
        current_error = np.linalg.norm(current_pose - desired_base_to_end_effector_pose)

        current_iteration = 0

        while current_error > termination_error_magnitude:

            history.append(current_error)

            if current_iteration > max_num_iterations:
                breakpoint()
                raise RuntimeError("Unable to solve the inverse kinematics problem.")

            pose_delta = desired_base_to_end_effector_pose - current_pose
            delta_theta_direction = self.jacobian(current_joint_angles).T @ pose_delta
            delta_theta = step_size * current_error * delta_theta_direction / np.linalg.norm(delta_theta_direction)
            current_joint_angles += delta_theta

            current_pose = self.end_effector_pose(current_joint_angles)
            current_error = np.linalg.norm(current_pose - desired_base_to_end_effector_pose)
            current_iteration += 1

        return current_joint_angles

    def inverse_kinematics_from_transformation(self,
                                               desired_base_to_ee: Transformation,
                                               termination_error_magnitude: float = 1.5e-1,
                                               max_num_iterations: float = 1000,
                                               step_size: float = 1e-1) -> np.ndarray:
        """Computes a set of joint angles which makes the end effector have desired transformation in the base frame.

        Args:
            desired_base_to_ee: Transformation
            termination_error_magnitude: float
            max_num_iterations: float
            step_size: float

        Returns:
            numpy.ndarray:
                The joint angles which put the end effector in at the provided transformation in the base frame.
        """

        desired_pose = pose(desired_base_to_ee)
        return self.inverse_kinematics(desired_pose, termination_error_magnitude, max_num_iterations, step_size)

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
