from typing import List
import numpy as np
import os
from geometry.kinematic_chain import KinematicOpenChain
from geometry.mesh import Mesh
from geometry.primitives.screw import Screw
from geometry.primitives.transformation import Transformation
from geometry.utilities import multiply


def mm_to_meters(n: float) -> float:
    """Converts a length in millimeters to meters.

    Args:
        n: float
            Length in millimeters to meters.

    Returns:
        float:
            The provided length measured in meters.
    """
    return n / 1000.


class UR5ZeroAngleTransformations:

    def __init__(self):
        # Define the Transformation from one joint to the next.
        self.root_to_base_link_transformation = Transformation.construct(1.0, 0., 0., 0., 0., 0.)

        delta = 0.0025
        self.base_to_shoulder_link_transformation = Transformation.construct(0., 0., delta * 5, 0., 0., 0.)
        self.shoulder_upper_arm_link_transformation = Transformation.construct(0., delta, 0., 0., 0., 0.)
        self.upper_arm_forearm_link_transformation = Transformation.construct(0., -delta, 0., 0., 0., 0.)
        self.forearm_wrist_1_link_transformation = Transformation.construct(0., delta, 0., 0., 0., 0.)
        self.wrist_1_wrist_2_link_transformation = Transformation.construct(0., 0., delta, 0., 0., 0.)

        self.wrist_2_wrist_3_link_transformation = Transformation.construct(0., delta, 0., 0., 0., 0.)
        self.wrist_3_end_effector_link_transformation = Transformation.construct(0., 0., 0., np.pi/2, 0., 0.)

        self.root_to_base_joint_transformation = Transformation.identity()

        self.base_to_shoulder_joint_transformation = Transformation.construct(0., 0., 0.089159, 0., 0., 0.)
        self.shoulder_upper_arm_joint_transformation = Transformation.construct(0., 0.13585, 0., 0., np.pi / 2, 0.)
        self.upper_arm_forearm_joint_transformation = Transformation.construct(0., -0.1197, 0.425, 0., 0., 0.)
        self.forearm_wrist_1_joint_transformation = Transformation.construct(0., 0., 0.39225, 0., np.pi / 2., 0.)
        self.wrist_1_wrist_2_joint_transformation = Transformation.construct(0., 0.093, 0., 0., 0., 0.)
        self.wrist_2_wrist_3_joint_transformation = Transformation.construct(0., 0., 0.09465, 0., 0., 0.)

        #       1           2            3          4          5!!       6
        # Base -> shoulder -> upper arm -> forearm -> wrist_1 -> wrist_2 -> wrist_3 (contains end effector)

        links = [ # Transformation.identity(),
                 self.base_to_shoulder_link_transformation,
                 self.shoulder_upper_arm_link_transformation,
                 self.upper_arm_forearm_link_transformation,
                 self.forearm_wrist_1_link_transformation,
                 self.wrist_1_wrist_2_link_transformation,
                 self.wrist_2_wrist_3_link_transformation]

        joints = [# self.root_to_base_joint_transformation,
                  self.base_to_shoulder_joint_transformation,
                  self.shoulder_upper_arm_joint_transformation,
                  self.upper_arm_forearm_joint_transformation,
                  self.forearm_wrist_1_joint_transformation,
                  self.wrist_1_wrist_2_joint_transformation,
                  self.wrist_2_wrist_3_joint_transformation]

        self.transformations = []
        for link_transformation, joint_transformation in zip(links, joints):
            self.transformations = self.transformations + [link_transformation, joint_transformation]
            #self.transformations = self.transformations + [joint_transformation, link_transformation]
            #self.transformations += [joint_transformation, link_transformation]
        self.transformations = self.transformations + [self.wrist_3_end_effector_link_transformation]


class URDimensions:

    def __init__(self):
        """Constructor."""
        self.W1 = mm_to_meters(109.)
        self.W2 = mm_to_meters(82.)
        self.L1 = mm_to_meters(425.)
        self.L2 = mm_to_meters(392.)
        self.H1 = mm_to_meters(89.)
        self.H2 = mm_to_meters(95.)
        self.zero_angle_transformations = UR5ZeroAngleTransformations()


class UR5Meshes:

    path_to_file = os.path.dirname(os.path.abspath(__file__))
    meshes_path = f"{path_to_file}/meshes/ur5/collision/"
    meshes_path = f"{path_to_file}/meshes/ur5/collision/decimated"

    def __init__(self,
                 base: str = f"{meshes_path}/base.stl",
                 shoulder: str = f"{meshes_path}/shoulder.stl",
                 upper_arm: str = f"{meshes_path}/upperarm.stl",
                 forearm: str = f"{meshes_path}/forearm.stl",
                 wrist1: str = f"{meshes_path}/wrist1.stl",
                 wrist2: str = f"{meshes_path}/wrist2.stl",
                 wrist3: str = f"{meshes_path}/wrist3.stl"):
        """Constructor.

            Args:
                base: str
                shoulder: str
                upper_arm: str
                forearm: str
                wrist1: str
                wrist2: str
                wrist3: str
        """
        self.base_path = base
        self.shoulder_path = shoulder
        self.upper_arm_path = upper_arm
        self.forearm_path = forearm
        self.wrist_1_path = wrist1
        self.wrist_2_path = wrist2
        self.wrist_3_path = wrist3

        self.base = Mesh.from_file(self.base_path)
        self.shoulder = Mesh.from_file(self.shoulder_path)
        self.upper_arm = Mesh.from_file(self.upper_arm_path)
        self.forearm = Mesh.from_file(self.forearm_path)
        self.wrist_1 = Mesh.from_file(self.wrist_1_path)
        self.wrist_2 = Mesh.from_file(self.wrist_2_path)
        self.wrist_3 = Mesh.from_file(self.wrist_3_path)

        self.ordered_meshes = [self.base, self.shoulder, self.upper_arm, self.forearm, self.wrist_1, self.wrist_2,
                               self.wrist_3]


def define_ur5_home_transformation() -> Transformation:
    """Defines the transformation from the base to the end effector when a UR5 robot has all zero joint angles.

    Notes:
        See page 147 of Modern Robotics.

    Returns:
        Transformation:
            T_base_ee when a UR5 has all its joints at zero degrees.
    """
    dimensions = URDimensions()

    matrix = np.zeros((4, 4), dtype=float)

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


def define_ur5_base_to_end_effector_transformations() -> List[Transformation]:
    """Defines all the transformations from the base to the end effector when a UR5 robot has all zero joint angles.

    Notes:
        See page 147 of Modern Robotics.

    Returns:
        List[Transformation]
            All the transformations from the base to the end effector when a UR5 robot has all zero joint angles.
    """
    a = URDimensions().zero_angle_transformations.transformations
    b = multiply(a)
    c = define_ur5_home_transformation()
    #breakpoint()
    return a


def define_ur5_kinematic_chain() -> KinematicOpenChain:
    """Defines a kinematic chain for the UR5 robot.

    Notes:
        See page 146 of Modern Robotics.

    Returns:
        KinematicOpenChain:
            The kinematic chain for the UR5 robot.
    """
    dimensions = URDimensions()
    h = 0.

    q = np.array([0., 0., 0.091659])
    s = np.array([0., 0., 1.])
    screw_1 = Screw.construct_from_q_s_h(q, s, h)

    q = np.array([0., 0.13835, 0.091659])
    s = np.array([0., 1., 0.])
    screw_2 = Screw.construct_from_q_s_h(q, s, h)

    q = np.array([-0.425, 0.01615, 0.091659])
    s = np.array([0., 1., 0.])
    screw_3 = Screw.construct_from_q_s_h(q, s, h)

    q = np.array([-0.81725, 0.01865, 0.091659])
    s = np.array([0., 1., 0.])
    screw_4 = Screw.construct_from_q_s_h(q, s, h)

    q = np.array([-0.81725, 0.11165, 0.089159])
    s = np.array([0., 0., 1.])
    screw_5 = Screw.construct_from_q_s_h(q, s, h)

    q = np.array([-0.81725, 0.11415, -0.005491])
    s = np.array([0., 1., 0.])
    screw_6 = Screw.construct_from_q_s_h(q, s, h)

    screws = [screw_1, screw_2, screw_3, screw_4, screw_5, screw_6]
    return KinematicOpenChain(screws, define_ur5_base_to_end_effector_transformations())
