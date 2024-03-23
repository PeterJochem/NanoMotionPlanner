import numpy as np
from geometry.primitives.transformation import Transformation


class Mesh:
    """
    Represents a surface in 3D space by decomposing it into a set of triangles.
    """

    def __init__(self, triangles: np.ndarray):
        """Constructor.

        Args:
            triangles: numpy.ndarray
                Dimension: Nx3x3, where N is the number of triangles.
        """
        self.triangles = triangles

    def transformed_mesh(self, transformation: Transformation) -> "Mesh":
        """Transforms the mesh's vertices to be measured in a different frame of reference.

        Args:
            transformation: Transformation

        Returns:
            Mesh:
                The same mesh but its triangles are measured in a different frame of reference.
        """

        transformed_triangles = np.copy(self.triangles)
        for i in range(len(self.triangles)):
            transformed_triangles[i] = transformation.transform_point(self.triangles[i])
        return Mesh(transformed_triangles)
