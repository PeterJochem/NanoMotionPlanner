from geometry.collision_detection.triangle_triangle_collision_detector import TriangleTriangleCollisionDetector
from geometry.mesh import Mesh


class MeshMeshCollisionDetector:
    """
    Detects collisions between two meshes.
    """
    def __init__(self, mesh_1: Mesh, mesh_2: Mesh):
        """Constructor.

        Args:
            mesh_1: Mesh
            mesh_2: Mesh
        """
        self.mesh_1 = mesh_1
        self.mesh_2 = mesh_2

    def detect(self) -> bool:
        """Detects if there is a collision between the two meshes.

        Returns:
            bool:
                True iff there is a collision between the two meshes.
        """

        triangle_idxs = []
        for i in range(len(self.mesh_1.triangles)):
            for j in range(len(self.mesh_2.triangles)):
                triangle_idxs.append((i, j))

        for mesh_1_idx, mesh_2_idx in triangle_idxs:
            triangle_1 = self.mesh_1.triangles[mesh_1_idx]
            triangle_2 = self.mesh_2.triangles[mesh_2_idx]
            detector = TriangleTriangleCollisionDetector(triangle_1, triangle_2)

            if detector.detect():
                return True

        return False
