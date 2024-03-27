from typing import Tuple, List
import numpy as np
import struct
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

    @classmethod
    def read_vertices_and_faces_from_binary_stl_file(cls, filename: str) -> Tuple[List, List]:
        """Reads a binary STL file and returns the vertices and faces.

        Args:
            filename: str:
                The path to the binary STL file.

        Returns:
            Tuple[List, List]:
                The vertices and faces.
        """
        vertices = []
        faces = []

        with open(filename, 'rb') as file:
            # Skip the first 80 bytes (header)
            file.read(80)

            # Read the number of triangles (4 bytes)
            num_triangles_bytes = file.read(4)
            num_triangles = struct.unpack('<I', num_triangles_bytes)[0]

            for _ in range(num_triangles):
                # Read the normal vector (12 bytes, 3 floats)
                file.read(12)

                # Read the vertices (36 bytes, 3 floats each)
                for _ in range(3):
                    vertex_bytes = file.read(12)  # 3 floats (12 bytes) for each vertex
                    vertices.extend(struct.unpack('<3f', vertex_bytes))

                    vertices[-1] = round(vertices[-1], 10)
                    vertices[-2] = round(vertices[-2], 10)
                    vertices[-3] = round(vertices[-3], 10)

                # Skip the attribute byte count (2 bytes)
                file.read(2)

                # Add face indices
                face = [len(vertices) // 3 - 3, len(vertices) // 3 - 2, len(vertices) // 3 - 1]
                faces.append(face)

        return vertices, faces

    @classmethod
    def read_vertices_from_binary_stl_file(cls, filename: str) -> np.array:
        """Reads a binary STL file and return the vertices and faces.

        Args:
            filename: str
                The path to the binary STL file.

        Returns:
            numpy.array:
                Nx3x3 array. N set of 3 points in 3D space.
        """
        triangles = []

        with open(filename, 'rb') as file:
            # Skip the first 80 bytes (header)
            file.read(80)

            # Read the number of triangles (4 bytes)
            num_triangles_bytes = file.read(4)
            num_triangles = struct.unpack('<I', num_triangles_bytes)[0]

            for _ in range(num_triangles):
                # Read the normal vector (12 bytes, 3 floats)
                file.read(12)

                # Read the vertices (36 bytes, 3 floats each)
                new_triangle = []
                for _ in range(3):
                    vertex_bytes = file.read(12)  # 3 floats (12 bytes) for each vertex
                    new_vertex = list(struct.unpack('<3f', vertex_bytes))

                    new_vertex[-1] = round(new_vertex[-1], 10)
                    new_vertex[-2] = round(new_vertex[-2], 10)
                    new_vertex[-3] = round(new_vertex[-3], 10)

                    new_triangle.append(new_vertex)

                triangles.append(new_triangle)

                # Skip the attribute byte count (2 bytes)
                file.read(2)

        return np.array(triangles)

    @classmethod
    def from_file(cls, file_path: str) -> "Mesh":
        """Reads in a binary STL mesh from a file.

        Args:
            file_path: str
                The path to the binary STL file.

        Returns:
            Mesh:
                The binary STL mesh represented as a Mesh object.
        """

        # vertices, faces = cls.read_vertices_and_faces_from_binary_stl_file(file_path)
        triangles = cls.read_vertices_from_binary_stl_file(file_path)
        return Mesh(triangles)


