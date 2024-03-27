import pytest
from geometry.mesh import Mesh

#breakpoint()

import open3d as mylib


def visualize_mesh(vertices, faces):
    """
    Visualize a mesh using Open3D.

    Parameters:
        vertices (numpy array): Vertices of the mesh.
        faces (numpy array): Faces of the mesh.
    """
    # Create Open3D triangle mesh
    mesh = mylib.geometry.TriangleMesh()
    breakpoint()
    mesh.vertices = mylib.utility.Vector3dVector(vertices)
    mesh.triangles = mylib.utility.Vector3iVector(faces)

    # Visualize the mesh
    mylib.visualization.draw_geometries([mesh])


def test_reading_stl_file():

    # Make this a relative path.
    file_path = "/Users/peterjochem/Desktop/Personal_Learning/MotionPlanning/nano_motion_planner"
    file_path += "/src/robots/meshes/ur5/collision/forearm.stl"

    #vertices, faces = Mesh.from_file(file_path)
    triangles = Mesh.from_file(file_path)

    # There are 578 faces in Meshlab.
    # assert len(mesh.triangles) == 578

    # There are 283 vertices in Meshlab.
    #import numpy as np
    #vertices = np.array(vertices).reshape(( int(len(vertices) / 3), 3))
    #faces = np.array(faces)
    #visualize_mesh(vertices=vertices, faces=faces)




