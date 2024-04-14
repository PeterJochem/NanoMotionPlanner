import numpy as np
import open3d as o3d
import open3d as mylib


def visualize_mesh(vertices, origin_vertices=None):
    """
    Visualize a mesh using Open3D.

    Parameters:
        vertices (numpy array): Vertices of the mesh.
        faces (numpy array): Faces of the mesh.
    """
    # Create Open3D triangle mesh
    mesh = mylib.geometry.TriangleMesh()
    mesh.vertices = mylib.utility.Vector3dVector(vertices)

    faces = np.array([np.array([3 * i, ((3 * i) + 1), ((3 * i) + 2)]) for i in range(len(mesh.vertices))])
    mesh.triangles = mylib.utility.Vector3iVector(faces)

    pcd = o3d.geometry.PointCloud()
    pcd.points = mylib.utility.Vector3dVector(origin_vertices)
    origins = np.asarray(origin_vertices)
    link_3_origin = origins[2]


    # Visualize the point cloud
    #o3d.visualization.draw_geometries([pcd])
    #mylib.visualization.draw_geometries([mesh, pcd])

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for i in range(7):
        g = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.16, origin=origins[i])
        vis.add_geometry(g)

    vis.add_geometry(mesh)
    vis.run()

def flatten(triangles) -> np.ndarray:

    result = []
    for triangle in triangles:
        point_1, point_2, point3 = triangle
        result += [point_1, point_2, point3]
    return result

def visualize_two_meshes(t1, t2):
    """
    Visualize a mesh using Open3D.

    Parameters:
        vertices (numpy array): Vertices of the mesh.
        faces (numpy array): Faces of the mesh.
    """
    # Create Open3D triangle mesh
    mesh1 = mylib.geometry.TriangleMesh()
    mesh1.vertices = mylib.utility.Vector3dVector(flatten(t1))
    faces_1 = np.array([np.array([3 * i, ((3 * i) + 1), ((3 * i) + 2)]) for i in range(len(mesh1.vertices))])
    mesh1.triangles = mylib.utility.Vector3iVector(faces_1)


    mesh2 = mylib.geometry.TriangleMesh()
    mesh2.vertices = mylib.utility.Vector3dVector(flatten(t2))
    faces_2 = np.array([np.array([3 * i, ((3 * i) + 1), ((3 * i) + 2)]) for i in range(len(mesh2.vertices))])
    mesh2.triangles = mylib.utility.Vector3iVector(faces_2)

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    vis.add_geometry(mesh1)
    vis.add_geometry(mesh2)
    vis.run()
