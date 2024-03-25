
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_triangle(ax, vertices, color='b'):
    vertices = np.array(vertices)
    ax.plot(vertices[[0, 1, 2, 0], 0], vertices[[0, 1, 2, 0], 1], vertices[[0, 1, 2, 0], 2], color=color)



# Define vertices of the triangles
triangle1_vertices = [
    [1, 1, 1],  # A
    [3, 1, 1],  # B
    [2, 3, 1]   # C
]

triangle2_vertices = [
    [2, 2, 2],  # D
    [4, 2, 2],  # E
    [3, 4, 2]   # F
]

triangle1_vertices = np.array([[0., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
triangle2_vertices = np.array([[0., 2., 0.], [0., 2., 1.], [0., 2. + 1., 0.]])

triangle1_vertices = np.array([[0., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
triangle2_vertices = np.array([[1., 2., 0.], [2., 2., 1.], [3., 2. + 1., 0.]])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the triangles
plot_triangle(ax, triangle1_vertices, color='b')
plot_triangle(ax, triangle2_vertices, color='r')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
