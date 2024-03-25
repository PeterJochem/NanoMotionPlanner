
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_triangle(ax, vertices, color='b'):
    vertices = np.array(vertices)
    ax.plot(vertices[[0, 1, 2, 0], 0], vertices[[0, 1, 2, 0], 1], vertices[[0, 1, 2, 0], 2], color=color)



triangle_1 = np.array([[1., 1., 1.], [3., 1., 1.], [2., 3., 1.]])
triangle_2 = np.array([[2., 2., 0.], [4., 2., 0.], [3., 4., 0.]])


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the triangles
plot_triangle(ax, triangle_1, color='b')
plot_triangle(ax, triangle_2, color='r')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
