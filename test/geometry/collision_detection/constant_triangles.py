import numpy as np

# The triangles don't overlap but they lie in the same plane.
triangle_1 = np.array([[0., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
triangle_2 = np.array([[0., 2., 0.], [0., 2., 1.], [0., 2. + 1., 0.]])

triangle_3 = np.array([[0., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
triangle_4 = np.array([[1., 2., 0.], [2., 2., 1.], [3., 2. + 1., 0.]])

triangle_5 = np.array([[0., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
triangle_6 = np.array([[1., 0., 0.], [2., 0., 1.], [3., 2. + 1., 0.]])

triangle_7 = np.array([[0., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
triangle_8 = np.array([[1., 0., 0.], [2., 0., 1.], [0., 2. + 1., 0.]])

triangle_9 = np.array([[1.1, 1.4, 1.6], [3., 1., 1.], [2., 3., 1.]])
triangle_10 = np.array([[0., 2., 0.], [4., 2., 2.2], [3., 4., 2.]])

triangle_11 = np.array([[1., 1., 1.], [3., 1., 1.], [2., 3., 1.]])
triangle_12 = np.array([[0., 1.5, -1.], [5., 1.5, 3.], [4., 3., 5.]])

triangle_13 = np.array([[1., 1., 1.], [3., 1., 1.], [2., 3., 1.]])
triangle_14 = np.array([[2., 2., 0.], [4., 2., 0.], [3., 4., 0.]])

triangle_15 = np.array([[1., 1., 1.], [3., 1., 1.], [2., 3., 1.]])
triangle_16 = np.array([[2., 2., 0.], [4., 2., 0.], [3., 4., 0.]])

triangle_17 = np.array([[1., 1., 2.2], [3., 1., 1.2], [2., 3., 1.]])
triangle_18 = np.array([[2., 2., 0.], [4., 2., 0.], [3., 4., 0.]])

triangle_19 = np.array([[1., 1., 2.2], [3., 1., 1.2], [2., 3., 1.]])
triangle_20 = np.array([[2., 2., 0.], [4., 2., 5.], [3., 4., 9.]])
