import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Transformation matrix
transformation_matrix = np.array([
    [-0.05212426, -0.9831042,  0.17546761,  0.],
    [-0.6679033,  -0.09630632, -0.7379905,  0.],
    [ 0.7424203,  -0.15566278, -0.6515986,  0.],
    [ 0.1185307,   0.12574661, -0.21154772, 1.]
])

# World points
world_points = np.array([
    [0.94448443, -0.08872926, -0.74513789],
    [1.48042676, 0.14289996, -0.88899106]
])

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the transformation matrix as a set of vectors
origin = np.array([0, 0, 0])
ax.quiver(origin[0], origin[1], origin[2], transformation_matrix[0, 0], transformation_matrix[1, 0], transformation_matrix[2, 0], color='r', label='X-axis')
ax.quiver(origin[0], origin[1], origin[2], transformation_matrix[0, 1], transformation_matrix[1, 1], transformation_matrix[2, 1], color='g', label='Y-axis')
ax.quiver(origin[0], origin[1], origin[2], transformation_matrix[0, 2], transformation_matrix[1, 2], transformation_matrix[2, 2], color='b', label='Z-axis')

# Plot the world points
ax.scatter(world_points[:, 0], world_points[:, 1], world_points[:, 2], color='k', s=100, label='World Points')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Transformation and World Points')
ax.legend()

plt.show()