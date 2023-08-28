from core.path import Path
import numpy as np
from matplotlib import pyplot as plt

# path_points = [[36, -63], [30.0, -23], [27, 0.5], [45, -10.5], [58.6, -9.5]]
# path_tangents = [120, 120, 45, 0, 15]

#path_points = [[0, 0], [10, 10], [30, 15], [60, 0], [30, 6]]
#path_tangents = [45, 45, 0, -45, 135]

path_points = [[0, 0], [20, 20], [40, 0]]
path_tangents = [90, 0, -90]

path = Path()
path.make_path(path_points, path_tangents)

figure, axis = plt.subplots(2, 2)


# The plot_points function works by plotting each segment
# separately based on the t in [0,1] parameter
x_values, y_values = path.plot_points(resolution=100)

# It's also possible to do it based on displacement
# along the entire curve, like this:
# for s in np.linspace(0, path.length, 50):
#     p = path.point_at_displacement(s)
#     x_values.append(p[0])
#     y_values.append(p[1])
axis[0][0].plot(x_values, y_values)


# Plot the tangent (slope) against the displacement
x_tangent = []
y_tangent = []
for s in np.linspace(0, path.length, 50):
    slope = path.slope_at_displacement(s)
    x_tangent.append(s)
    y_tangent.append(slope)

axis[0][1].plot(x_tangent, y_tangent)

# Plot the first derivative against the displacement
x_deriv1 = []
y_deriv1 = []
for s in np.linspace(0, path.length, 100):
    p = path.first_deriv_at_displacement(s)
    x_deriv1.append(p[0])
    y_deriv1.append(p[1])
axis[1][0].plot(x_deriv1, y_deriv1)

# Plot the curvature against the displacement
x_curv = []
y_curv = []
for s in np.linspace(0, path.length, 50):
    slope = path.curvature_at_displacement(s)
    x_curv.append(s)
    y_curv.append(slope)

axis[1][1].plot(x_curv, y_curv)

# Scatter the knots
anchor_x = [point[0] for point in path_points]
anchor_y = [point[1] for point in path_points]
axis[0][0].scatter(anchor_x, anchor_y, color='red',
                   s=50, marker='o', zorder=10)

axis[0][0].set_xlabel('X')
axis[0][0].set_ylabel('Y')
axis[0][0].set_title('Example spline')

axis[0][1].set_xlabel('Displacement')
axis[0][1].set_ylabel('Slope')
axis[0][1].set_title('Slope vs displacement')

axis[1][0].set_xlabel('X')
axis[1][0].set_ylabel('Y')
axis[1][0].set_title('First derivative')

axis[1][1].set_xlabel('Displacement')
axis[1][1].set_ylabel('Curvature')
axis[1][1].set_title('Curvature vs displacement')

# p = path.point_at_displacement(73.0)
# axis[0][0].scatter(p[0], p[1], color='black', s=50, zorder=10)

plt.show()
