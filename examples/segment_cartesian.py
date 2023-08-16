from matplotlib import pyplot as plt

from core.segment import Segment

# Calculates a single spline *segment* given the start and end points,
# as well as the first derivative of the start and end points.

start_x = 0
start_y = 0
start_x_deriv = 1
start_y_deriv = 0

end_x = 1
end_y = 1
end_x_deriv = 1
end_y_deriv = 0

example_segment = Segment()
example_segment.compute_coeffs([start_x, start_x_deriv, 0, end_x, end_x_deriv, 0], [start_y, start_y_deriv, 0, end_y, end_y_deriv, 0])
x_values, y_values = example_segment.plot_points(resolution=100)

print('x(t): ' + str(example_segment.xpoly))
print('y(t): ' + str(example_segment.ypoly))

# Plot the curve
plt.plot(x_values, y_values)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Single spline segment')
plt.show()