from core.path import *
from core.motion_profile import *
from matplotlib import pyplot as plt
import time

# path_points = [[0, 0], [10, 10], [30, 15], [60, 0], [30, 6]]
# path_tangents = [45, 45, 0, -45, 135]

# path_points = [ [0,0], [20, 0], [20, 50], [-50, -20]]
# path_tangents = [0, 0, 0, 0]

# path_points = [[0, 0], [20, 20], [40, 0]]
# path_tangents = [-30, -90, 135]

# path_points = [[36, -63], [30.0, -23], [27, 0.5], [45, -10.5], [58.6, -9.5]]
# path_tangents = [-10, 120, 45, 0, 15]

# path_points = [[0, 0], [20, 20], [40, 0]]
# path_tangents = [90, 0, -90]

# Basic constraints
max_vel = 30
max_acc = 40
max_ang_vel = 3.14
max_ang_acc = 3.14

start_time = time.perf_counter()

path_builder = PathBuilder([36, -63], start_heading_deg=-10)
path_builder.point_linear_heading([30.0, -23], tangent_angle_deg=120, heading_deg=120)
path_builder.point_constant_heading([27, 0.5], tangent_angle_deg=45)
path_builder.point_linear_heading([45, -10.5], tangent_angle_deg=0, heading_deg=0)
path_builder.point_constant_heading([58.6, -9.5], tangent_angle_deg=15)
path = path_builder.build()
path_points = path_builder.points

motion_profile = MotionProfile(path, max_vel, max_acc, max_ang_vel, max_ang_acc)
time_profile_x, time_profile_y = motion_profile.make_profile(start_vel=0, end_vel=0, num_points=200)
time_profile_abs = motion_profile.time_profile_abs
displacement_profile = motion_profile.displacement_profile
heading_profile = motion_profile.heading_profile

print('Trajectory generated in {0:3.2f} seconds'.format(time.perf_counter() - start_time))
print('Trajectory duration: {0:3.2f} seconds'.format(motion_profile.duration))

# ----- PLOTTING STUFF -----
figure, axis = plt.subplots(2, 3)

# ----- PATH -----
x_values, y_values = path.plot_points(resolution=100)
axis[0][0].plot(x_values, y_values, color='black')
axis[0][0].set_xlim([-71, 71])
axis[0][0].set_ylim([-71, 71])
# Scatter the knots
anchor_x = [point[0] for point in path_points]
anchor_y = [point[1] for point in path_points]
axis[0][0].scatter(anchor_x, anchor_y, color='black',
                   s=50, marker='o')
axis[0][0].set_title('Spline Path')

# ----- DISPLACEMENT_PROFILE (absolute value) -----
disp_vx = [point[0] for point in displacement_profile]
disp_vy = [point[1] for point in displacement_profile]
axis[0][1].plot(disp_vx, disp_vy, color='black')
axis[0][1].set_title('Displacement profile (absolute value)')
axis[0][1].set_xlabel('displacement along the path (in)')
axis[0][1].set_ylabel('velocity (in/s)')
axis[0][1].grid()

# ----- TIME PROFILE (absolute value) -----
t = [point[0] for point in time_profile_abs]
velocities = [point[1].velocity for point in time_profile_abs]
accelerations = [point[1].acceleration for point in time_profile_abs]
axis[0][2].plot(t, velocities, color='green')
axis[0][2].plot(t, accelerations, color='orange')
axis[0][2].set_title('Time profile (absolute value)')
axis[0][2].set_xlabel('time (s)')
axis[0][2].legend(['velocity (in/s)', 'acceleration (in/s^2)'])
axis[0][2].grid()

# ----- VELOCITY TIME PROFILE (per component) -----
t = [point[0] for point in time_profile_x]
velocities_x = [point[1].velocity for point in time_profile_x]
velocities_y = [point[1].velocity for point in time_profile_y]
axis[1][0].plot(t, velocities_x, color='blue')
axis[1][0].plot(t, velocities_y, color='red')
axis[1][0].set_title('Velocity profile (component-wise)')
axis[1][0].grid()
axis[1][0].legend(['ẋ', 'ẏ'])
axis[1][0].set_xlabel('time (s)')
axis[1][0].set_ylabel('velocity (in/s)')

# ----- ACCELERATION TIME PROFILE (per component) -----
acc_x = [point[1].acceleration for point in time_profile_x]
acc_y = [point[1].acceleration for point in time_profile_y]
axis[1][1].plot(t, acc_x, color='blue')
axis[1][1].plot(t, acc_y, color='red')
axis[1][1].set_title('Acceleration profile (component-wise)')
axis[1][1].grid()
axis[1][1].legend(['ẍ', 'ÿ'])
axis[1][1].set_xlabel('time (s)')
axis[1][1].set_ylabel('acceleration (in/s^2)')

# ----- HEADING-----
heading = [math.degrees(h[1]) for h in heading_profile]
axis[1][2].plot(t, heading)
axis[1][2].set_title('Heading (deg)')
plt.show()
