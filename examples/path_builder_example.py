from core.path import *
from core.motion_profile import *
from matplotlib import pyplot as plt


figure, axis = plt.subplots(1, 2)

# path = PathBuilder([0, 0], start_heading_deg=90).point_constant_heading([20, 20], tangent_angle=45).build()

path_builder = PathBuilder([36, -63], start_heading_deg=-10)
path_builder.point_constant_heading([30.0, -23], tangent_angle_deg=120)
path_builder.point_constant_heading([27, 0.5], tangent_angle_deg=45)
path_builder.point_constant_heading([45, -10.5], tangent_angle_deg=0)
path_builder.point_constant_heading([58.6, -9.5], tangent_angle_deg=15)
path = path_builder.build()

x_values, y_values = path.plot_points(resolution=100)
axis[0].plot(x_values, y_values)

max_vel = 30
max_acc = 40
max_ang_vel = 3.14
max_ang_acc = 3.14

motion_profile = MotionProfile(path, max_vel, max_acc, max_ang_vel, max_ang_acc)
motion_profile.make_profile(start_vel=0, end_vel=0, num_points=200)
time_profile_abs = motion_profile.time_profile_abs
t = [point[0] for point in time_profile_abs]
velocities = [point[1].velocity for point in time_profile_abs]
axis[1].plot(t, velocities, color='green')
axis[1].set_title('Time profile (absolute value)')
axis[1].set_xlabel('time (s)')
axis[1].legend(['velocity (in/s)'])
axis[1].grid()

plt.show()