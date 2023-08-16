from core.path import *
import math

class MotionProfile:
    def __init__(self, path, max_vel, max_acc, max_ang_vel, max_ang_acc):
        self.path = path
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_ang_vel = max_ang_vel
        self.max_ang_acc = max_ang_acc

        self.time_profile_x = []
        self.time_profile_y = []
        self.time_profile_abs = []

        self.displacement_profile = []

        self.heading_profile = []
        self.duration = 0

    # Build the profile based on the kinematic constraints, path,
    # as well as initial and end velocities
    def make_profile(self, start_vel, end_vel, num_points, start_acc=0, end_acc=0):
        # Place velocity planning points on the path
        self.displacement_profile = []
        for s in numpy.linspace(0, self.path.length, num_points):
            planning_vel = self.max_vel
            ci = self.path.curvature_at_displacement(s)
            vmax_omega = 0
            if abs(ci) < 0.001:
                vmax_omega = float('inf')
            else:
                vmax_omega = self.max_ang_acc / abs(ci)
            planning_vel = min(planning_vel, vmax_omega)
            self.displacement_profile.append([s, planning_vel])

        # Add the start and end velocities
        self.displacement_profile[0] = [0.0, start_vel]
        self.displacement_profile[num_points - 1] = [self.path.length, end_vel]

        # ----- FORWARD PASS -----
        for i in range(1, num_points):
            current_point = self.displacement_profile[i]
            prev_point = self.displacement_profile[i-1]
            # The velocity of the current point must be adjusted if its too high
            # i.e. we cannot accelerate from the previous point to this one
            if current_point[1] <= prev_point[1]:
                # nothing to do here because its less or equal (we don't accelerate)
                continue
            current_vel = current_point[1]
            prev_vel = prev_point[1]
            ds = current_point[0] - prev_point[0]
            current_max_vel = math.sqrt(prev_vel ** 2 + 2 * self.max_acc * ds)

            self.displacement_profile[i][1] = min(current_max_vel, current_vel)

        # ----- BACKWARDS PASS -----
        for i in range(num_points - 1, 0, -1):
            current_point = self.displacement_profile[i]
            prev_point = self.displacement_profile[i-1]
            # The velocity of the previous point must be adjusted if its too high
            # i.e. we cannot decelerate from the previous point to the current one
            if current_point[1] >= prev_point[1]:
                # nothing to do here because its greater or equal (we don't decelerate)
                continue
            current_vel = current_point[1]
            prev_vel = prev_point[1]
            ds = current_point[0] - prev_point[0]
            prev_max_vel = math.sqrt(current_vel ** 2 + 2 * self.max_acc * ds)

            self.displacement_profile[i -1][1] = min(prev_max_vel, prev_point[1])

        # ----- TIME PROFILE CREATION-----
        self.time_profile_abs.append([0.0, KinematicState(0, start_vel, start_acc)])
        tangent_vector = numpy.array(self.path.first_deriv_at_displacement(0))
        # normalize it, it should never be 0
        tangent_vector /= numpy.linalg.norm(tangent_vector)

        self.time_profile_x.append([0, KinematicState(0, start_vel * tangent_vector[0], start_acc * tangent_vector[0])])
        self.time_profile_y.append([0, KinematicState(0, start_vel * tangent_vector[1], start_acc * tangent_vector[1])])

        self.heading_profile.append([0, math.radians(30)])

        prev_time = 0
        current_time = 0
        for i in range(1, num_points):
            current_point = self.displacement_profile[i]
            prev_point = self.displacement_profile[i-1]

            current_vel = current_point[1]
            prev_vel = prev_point[1]

            ds = current_point[0] - prev_point[0]
            dt = 2 * ds / (current_vel + prev_vel)

            current_time += dt

            tangent_vector = numpy.array(self.path.first_deriv_at_displacement(current_point[0]))
            # Normalize it, it should never be 0
            tangent_vector /= numpy.linalg.norm(tangent_vector)

            v_x = current_vel * tangent_vector[0]
            v_y = current_vel * tangent_vector[1]

            current_acc = (current_vel - prev_vel) / (current_time - prev_time)

            self.time_profile_abs.append([current_time,  KinematicState(current_point[0], current_vel, current_acc)])

            self.time_profile_x.append([current_time, KinematicState(current_point[0], v_x, current_acc * tangent_vector[0])])
            self.time_profile_y.append([current_time, KinematicState(current_point[0], v_y, current_acc * tangent_vector[1])])

            self.heading_profile.append([current_time, self.path.heading_at_displacement(current_point[0])])

            prev_time = current_time

        self.duration = current_time
        return self.time_profile_x, self.time_profile_y


class KinematicState:
    def __init__(self, position, velocity, acceleration=0, jerk=0):
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.jerk = jerk
