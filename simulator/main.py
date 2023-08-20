import pygame
import pygame.gfxdraw
import time
import numpy as np
from scipy.integrate import cumtrapz

from core.path import *
from core.motion_profile import *
from core.interp_lut import *

width = 900
height = 900

# path_points = [[0, 0], [10, 10], [30, 15], [60, 0], [30, 6]]
# path_tangents = [45, 45, 0, -45, 135]
# tangent_coeff = 20

# path_points = [ [0,0], [20, 0], [20, 50], [-50, -20]]
# path_tangents = [0, 0, 0, 0]
# tangent_coeff = 50

path_points = [[36, -63], [30.0, -23], [27, 0.5], [45, -10.5], [58.6, -9.5]]
path_tangents = [-10, 120, 45, 0, 15]
tangent_coeff = 20

# Basic constraints
max_vel = 30
max_acc = 40
max_ang_vel = 3.14
max_ang_acc = 3.14

robot_width = 18
robot_height = 18
field_len = 142

def rect(screen, center, width, height, angle_rad, color):
    cos_theta = math.cos(angle_rad)
    sin_theta = math.sin(angle_rad)

    vertices = [
        (center[0] + width * cos_theta - height * sin_theta, center[1] + width * sin_theta + height * cos_theta),
        (center[0] - width * cos_theta - height * sin_theta, center[1] - width * sin_theta + height * cos_theta),
        (center[0] - width * cos_theta + height * sin_theta, center[1] - width * sin_theta - height * cos_theta),
        (center[0] + width * cos_theta + height * sin_theta, center[1] + width * sin_theta - height * cos_theta)
    ]

    pygame.draw.lines(screen, color, True, vertices, 2)

def draw_line(screen, p0, p1, thickness, color):
    # https://stackoverflow.com/questions/30578068/pygame-draw-anti-aliased-thick-line
    # The fact that this abomination is the 'best' way to draw a thick anti-aliased line is just...
    # Bonus, it's also barely noticeably better
    center_L1 = [(p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0]
    length = math.sqrt( (p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)
    angle = math.atan2(p0[1] - p1[1], p0[0] - p1[0])
    UL = (center_L1[0] + (length/2.) * math.cos(angle) - (thickness/2.) * math.sin(angle),
      center_L1[1] + (thickness/2.) * math.cos(angle) + (length/2.) * math.sin(angle))
    UR = (center_L1[0] - (length/2.) * math.cos(angle) - (thickness/2.) * math.sin(angle),
        center_L1[1] + (thickness/2.) * math.cos(angle) - (length/2.) * math.sin(angle))
    BL = (center_L1[0] + (length/2.) * math.cos(angle) + (thickness/2.) * math.sin(angle),
        center_L1[1] - (thickness/2.) * math.cos(angle) + (length/2.) * math.sin(angle))
    BR = (center_L1[0] - (length/2.) * math.cos(angle) + (thickness/2.) * math.sin(angle),
        center_L1[1] - (thickness/2.) * math.cos(angle) - (length/2.) * math.sin(angle))
    pygame.gfxdraw.aapolygon(screen, (UL, UR, BR, BL), color)
    pygame.gfxdraw.filled_polygon(screen, (UL, UR, BR, BL), color)


def draw_path(screen, points):
    for i in range(0, len(points) - 1):
        draw_line(screen, points[i], points[i+1], 2, (255,255,255))

def main():
    pygame.init()
    pygame.display.set_caption("Vlad's Path Planner")
    screen = pygame.display.set_mode((width, height))
    field = pygame.image.load("./simulator/field.png")
    running = True
    
    start_time = time.perf_counter()
    # Create and build the path
    path = Path()
    path.manual_tangent_coefficient = tangent_coeff
    path.make_path(path_points, path_tangents)

    # Create and construct the motion profile based on path and constraints
    motion_profile = MotionProfile(path, max_vel, max_acc, max_ang_vel, max_ang_acc)
    profile_x, profile_y = motion_profile.make_profile(start_vel=0, end_vel=0, num_points=int(path.length / 0.5))
    heading_profile = motion_profile.heading_profile

    # Get the velocity profiles for each axis
    t = [point[0] for point in profile_x]
    velocities_x = [point[1].velocity for point in profile_x]
    velocities_y = [point[1].velocity for point in profile_y]

    # Integrate them to obtain relative positions for each axis
    # The start position is added to the relative positions
    x_position = [path_points[0][0] + x_offset for x_offset in cumtrapz(velocities_x, t, initial=0)]
    y_position = [path_points[0][1] + y_offset for y_offset in cumtrapz(velocities_y, t, initial=0)]
    # Create the interpolated lookup tables for the position
    lut_x = InterpLUT(t, x_position)
    lut_y = InterpLUT(t, y_position)

    # Get the heading
    heading = [math.degrees(h[1]) for h in heading_profile]
    lut_heading = InterpLUT(t, heading)

    print('Trajectory generated in {0:3.2f} seconds'.format(time.perf_counter() - start_time))
    print('Trajectory duration: {0:3.2f} seconds'.format(motion_profile.duration))

    start_time = time.perf_counter()
    animation = True

    # main loop
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        screen.blit(field, (0, 0))

        if animation is True:
            # Draw the path by mapping real coordinates to pixel coordinates
            x_path = [map_range(point, -field_len / 2, field_len / 2, 0, width) for point in x_position]
            y_path = [map_range(point, -field_len / 2, field_len / 2, 0, width) for point in y_position]
            points = list(zip(x_path, y_path))
            draw_path(screen, points)
            # pygame.draw.lines(screen, (255, 255, 255), False, points, 4)

            # Draw the knots by mapping real coordinates to pixel coordinates
            for point in path_points:
                pixel_point = [map_range(point[0], -field_len / 2, field_len / 2, 0, width), map_range(point[1], -field_len / 2, field_len / 2, 0, height)]
                pygame.draw.circle(screen, (255, 255, 255), tuple(pixel_point), 7)

            # Now draw the robot
            # Get the elapsed time since the start of the animation of the trajectory
            rel_t = time.perf_counter() - start_time
            # Check if the trajectory is finished and reset
            if rel_t > motion_profile.duration:
                start_time = time.perf_counter()
                screen.blit(field, (0, 0))
                continue

            # Get the pose of the robot at this time
            x = lut_x.lookup(rel_t)
            y = lut_y.lookup(rel_t)
            angle = lut_heading.lookup(rel_t)
            # Calculate the pixel coordinates and draw the rectangle
            pixel_point_x = map_range(x, -field_len / 2, field_len / 2, 0, width)
            pixel_point_y = map_range(y, -field_len / 2, field_len / 2, 0, height)
            rect(screen, (pixel_point_x, pixel_point_y), robot_width / 2 * (width / field_len), robot_height / 2 * (height / field_len), math.radians(angle), (255, 255, 255))

        pygame.display.update()

if __name__ == "__main__":
    main()