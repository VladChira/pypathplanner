import math
import numpy
from core.segment import *

class Path:
    def __init__(self):
        self.segments = []
        self.length = 0
        self.manual_tangent_coefficient = 20

    def get_correct_segment(self, disp):
        # Immediately throw out absurd displacements
        if not in_range(disp, 0, self.length):
            raise ValueError(
                'Incorrect displacement provided (must be between 0 and the total length)')
        current_segment = Segment()
        found = False
        running_disp = 0
        for segment in self.segments:
            if disp >= running_disp and disp <= running_disp + segment.length:
                current_segment = segment
                found = True
                break
            running_disp += segment.length
        if found is False:
            raise ValueError(
                'Fatal Error: Could not find the correct segment for displacement. Perhaps a numerical error?')
        return current_segment, disp - running_disp

    # Returns a point on the path at a certain displacement
    def point_at_displacement(self, disp):
        # Find out in which segment we are
        current_segment, relative_disp = self.get_correct_segment(disp)
        # Now evaluate the segment at the correct point
        return current_segment.point_at_displacement(relative_disp)

    # Returns the path curvature at a certain displacement
    def curvature_at_displacement(self, disp):
        # Find out in which segment we are
        current_segment, relative_disp = self.get_correct_segment(disp)
        t = current_segment.parameter_at_displacement(relative_disp)
        rt1 = numpy.array(current_segment.first_deriv_at_parameter(t))
        rt2 = numpy.array(current_segment.second_deriv_at_parameter(t))
        return cross(rt1, rt2) / (numpy.linalg.norm(rt1) ** 3)

    def slope_at_displacement(self, disp):
        # Find out in which segment we are
        current_segment, relative_disp = self.get_correct_segment(disp)
        t = current_segment.parameter_at_displacement(relative_disp)
        return current_segment.tangent_at_parameter(t)

    # Returns the first derivative of the spline
    # at particular displacement
    def first_deriv_at_displacement(self, disp):
        # Find out in which segment we are
        current_segment, relative_disp = self.get_correct_segment(disp)
        t = current_segment.parameter_at_displacement(relative_disp)
        rt = numpy.array(current_segment.first_deriv_at_parameter(t))
        return rt / numpy.linalg.norm(rt)

    # Returns the second derivative of the spline
    # at particular displacement
    def second_deriv_at_displacement(self, disp):
        # Find out in which segment we are
        current_segment, relative_disp = self.get_correct_segment(disp)
        t = current_segment.parameter_at_displacement(relative_disp)
        rt1 = numpy.array(current_segment.first_deriv_at_parameter(t))
        rt2 = numpy.array(current_segment.second_deriv_at_parameter(t))
        return (rt2 / (numpy.linalg.norm(rt1) ** 2)) - (rt1 * rt2 * rt1) / (numpy.linalg.norm(rt1) ** 4)
    
    def heading_at_displacement(self, disp):
        # # Find out in which segment we are
        # current_segment, relative_disp = self.get_correct_segment(disp)
        # t = current_segment.parameter_at_displacement(relative_disp)
        # return current_segment.heading_interpolator.heading_at_parameter(t)
        return 0

    # Constructs a path of len(points)-1 segments
    # that passes through all points in the array.
    # All derivatives are given manually.
    # First derivative: in angle form
    # Second derivative: assume 0 for now
    def make_path(self, points, tangents):
        if len(points) != len(tangents):
            raise ValueError(
                'The number of points must match the number of angles provided.')
        if len(points) == 1:
            raise ValueError("A path must contain at least two points")
        
        for i in range(len(points) - 1):
            if points[i] == points[i-1]:
                raise ValueError('Null segment error. Duplicate points found.')

        for i in range(len(points) - 1):
            # Make a segment between current_point and next_point
            # with given tangents
            current_start_x = points[i][0]
            current_start_y = points[i][1]
            current_deriv_angle = tangents[i]

            next_start_x = points[i+1][0]
            next_start_y = points[i+1][1]
            next_deriv_angle = tangents[i+1]

            new_segment = Segment()
            new_segment.compute_coeffs([current_start_x, self.manual_tangent_coefficient * math.cos(math.radians(current_deriv_angle)), 0,
                                        next_start_x, self.manual_tangent_coefficient * math.cos(math.radians(next_deriv_angle)), 0],
                                       [current_start_y, self.manual_tangent_coefficient * math.sin(math.radians(current_deriv_angle)), 0,
                                        next_start_y, self.manual_tangent_coefficient * math.sin(math.radians(next_deriv_angle)), 0])
            self.segments.append(new_segment)

            self.length += new_segment.length

    def plot_points(self, resolution):
        x_points = []
        y_points = []
        for segment in self.segments:
            x_segment, y_segment = segment.plot_points(resolution)
            x_points += x_segment
            y_points += y_segment
        return x_points, y_points


def dist(A, B):
    a = numpy.array(A)
    b = numpy.array(B)
    return numpy.linalg.norm(a - b)

def cross(v1, v2):
    return v1[0] * v2[1] - v2[0] * v1[1]

def in_range(x, a, b, eps=0.01):
    return a-eps <= x <= b + eps

def map_range(value, from_min, from_max, to_min, to_max):
    # Ensure the value is within the input range
    value = max(min(value, from_max), from_min)
    # Calculate the proportion of the value in the input range
    proportion = (value - from_min) / (from_max - from_min)
    mapped_value = to_min + proportion * (to_max - to_min)
    return mapped_value
