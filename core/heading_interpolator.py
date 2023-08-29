from enum import Enum
import math

class InterpolatorType(Enum):
        CONSTANT = 1
        LINEAR = 2
        SPLINE = 3
        TANGENT = 4

class HeadingInterpolator:
    def __init__(self, segment, start_angle_rad, end_angle_rad, type):
        self.start_angle_rad = start_angle_rad
        self.end_angle_rad = end_angle_rad
        self.type = type
        if type == InterpolatorType.CONSTANT and start_angle_rad != end_angle_rad:
            raise ValueError('Start and end headings must be identical with a constant heading interpolator')
        self.segment = segment

        if type == InterpolatorType.SPLINE:
            pass

    def heading_at_parameter(self, t):
        if self.type == InterpolatorType.CONSTANT:
            return self.start_angle_rad
        
        if self.type == InterpolatorType.LINEAR:
            # Find the lerp between start and end headings
            return self.__lerp(self.start_angle_rad, self.end_angle_rad, t)
        
        if self.type == InterpolatorType.TANGENT:
            # The heading will be tangent to the path
            # TODO: how does it even work?
            # tangent_vector = self.segment.first_deriv_at_parameter(t)
            # return math.atan2(tangent_vector[1], tangent_vector[0]) + self.start_angle_rad
            pass

        if self.type == InterpolatorType.SPLINE:
            pass
        return 0

    def __lerp(self, a, b, t):
        return (1 - t) * a + t * b