from core.quintic_polynomial import *
import math
from scipy.integrate import quad
from scipy import linalg, optimize

coefficient_matrix = [
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 2, 0, 0],
    [1, 1, 1, 1, 1, 1],
    [5, 4, 3, 2, 1, 0],
    [20, 12, 6, 2, 0, 0]
]

class Segment:
    def __init__(self):
        self.xpoly = QuinticPolynomial(0, 0, 0, 0, 0, 0)
        self.ypoly = QuinticPolynomial(0, 0, 0, 0, 0, 0)

        self.xpoly_first_deriv = QuinticPolynomial(0, 0, 0, 0, 0, 0)
        self.ypoly_first_deriv = QuinticPolynomial(0, 0, 0, 0, 0, 0)

        self.xpoly_second_deriv = QuinticPolynomial(0, 0, 0, 0, 0, 0)
        self.ypoly_second_deriv = QuinticPolynomial(0, 0, 0, 0, 0, 0)
        self.length = -1
        self.heading_interpolator = None

    # Generates the points necessary for plotting the segment
    # with given no. of points
    def plot_points(self, resolution):
        parameter_values = [i / resolution for i in range(resolution+1)]
        curve_points = [self.point_at_parameter(p) for p in parameter_values]

        x_values = [point[0] for point in curve_points]
        y_values = [point[1] for point in curve_points]
        return x_values, y_values

    def point_at_parameter(self, t):
        return self.xpoly.eval(t), self.ypoly.eval(t)

    def point_at_displacement(self, s):
        return self.point_at_parameter(self.parameter_at_displacement(s))

    def first_deriv_at_parameter(self, t):
        return self.xpoly_first_deriv.eval(t), self.ypoly_first_deriv.eval(t)

    def second_deriv_at_parameter(self, t):
        return self.xpoly_second_deriv.eval(t), self.ypoly_second_deriv.eval(t)

    def tangent_at_parameter(self, t):
        xderiv, yderiv = self.first_deriv_at_parameter(t)
        if abs(xderiv) < 0.1:
            return float('inf')
        return yderiv / xderiv

    def unit_arc_length(self, tau):
        return math.sqrt(self.xpoly_first_deriv.eval(tau)**2 + self.ypoly_first_deriv.eval(tau)**2)

    # Integrate sqrt(dx/dt ^2 + dy/dt^2) from 0 to t
    # to find the arc length
    # Returns s(t)
    def displacement_at_parameter(self, t):
        len, _ = quad(self.unit_arc_length, 0, t)
        return len

    # Calculates at which t in [0,1] we have a particular displacement s0
    # Returns t such that s(t) = s0
    # Finds the root of the function s(t) - s0 = 0 using Brent's method
    def parameter_at_displacement(self, s0):
        if not in_range(s0, 0, self.length):
            raise ValueError('Incorrect displacement provided')
        
        def f(t):
            return self.displacement_at_parameter(t) - s0
        t = optimize.brentq(f, 0, 1)
        if in_range(t, 0, 1, eps=0.01):
            return t
        else:
            raise ValueError("parameter_at_displacement failed")
        
        # _, t = custom_trapezoidal(
        #     self.unit_arc_length, 0, 1, 1000, s, eps=0.2)
        # if t == -1:
        #     # The integration failed, as a last resort try integrating with more steps and higher tolerance
        #     print('Integration failed, attempting to fix it')
        #     _, t = custom_trapezoidal(
        #         self.unit_arc_length, 0, 1, 3000, s, eps=0.2)
        #     if t == -1:
        #         print("This should not show. Check parameter_at function")
        #         return 1
        #     return t
        # else:
        #     return t

    # Calculates the coefficients of the polynomials given the
    # values and derivatives
    # It solves the systems:
    # coefficient_matrix * x_coeffs = x_derivs
    # coefficient_matrix * y_coeffs = y_derivs
    def compute_coeffs(self, x_derivs, y_derivs):
        if not (len(x_derivs) == 6 and len(y_derivs) == 6):
            raise ValueError(
                "Invalid coefficients. Expected xn, xn', xn\", xn+1, xn+1', xn+1\" ")

        x_coeffs = linalg.solve(coefficient_matrix, x_derivs)
        y_coeffs = linalg.solve(coefficient_matrix, y_derivs)
        self.xpoly = QuinticPolynomial(x_coeffs)
        self.ypoly = QuinticPolynomial(y_coeffs)

        self.xpoly_first_deriv = self.xpoly.first_deriv()
        self.xpoly_second_deriv = self.xpoly.second_deriv()

        self.ypoly_first_deriv = self.ypoly.first_deriv()
        self.ypoly_second_deriv = self.ypoly.second_deriv()

        self.length = self.displacement_at_parameter(1)

# Custom integration for stopping
# Not ideal because its a poor algorithm
# DEPRECATED
def custom_trapezoidal(f, x0, xn, n, stop, eps=0.1):
    h = (xn - x0) / n

    integration = (f(x0) + f(xn)) / 2

    for i in range(1, n):
        k = x0 + i * h
        integration += f(k) * 2
        if math.fabs(integration * h / 2 - stop) <= eps:
            return integration, k - h

    integration *= h / 2
    return integration, -1
    
def in_range(x, a, b, eps=0.01):
    return a-eps <= x <= b + eps
