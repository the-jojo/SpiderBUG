# cython: profile=True

import math
cimport numpy as cnp
import numpy as np

from src.geom.Lines cimport Ray, _is_point_on_ray
from src.geom.Node cimport Node
from src.geom.Node import Node


cdef list get_intersect_with_ray(cnp.ndarray[double, ndim=1] cone_min,
                                        double cone_gradient,
                                        cnp.ndarray[double, ndim=1] ray_start,
                                        cnp.ndarray[double, ndim=1] ray_gradient): # -> []
        """
        Cone equation
             A*x^2 + B*y^2 = C*z^2
             A = 1/gradient^2
             B = 1/gradient^2
             C = 1
        Line equations
             x = x0 + l * u1
             y = y0 + l * u2
             z = z0 + l * u3
        So we substitute the line equations into the cone equations to get
             A*(x0 + l*u1)^2 + B*(y0 + l*u2)^2 - C*(z0 + l*u3)^2 = 0
        From this we get
             [ (A*x0^2) + (A*2*x0*u1)*l + (A*u1^2)*l^2 ] + [ B*y0^2 + (B*2*y0*u2)*l + (B*u2^2)*l^2 ] - [ C*z0^2 + C*2*z0*u3*l + C*u3^2*l^2 ]
             [    c1    +     b1     *l +   a1    *l^2 ] + [   c2   +   b2       *l +   a2    *l^2 ] - [  c3  +   b3*l    +   a3*l^2 ]
             c = c1 + c2 - c3
             b = b1 + b2 - b3
             a = a1 + a2 - a3
        Using the quadratic formula we yield
             l1 = (-b + sqrt(b^2-4*a*c))/(2*a)
             l2 = (-b - sqrt(b^2-4*a*c))/(2*a)
        none, l1, l2 or both may be NaN indicating 2, 1 or no solutions
        substitute l1 and l2 back into line equations to yield coordinates of intersects p1 and p2
        check p1 and p2 have positive z coordinate
        translate p1 and p2 back to real positions using real cone min m
             p1 = p1 + m
             p2 = p2 + m
        check p1 and p2 lie on ray
        :return: 2, 1, or 0 intersection points. Will return 0 points if ray is on cone
        """
        if ray_gradient[0] == 0:
            ray_gradient[0] = 0.0000001
        if ray_gradient[1] == 0:
            ray_gradient[1] = 0.0000001
        if ray_gradient[2] == 0:
            ray_gradient[2] = 0.0000001
        cdef list res = []
        cdef double A = 1. / math.pow(cone_gradient, 2.)
        cdef double B = 1. / math.pow(cone_gradient, 2.)
        cdef double C = 1.

        cdef double x0 = ray_start[0] - cone_min[0]
        cdef double y0 = ray_start[1] - cone_min[1]
        cdef double z0 = ray_start[2] - cone_min[2]

        cdef double u1 = ray_gradient[0]
        cdef double u2 = ray_gradient[1]
        cdef double u3 = ray_gradient[2]

        if (u1+u2) == 0 or abs(u3/(u1+u2)) == 1:
            u2 = u2 + 0.000001

        cdef double c1 = A * math.pow(x0, 2)
        cdef double c2 = B * math.pow(y0, 2)
        cdef double c3 = C * math.pow(z0, 2)

        cdef double b1 = A * 2. * x0 * u1
        cdef double b2 = B * 2. * y0 * u2
        cdef double b3 = C * 2. * z0 * u3

        cdef double a1 = A * math.pow(u1, 2)
        cdef double a2 = B * math.pow(u2, 2)
        cdef double a3 = C * math.pow(u3, 2)

        cdef double c = c1 + c2 - c3
        cdef double b = b1 + b2 - b3
        cdef double a = a1 + a2 - a3

        cdef double l1, l2, x1, y1, z1, x2, y2, z2
        try:
            l1 = (-b + math.sqrt(math.pow(b, 2) - 4 * a * c)) / (2 * a)
        except:
            l1 = math.nan

        try:
            l2 = (-b - math.sqrt(math.pow(b, 2) - 4 * a * c)) / (2 * a)
        except:
            l2 = math.nan

        cdef cnp.ndarray[double, ndim=1] p0, p1
        if not math.isnan(l1):
            x1 = x0 + l1 * u1
            y1 = y0 + l1 * u2
            z1 = z0 + l1 * u3
            p0 = np.array([x1+cone_min[0], y1+cone_min[1], z1+cone_min[2]])

            if z1 > 0 and _is_point_on_ray(ray_start, ray_gradient, p0):
                res.append(p0)

        if not math.isnan(l2) and not l1 == l2:
            x2 = x0 + l2 * u1
            y2 = y0 + l2 * u2
            z2 = z0 + l2 * u3
            p1 = np.array([x2+cone_min[0], y2+cone_min[1], z2+cone_min[2]])

            if z2 >= 0 and _is_point_on_ray(ray_start, ray_gradient, p1):
                res.append(p1)

        return res


cdef class Cone:
    #cdef readonly Node min
    #cdef readonly double gradient

    def __cinit__(self, Node minimum, double gradient):
        self.min = minimum
        self.gradient = gradient

    cpdef list get_intersect_with_ray(self, Ray ray): # -> []
        """
        Cone equation
             x^2 + y^2 = z^2
        Line equations
             x = x0 + l * u1
             y = y0 + l * u2
             z = z0 + l * u3
        So we substitute the line equations into the cone equations to get
             (x0 + l*u1)^2 + (y0 + l*u2)^2 - (z0 + l*u3)^2 = 0
        From this we get
             [ x0^2 + 2*x0*u1*l + u1^2*l^2 ] + [ y0^2 + 2*y0*u2*l + u2^2*l^2 ] - [ z0^2 + 2*z0*u3*l + u3^2*l^2 ]
             [  c1  +   b1*l    +   a1*l^2 ] + [  c2  +   b2*l    +   a2*l^2 ] - [  c3  +   b3*l    +   a3*l^2 ]
             c = c1 + c2 - c3
             b = b1 + b2 - b3
             a = a1 + a2 - a3
        Using the quadratic formula we yield
             l1 = (-b + sqrt(b^2-4*a*c))/(2*a)
             l2 = (-b - sqrt(b^2-4*a*c))/(2*a)
        none, l1, l2 or both may be NaN indicating 2, 1 or no solutions
        substitute l1 and l2 back into line equations to yield coordinates of intersects p1 and p2
        check p1 and p2 have positive z coordinate
        translate p1 and p2 back to real positions using real cone min m
             p1 = p1 + m
             p2 = p2 + m
        check p1 and p2 lie on ray
        :param ray:
        :return: 2, 1, or 0 intersection points. Will return 0 points if ray is on cone
        """
        cdef list res = get_intersect_with_ray(self.min.as_ndarray(), self.gradient,
                               ray.start.as_ndarray(), ray.gradient.as_ndarray())
        return [Node.from_mem_slice(r) for r in res]

