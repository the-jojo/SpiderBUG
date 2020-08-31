import numpy as np
import math

from src.geom_old.Lines import Ray


class Cone:
    def __init__(self, min, gradient):
        self.min = min
        self.gradient = gradient

    def get_intersect_with_ray(self, ray: Ray) -> []:
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
        res = []
        x0 = ray.start[0] - self.min[0]
        y0 = ray.start[1] - self.min[1]
        z0 = ray.start[2] - self.min[2]

        u1 = ray.gradient[0]
        u2 = ray.gradient[1]
        u3 = ray.gradient[2]

        if (u1+u2)!= 0 and abs(u3/(u1+u2)) == 1:
            u2 = u2 + 0.000001

        c1 = math.pow(x0, 2)
        c2 = math.pow(y0, 2)
        c3 = math.pow(z0, 2)

        b1 = 2. * x0 * u1
        b2 = 2. * y0 * u2
        b3 = 2. * z0 * u3

        a1 = math.pow(u1, 2)
        a2 = math.pow(u2, 2)
        a3 = math.pow(u3, 2)

        c = c1 + c2 - c3
        b = b1 + b2 - b3
        a = a1 + a2 - a3

        l1 = (-b + math.sqrt(math.pow(b, 2) - 4 * a * c)) / (2 * a)
        l2 = (-b - math.sqrt(math.pow(b, 2) - 4 * a * c)) / (2 * a)

        if math.isnan(l1):
            x1 = x0 + l1 * u1
            y1 = y0 + l1 * u2
            z1 = z0 + l1 * u3

            p = np.array([x1+self.min[0], y1+self.min[1], z1+self.min[2]])
            if z1 >= 0 and ray.is_point_on_ray(p):
                res.append(p)
                
        if not math.isnan(l2) and l1 != l2:
            x2 = x0 + l2 * u1
            y2 = y0 + l2 * u2
            z2 = z0 + l2 * u3

            p = np.array([x2+self.min[0], y2+self.min[1], z2+self.min[2]])
            if z2 >= 0 and ray.is_point_on_ray(p):
                res.append(p)
                
        return res
