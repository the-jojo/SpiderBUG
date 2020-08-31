import numpy as np

from src.geom_old.Utils import *
from src.geom_old.Lines import LineSegment, Ray


class BoundedPlane:
    def __init__(self, a, b, v_dir):
        """
        Construct Plane from lower 2 points and direction vector of the plane (pointing upwards)
        :param a: lower point
        :param b: lower point
        :param v_dir: direction vector
        """
        self.lower_a = Node(a).as3d()
        self.lower_b = Node(b).as3d()
        self.plane_direction = v_dir
        c = self.lower_b + v_dir
        t = np.cross((self.lower_b - self.lower_a), (c - self.lower_a))
        norm = unit_vector(t)
        self.normal = Node(norm).as3d()  # normal that defines this plane
        self.constant = 0 - (self.lower_a[0] * self.normal[0] + self.lower_a[1] * self.normal[1] + self.lower_a[2] * self.normal[2])  # constant of the plane equation

    def get_intersect_with_lseg(self, lseg: LineSegment) -> Node or None:
        # turn to ray
        lray = lseg.to_ray()
        # find intersect
        res = self.get_intersect_with_ray(lray)
        # check out of bounds
        if res is None or not is_point_btw(lseg.start, lseg.end, res):
            return None
        return res

    def get_intersect_with_ray(self, ray: Ray):
        """
        Find the intersection of the plane and a given ray.

        The return value is positive is the intersection is found and
        this value gives the distance along the ray. Negative values
        imply that the intersection was either not successful or the
        intersection point was before the origin. This value can be used
        with the pointAt method of the Ray class (@see Ray#pointAt)
        :param ray: the ray to getIntersect with
        :return: the distance along the ray.
        """
        diff = ray.start.as2d() - self.lower_a.as2d()
        prod1 = np.dot(diff, self.normal)
        prod2 = np.dot(ray.gradient, self.normal)
        prod3 = prod1 / prod2
        intersect = ray.start - ray.gradient * prod3

        # check intersect is within bounds
        dist_1 = dist_2d_p(intersect, self.lower_a)
        dist_2 = dist_2d_p(intersect, self.lower_b)
        scaled_dir_a = self.plane_direction * (dist_1 / np.linalg.norm(self.plane_direction))
        scaled_dir_b = self.plane_direction * (dist_2 / np.linalg.norm(self.plane_direction))
        upper_a = self.lower_a + scaled_dir_a
        upper_b = self.lower_b + scaled_dir_b
        AB = upper_a - self.lower_a
        AM = intersect - self.lower_a
        BC = upper_b - upper_a
        BM = intersect - upper_a

        if (0 <= np.dot(AB, AM)) and \
                (np.dot(AB, AM) <= np.dot(AB, AB)) and \
                (0 <= np.dot(BC, BM)) and \
                (np.dot(BC, BM) <= np.dot(BC, BC)):
            return intersect

        return None
