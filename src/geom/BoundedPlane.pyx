# cython: profile=True

import numpy as np
cimport numpy as cnp

from src.geom.Lines cimport LineSegment, Ray
from src.geom.Node cimport Node, _is_point_btw, _dist_2d, _as_2d, _dist_3d
from src.geom.Node import Node

# +++ cdef functions

debug = 0

def setdebug(s):
    debug = s

cdef cnp.ndarray _get_plane_normal(cnp.ndarray[double, ndim=1] plane_lower_a,
                                   cnp.ndarray[double, ndim=1] plane_lower_b,
                                   cnp.ndarray[double, ndim=1] plane_direction):
    cdef cnp.ndarray[double, ndim=1] c = plane_lower_b + plane_direction
    cdef cnp.ndarray[double, ndim=1] t = np.cross((plane_lower_b - plane_lower_a), (c - plane_lower_a))
    return t #_as_2d(t) # normal that defines this plane

cdef double _get_plane_constant(cnp.ndarray[double, ndim=1] plane_lower_a,
                                cnp.ndarray[double, ndim=1] plane_lower_b,
                                cnp.ndarray[double, ndim=1] plane_normal):
    cdef double c = 0 - (plane_lower_a[0] * plane_normal[0] +
                        plane_lower_a[1] * plane_normal[1] +
                        plane_lower_a[1] * plane_normal[1])
    return c

cdef cnp.ndarray _get_intersect_btw_plane_and_ray(cnp.ndarray[double, ndim=1] plane_lower_a,
                                        cnp.ndarray[double, ndim=1] plane_lower_b,
                                        cnp.ndarray[double, ndim=1] plane_normal,
                                        cnp.ndarray[double, ndim=1] plane_direction,
                                        double plane_constant,
                                        cnp.ndarray[double, ndim=1] ray_start,
                                        cnp.ndarray[double, ndim=1] ray_gradient):
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
        cdef cnp.ndarray[double, ndim=1] diff = ray_start - plane_lower_a
        cdef double prod1 = np.dot(diff, plane_normal)
        cdef double prod2 = np.dot(ray_gradient, plane_normal)
        if prod2 == 0:
            return None
        cdef double prod3 = prod1 / prod2
        cdef cnp.ndarray[double, ndim=1] intersect = ray_start - ray_gradient * prod3

        # check intersect is within bounds
        cdef double dist_1 = _dist_3d(intersect, plane_lower_a)
        cdef double dist_2 = _dist_3d(intersect, plane_lower_b)
        cdef cnp.ndarray[double, ndim=1] scaled_dir_a = plane_direction * (dist_1 / np.linalg.norm(plane_direction))
        cdef cnp.ndarray[double, ndim=1] scaled_dir_b = plane_direction * (dist_2 / np.linalg.norm(plane_direction))
        cdef cnp.ndarray[double, ndim=1] upper_a = plane_lower_a + scaled_dir_a
        cdef cnp.ndarray[double, ndim=1] upper_b = plane_lower_b + scaled_dir_b
        cdef cnp.ndarray[double, ndim=1] AB = upper_a - plane_lower_a
        cdef cnp.ndarray[double, ndim=1] AM = intersect - plane_lower_a
        cdef cnp.ndarray[double, ndim=1] BC = upper_b - upper_a
        cdef cnp.ndarray[double, ndim=1] BM = intersect - upper_a

        if (0 <= np.dot(AB, AM)) and (np.dot(AB, AM) <= np.dot(AB, AB)) and \
                (0 <= np.dot(BC, BM)) and (np.dot(BC, BM) <= np.dot(BC, BC)):
            return intersect
        return None

cdef cnp.ndarray _get_intersect_btw_plane_and_lseg(cnp.ndarray[double, ndim=1] plane_lower_a,
                                        cnp.ndarray[double, ndim=1] plane_lower_b,
                                        cnp.ndarray[double, ndim=1] plane_normal,
                                        cnp.ndarray[double, ndim=1] plane_direction,
                                        double plane_constant,
                                        cnp.ndarray[double, ndim=1] lseg_start,
                                        cnp.ndarray[double, ndim=1] lseg_end): # -> Node or None
        # turn to ray
        cdef cnp.ndarray[double, ndim=1] ray_start = lseg_start
        cdef cnp.ndarray[double, ndim=1] ray_gradient = lseg_end - lseg_start
        # find intersect
        cdef cnp.ndarray[double, ndim=1] res = _get_intersect_btw_plane_and_ray(plane_lower_a,
                                                                             plane_lower_b,
                                                                             plane_normal,
                                                                             plane_direction,
                                                                             plane_constant,
                                                                             ray_start,
                                                                             ray_gradient)
        # check out of bounds
        if res is None or not _is_point_btw(lseg_start, lseg_end, res):
            return None
        return res

# ++ class

cdef class BoundedPlane:
    #cdef readonly Node lower_a, lower_b, plane_direction, normal
    #cdef readonly double constant

    def __cinit__(self, Node a, Node b, Node v_dir, normal=None, constant=None):
        """
        Construct Plane from lower 2 points and direction vector of the plane (pointing upwards)
        :param a: lower point
        :param b: lower point
        :param v_dir: direction vector
        """
        self.lower_a = a
        self.lower_b = b
        self.plane_direction = v_dir
        cdef Node c, t
        if normal is None:
            c = self.lower_b + v_dir
            t = Node.from_array(np.cross((self.lower_b - self.lower_a).as_ndarray(), (c - self.lower_a).as_ndarray()))
            self.normal = t.as_unit_vector() # normal that defines this plane
        else:
            self.normal = normal
        if constant is None:
            self.constant = 0 - (self.lower_a.x() * self.normal.x() + self.lower_a.y() * self.normal.y()
                             + self.lower_a.y() * self.normal.y())  # constant of the plane equation
        else:
            self.constant = constant

    cpdef Node get_intersect_with_lseg(self, LineSegment lseg): # -> Node or None
        cdef cnp.ndarray[double, ndim=1] res = \
            _get_intersect_btw_plane_and_lseg(self.lower_a.as_ndarray(),
                                      self.lower_b.as_ndarray(),
                                      self.normal.as_ndarray(),
                                      self.plane_direction.as_ndarray(),
                                      self.constant,
                                      lseg.start.as_ndarray(),
                                      lseg.end.as_ndarray())
        if res is None:
            return None
        return Node.from_mem_slice(res)

    cpdef Node get_intersect_with_ray(self, Ray ray):
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
        cdef cnp.ndarray[double, ndim=1] res = \
            _get_intersect_btw_plane_and_ray(self.lower_a.as_ndarray(),
                                      self.lower_b.as_ndarray(),
                                      self.normal.as_ndarray(),
                                      self.plane_direction.as_ndarray(),
                                      self.constant,
                                      ray.start.as_ndarray(),
                                      ray.gradient.as_ndarray())
        if res is None:
            return None
        return Node.from_mem_slice(res)
