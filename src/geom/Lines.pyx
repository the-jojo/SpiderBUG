import math
import numpy as np
cimport numpy as cnp

from src.geom.Node cimport Node, _is_point_btw, _dist_3d, _as_unit_vector
from src.geom.Node import Node

# +++ Line cdef functions

cdef cnp.ndarray _get_closest_point_on_line_to_p_2d(cnp.ndarray[double, ndim=1] lseg_start,
                                                    cnp.ndarray[double, ndim=1] lseg_end,
                                                    cnp.ndarray[double, ndim=1] point):
    """Get point on line closest to given point or None if given point is past the ends of the line segment"""
    cdef double k = (((lseg_end[1] - lseg_start[1]) * (point[0] - lseg_start[0]) - (lseg_end[0] - lseg_start[0])
          * (point[1] - lseg_start[1])) / (math.pow(lseg_end[1] - lseg_start[1], 2)
          + math.pow(lseg_end[0] - lseg_start[0], 2)))
    cdef double x4 = point[0] - k * (lseg_end[1] - lseg_start[1])
    cdef double y4 = point[1] + k * (lseg_end[0] - lseg_start[0])
    cdef cnp.ndarray[double, ndim=1] intersect = np.array([x4, y4, 0])
    if _is_point_btw(lseg_start, lseg_end, intersect):
        return intersect
    else:
        return None

cdef double _get_distance_point_to_line(cnp.ndarray[double, ndim=1] lseg_start,
                                        cnp.ndarray[double, ndim=1] lseg_end,
                                        cnp.ndarray[double, ndim=1] point):
    """
    Calls '_get_closest_point_on_line_to_p_2d()' and returns the 3D distance from the returned point to
    the given point. If the function call returns None, the minimum distance between the ends of the 
    line segment and the given point is returned 
    """
    cdef cnp.ndarray[double, ndim=1] x = _get_closest_point_on_line_to_p_2d(lseg_start,
                                                                            lseg_end,
                                                                            point)
    if x is not None:
        # x is on line segment
        return _dist_3d(x, point)
    else:
        # x is past the ends of the line segment
        return min(_dist_3d(lseg_start, point), _dist_3d(lseg_end, point))

cdef cnp.ndarray _get_intersect_btw_lsegs_2d(cnp.ndarray[double, ndim=1] lseg_0_start,
                                             cnp.ndarray[double, ndim=1] lseg_0_end,
                                             cnp.ndarray[double, ndim=1] lseg_1_start,
                                             cnp.ndarray[double, ndim=1] lseg_1_end):
    """gets intersect of this and other line segment, None if they dont meet"""
    cdef double x1 = lseg_1_start[0]
    cdef double y1 = lseg_1_start[1]
    cdef double x2 = lseg_1_end[0]
    cdef double y2 = lseg_1_end[1]
    cdef double x3 = lseg_0_start[0]
    cdef double y3 = lseg_0_start[1]
    cdef double x4 = lseg_0_end[0]
    cdef double y4 = lseg_0_end[1]
    cdef double intersect_x, intersect_y

    if ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)) == 0:
        # Either this or the other line segment had zero length
        return None

    cdef double uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))
    cdef double uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))

    if uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1:
        # collision
        intersect_x = x1 + (uA * (x2 - x1))
        intersect_y = y1 + (uA * (y2 - y1))
        return np.array([intersect_x, intersect_y, 0])
    else:
        return None

cdef cnp.ndarray _get_point_lseg_intersect_or_end(cnp.ndarray[double, ndim=1] lseg_start,
                                                    cnp.ndarray[double, ndim=1] lseg_end,
                                                    cnp.ndarray[double, ndim=1] point):
    """
    Calls '_get_closest_point_on_line_to_p_2d()' and returns the returned point or, if the function call 
    returns None, the end of the line segment that is closest to the given point
    """
    cdef cnp.ndarray[double, ndim=1] x = _get_closest_point_on_line_to_p_2d(lseg_start,
                                                                            lseg_end,
                                                                            point)
    if x is None:
        if _dist_3d(lseg_start, point) < _dist_3d(lseg_end, point):
            return lseg_start
        else:
            return lseg_end
    else:
        return x

# +++ Ray cdef functions

cdef cnp.ndarray _get_shortest_line_btw_rays(cnp.ndarray[double, ndim=1] ray_0_start,
                                              cnp.ndarray[double, ndim=1] ray_0_grad,
                                              cnp.ndarray[double, ndim=1] ray_1_start,
                                              cnp.ndarray[double, ndim=1] ray_1_grad):
    """
    Finds the points on the rays where the line segment and ray are closest.
    May be start points of ray or later.
    If ray and line are parallel, return the start points of rays.
    :return: LineSegment containing 2 points.
    First point on this ray, second point on given line.
    """

    if ray_0_grad[2] == 0:
        ray_0_grad[2] = 0.0001
    if ray_1_grad[2] == 0:
        ray_1_grad[2] = 0.0001

    cdef cnp.ndarray[double, ndim=1] p_diff = ray_1_start - ray_0_start

    cdef double n1 = (p_diff[0] * ray_0_grad[0] + p_diff[1] * ray_0_grad[1] + p_diff[2] * ray_0_grad[2])
    cdef double n2 = (p_diff[0] * ray_1_grad[0] + p_diff[1] * ray_1_grad[1] + p_diff[2] * ray_1_grad[2])
    n1 = -n1
    n2 = -n2

    cdef double s1 = ray_1_grad[0] * ray_0_grad[0] + ray_1_grad[1] * ray_0_grad[1] + ray_1_grad[2] * ray_0_grad[2]
    cdef double r1 = -ray_0_grad[0] * ray_0_grad[0] - ray_0_grad[1] * ray_0_grad[1] - ray_0_grad[2] * ray_0_grad[2]
    cdef double s2 = ray_1_grad[0] * ray_1_grad[0] + ray_1_grad[1] * ray_1_grad[1] + ray_1_grad[2] * ray_1_grad[2]
    cdef double r2 = -ray_0_grad[0] * ray_1_grad[0] - ray_0_grad[1] * ray_1_grad[1] - ray_0_grad[2] * ray_1_grad[2]

    cdef double s_scale = s2 / s1
    cdef double n1_scaled = n1 * s_scale
    cdef double s1_scaled = s1 * s_scale
    cdef double r1_scaled = r1 * s_scale

    cdef double n_ = n1_scaled - n2
    cdef double r_ = r1_scaled - r2

    cdef double r = n_ / r_
    cdef double s = (n1 - (r1 * r)) / s1

    cdef double[:] A = ray_0_start + (ray_0_grad * r)
    cdef double[:] B = ray_1_start + (ray_1_grad * s)

    # check points are not behind start points
    cdef cnp.ndarray[double, ndim=1] t = A - ray_0_start
    cdef cnp.ndarray[double, ndim=1] u = B - ray_1_start
    if ray_0_grad[0] != 0 and ray_0_grad[1] != 0 and ray_0_grad[2] != 0 and not _is_equal(t[0] / ray_0_grad[0], t[1] / ray_0_grad[1], t[2] / ray_0_grad[2]):
        A = ray_0_start
    if ray_1_grad[0] != 0 and ray_1_grad[1] != 0 and ray_1_grad[2] != 0 and not _is_equal(u[0] / ray_1_grad[0], u[1] / ray_1_grad[1], u[2] / ray_1_grad[2]):
        B = ray_1_start

    return np.vstack((A, B))


cdef cnp.ndarray _get_shortest_line_btw_ray_and_line(cnp.ndarray[double, ndim=1] ray_0_start,
                                                     cnp.ndarray[double, ndim=1] ray_0_grad,
                                                     cnp.ndarray[double, ndim=1] lseg_start,
                                                     cnp.ndarray[double, ndim=1] lseg_end): # -> LineSegment or None
    """Finds shortest line between given seg and this ray or None"""
    # turn seg into ray
    cdef cnp.ndarray[double, ndim=1] ray_1_grad = lseg_start - lseg_end
    # get shortest dist between rays
    cdef cnp.ndarray[double, ndim=2] res = _get_shortest_line_btw_rays(ray_0_start, ray_0_grad, lseg_start, ray_1_grad)
    # check out of bounds for segment
    if not _is_point_btw(lseg_start, lseg_end, res[1]):
        return None
    return res

cdef bint _is_point_on_ray(cnp.ndarray[double, ndim=1] ray_start,
                           cnp.ndarray[double, ndim=1] ray_gradient,
                           cnp.ndarray[double, ndim=1] point): # -> bool
    """Checks if a given point lies on this ray"""
    cdef cnp.ndarray[double, ndim=1] point_gradient = point - ray_start # x2-x1, y2-y1, z2-z1

    return np.all(np.sign(ray_gradient) == np.sign(point_gradient)) and \
        _dist_3d(_as_unit_vector(ray_gradient), _as_unit_vector(point_gradient))  < 0.01

# +++ General cdef functions

cdef bint _is_equal(double a, double b, double c, double tolerance=0.001):
    """checks if floats a, b, c are within tolerance of each other"""
    if math.isnan(a) or math.isnan(b) or math.isnan(c):
        return True
    return abs(a-b) < tolerance and abs(a-c) < tolerance

# classes

cdef class LineSegment:
    """Line segment with start and end points"""
    def __cinit__(self, Node start, Node end):
        self.start = start
        self.end = end

    cpdef Ray to_ray(self):
        return Ray(self.start, self.end - self.start)

    cpdef double length_2d(self):
        return self.start.dist_2d(self.end)

    cpdef double length_3d(self):
        return self.start.dist_3d(self.end)

    cpdef Node get_point_closest_to_p(self, Node point): # -> Node or None
        cdef cnp.ndarray[double, ndim=1] res = _get_closest_point_on_line_to_p_2d(self.start.as_ndarray(),
                                                                                  self.end.as_ndarray(),
                                                                                  point.as_ndarray())
        """Get point on line closest to given point or None"""
        if res is None:
            return None
        return Node.from_mem_slice(res)

    cpdef double get_distance_point_to_line(self, Node point): # -> float
        cdef double res = _get_distance_point_to_line(self.start.as_ndarray(),
                                                      self.end.as_ndarray(),
                                                      point.as_ndarray())
        return res

    cpdef Node get_intersect_with_lseg(self, LineSegment other): # -> Node or None
        """gets intersect of this and other line segment, None if they dont meet"""
        cdef cnp.ndarray[double, ndim=1] res = _get_intersect_btw_lsegs_2d(self.start.as_ndarray(),
                                                                           self.end.as_ndarray(),
                                                                           other.start.as_ndarray(),
                                                                           other.end.as_ndarray())

        if res is None:
            return None
        return Node.from_mem_slice(res)

    cpdef Node get_point_lineseg_intersect_or_end(self, Node p): # -> Node
        cdef cnp.ndarray[double, ndim=1] res = _get_point_lseg_intersect_or_end(self.start.as_ndarray(),
                                                                                self.end.as_ndarray(),
                                                                                p.as_ndarray())
        return Node.from_mem_slice(res)


cdef class Ray:
    """Ray with start point and 3D gradient"""

    def __cinit__(self, Node start, Node gradient):
        self.start = start
        self.gradient = gradient

    cpdef bint is_point_on_ray(self, Node point): # -> bool
        return _is_point_on_ray(self.start.data, self.gradient.data, point.data)

    cpdef LineSegment get_shortest_line_to_seg(self, LineSegment lseg): # -> LineSegment or None
        """Finds shortest line between given seg and this ray or None"""
        cdef cnp.ndarray[double, ndim=2] res = _get_shortest_line_btw_ray_and_line(self.start.as_ndarray(),
                                                                                   self.gradient.as_ndarray(),
                                                                                   lseg.start.as_ndarray(),
                                                                                   lseg.end.as_ndarray())
        if res is None:
            return None
        return LineSegment(Node.from_mem_slice(res[0]), Node.from_mem_slice(res[1]))

    cpdef LineSegment get_shortest_line_to_ray(self, Ray ray): #  -> LineSegment
        """
        Finds the points on the rays where they are closest.
        May be start points of ray or later.
        If rays are parallel, returns the start points of rays.
        :param ray:
        :return: LineSegment containing 2 points.
        First point on this ray, second point on given ray.
        """
        cdef cnp.ndarray[double, ndim=2] res = _get_shortest_line_btw_rays(self.start.as_ndarray(),
                                                                       self.gradient.as_ndarray(),
                                                                       ray.start.as_ndarray(),
                                                                       ray.gradient.as_ndarray())

        return LineSegment(Node.from_mem_slice(res[0]), Node.from_mem_slice(res[1]))
