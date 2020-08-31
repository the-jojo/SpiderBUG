# cython: profile=True

cimport numpy as cnp
from src.geom.Node cimport Node

# Line functions

cdef cnp.ndarray _get_closest_point_on_line_to_p_2d(cnp.ndarray[double, ndim=1],
                                                 cnp.ndarray[double, ndim=1],
                                                 cnp.ndarray[double, ndim=1])

cdef double _get_distance_point_to_line(cnp.ndarray[double, ndim=1],
                                        cnp.ndarray[double, ndim=1],
                                        cnp.ndarray[double, ndim=1])

cdef cnp.ndarray _get_intersect_btw_lsegs_2d(cnp.ndarray[double, ndim=1],
                                             cnp.ndarray[double, ndim=1],
                                             cnp.ndarray[double, ndim=1],
                                             cnp.ndarray[double, ndim=1])

cdef cnp.ndarray _get_point_lseg_intersect_or_end(cnp.ndarray[double, ndim=1],
                                                    cnp.ndarray[double, ndim=1],
                                                    cnp.ndarray[double, ndim=1])

# Ray founctions

cdef cnp.ndarray _get_shortest_line_btw_rays(cnp.ndarray[double, ndim=1], cnp.ndarray[double, ndim=1],
                                              cnp.ndarray[double, ndim=1], cnp.ndarray[double, ndim=1])

cdef cnp.ndarray _get_shortest_line_btw_ray_and_line(cnp.ndarray[double, ndim=1],
                                                     cnp.ndarray[double, ndim=1],
                                                     cnp.ndarray[double, ndim=1],
                                                     cnp.ndarray[double, ndim=1])

cdef bint _is_point_on_ray(cnp.ndarray[double, ndim=1],
                           cnp.ndarray[double, ndim=1],
                           cnp.ndarray[double, ndim=1])

# classes

cdef class LineSegment:
    cdef public Node start, end

    cpdef Ray to_ray(self)

    cpdef double length_2d(self)

    cpdef double length_3d(self)

    cpdef Node get_point_closest_to_p(self, Node point)

    cpdef double get_distance_point_to_line(self, Node point)

    cpdef Node get_intersect_with_lseg(self, LineSegment other)

    cpdef Node get_point_lineseg_intersect_or_end(self, Node p)


cdef class Ray:
    cdef public Node start, gradient

    cpdef bint is_point_on_ray(self, Node point)

    cpdef LineSegment get_shortest_line_to_seg(self, LineSegment lseg)

    cpdef LineSegment get_shortest_line_to_ray(self, Ray ray)