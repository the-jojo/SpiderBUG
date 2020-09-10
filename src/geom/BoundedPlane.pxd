cimport numpy as cnp

from src.geom.Node cimport Node
from src.geom.Lines cimport LineSegment, Ray

# cdef functions

cdef cnp.ndarray _get_plane_normal(cnp.ndarray[double, ndim=1],
                                   cnp.ndarray[double, ndim=1],
                                   cnp.ndarray[double, ndim=1])

cdef double _get_plane_constant(cnp.ndarray[double, ndim=1],
                                cnp.ndarray[double, ndim=1],
                                cnp.ndarray[double, ndim=1])

cdef cnp.ndarray _get_intersect_btw_plane_and_ray(cnp.ndarray[double, ndim=1],
                                        cnp.ndarray[double, ndim=1],
                                        cnp.ndarray[double, ndim=1],
                                        cnp.ndarray[double, ndim=1],
                                        double,
                                        cnp.ndarray[double, ndim=1],
                                        cnp.ndarray[double, ndim=1])

cdef cnp.ndarray _get_intersect_btw_plane_and_lseg(cnp.ndarray[double, ndim=1],
                                        cnp.ndarray[double, ndim=1],
                                        cnp.ndarray[double, ndim=1],
                                        cnp.ndarray[double, ndim=1],
                                        double plane_constant,
                                        cnp.ndarray[double, ndim=1],
                                        cnp.ndarray[double, ndim=1])

# class

cdef class BoundedPlane:
    cdef public Node lower_a, lower_b, plane_direction, normal
    cdef public double constant
    cpdef Node get_intersect_with_lseg(self, LineSegment lseg)
    cpdef Node get_intersect_with_ray(self, Ray ray)
