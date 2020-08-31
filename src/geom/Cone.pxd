# cython: profile=True

cimport numpy as cnp

from src.geom.Node cimport Node
from src.geom.Lines cimport Ray

cdef list get_intersect_with_ray(cnp.ndarray[double, ndim=1],
                                        double,
                                        cnp.ndarray[double, ndim=1],
                                        cnp.ndarray[double, ndim=1])

cdef class Cone:
    cdef public Node min
    cdef public double gradient

    cpdef list get_intersect_with_ray(self, Ray ray)
