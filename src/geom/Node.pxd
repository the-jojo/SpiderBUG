# cython: profile=True

cimport numpy as cnp

cdef fused F_Node:
    double[:]
    Node

cdef cnp.ndarray _as_unit_vector(cnp.ndarray[double, ndim=1])

cdef cnp.ndarray _as_2d(cnp.ndarray[double, ndim=1])

cdef cnp.ndarray _as_2d(cnp.ndarray[double, ndim=1])

cdef double _dist_3d(cnp.ndarray[double, ndim=1],
                     cnp.ndarray[double, ndim=1])

cdef double _dist_2d(cnp.ndarray[double, ndim=1],
                     cnp.ndarray[double, ndim=1])

cdef int _is_point_btw(cnp.ndarray[double, ndim=1],
                       cnp.ndarray[double, ndim=1],
                       cnp.ndarray[double, ndim=1])


cpdef bint _eq(cnp.ndarray[double, ndim=1] a, cnp.ndarray[double, ndim=1] b)

cpdef Node toN(t)

cdef class Node:
    cdef public double[:] data

    cpdef double x(self, double x=?)
    cpdef double y(self, double y=?)
    cpdef double z(self, double z=?)
    #cpdef S_Node get_data(self)
    cpdef cnp.ndarray as_ndarray(self)
    cpdef cnp.ndarray as_ndarray_2d(self)
    cpdef list as_list_2d(self)
    cpdef list as_list_3d(self)
    cpdef Node as_node_2d(self)
    cpdef Node as_unit_vector(self)
    cpdef double mag(self)
    # @staticmethod
    # cpdef Node from_array(cnp.ndarray cls)
    # @staticmethod
    # cpdef Node from_list(list cls)
    # @staticmethod
    # cpdef bint is_point_btw(Node a, Node b, Node point, tolerance=?)
    cpdef double dist_2d(self, other)
    cpdef double dist_3d(self, other)
    #cpdef __add__(self, other)
    #cpdef __sub__(self, other)
    #cpdef __truediv__(self, double other)
    #cpdef __mul__(self, double other)
    #cpdef __str__(self)
    #cpdef __hash__(self)
    #cpdef __eq__(self, other)
