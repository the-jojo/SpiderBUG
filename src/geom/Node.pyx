import math

import numpy as np
cimport numpy as cnp

cdef cnp.ndarray _as_2d(cnp.ndarray[double, ndim=1] a):
    """Returns a 1D ndarray with x and y elements from a and z as 0"""
    cdef cnp.ndarray[double, ndim=1] b = np.array([a[0], a[1], 0.])
    return b

cdef cnp.ndarray _as_unit_vector(cnp.ndarray[double, ndim=1] a):
    """normalises the given ndarray a to unit length"""
    cdef cnp.ndarray[double, ndim=1] b = a / np.linalg.norm(a)
    return b

cdef double _dist_3d(cnp.ndarray[double, ndim=1] a,
                     cnp.ndarray[double, ndim=1] b):
    """Returns the 3D distance between ndarrays a and b representing 2 3D points"""
    return math.sqrt((b[0] - a[0]) ** 2. + (b[1] - a[1]) ** 2. + (b[2] - a[2]) ** 2.)

cdef double _dist_2d(cnp.ndarray[double, ndim=1] a,
                     cnp.ndarray[double, ndim=1] b):
    """Returns the 2D distance between ndarrays a and b representing 2 2D points"""
    return math.sqrt((b[0] - a[0]) ** 2. + (b[1] - a[1]) ** 2.)


cdef int _is_point_btw(cnp.ndarray[double, ndim=1] a,
                       cnp.ndarray[double, ndim=1] b,
                       cnp.ndarray[double, ndim=1] point):
    """Checks if the given ndarray point lies between a and b with a tolerance of 0.1"""
    cdef double d_apb = _dist_3d(a, point) + _dist_3d(b, point)
    cdef double d_ab = _dist_3d(a, b)
    return abs(d_ab - d_apb) < 0.1

cpdef bint _eq(cnp.ndarray[double, ndim=1] a, cnp.ndarray[double, ndim=1] b):
    """checks if all elements of a and b are equal"""
    return a[0] is b[0] and a[1] is b[1] and a[2] is b[2]

cpdef Node toN(t):
    """converts a given value of type {Node, ndarray, tuple, list or memory-slice} to Node"""
    if type(t) is Node:
        return <Node> t
    if type(t) is cnp.ndarray:
        return Node.from_array(<cnp.ndarray[double]> t)
    if type(t) is tuple:
        return Node.from_tuple(<tuple> t)
    if type(t) is list:
        return Node.from_list(<list> t)
    return Node.from_mem_slice(t)

cdef class Node:
    """
    Wrapper for numpy arrays. Forces float values of elements
    Allows hashing and equality.
    Allows conversion to 2d or 3d formats
    """
    def __cinit__(self, double x=0., double y=0., double z=0.):
        self.data = np.array((<double>x, <double>y, <double>z))

    cpdef double x(self, double x=math.nan):
        if not math.isnan(x):
            self.data[0] = x
        return self.data[0]

    cpdef double y(self, double y=math.nan):
        if not math.isnan(y):
            self.data[1] = y
        return self.data[1]

    cpdef double z(self, double z=math.nan):
        if not math.isnan(z):
            self.data[2] = z
        return self.data[2]

    cpdef cnp.ndarray as_ndarray(self):
        return np.array([self.x(), self.y(), self.z()])

    cpdef cnp.ndarray as_ndarray_2d(self):
        return np.array([self.x(), self.y()])

    cpdef list as_list_2d(self):
        return [self.x(), self.y()]

    cpdef list as_list_3d(self):
        return [self.x(), self.y(), self.z()]

    cpdef Node as_node_2d(self):
        return Node(self.x(), self.y(), 0)

    cpdef Node as_unit_vector(self):
        return Node.from_array(self.as_ndarray() / np.linalg.norm(self.as_ndarray()))

    cpdef double mag(self):
        return np.linalg.norm(self.as_ndarray())

    @staticmethod
    def from_array(cnp.ndarray[double] cls):
        obj = np.asarray(cls).astype(float).copy()
        obj.resize(3, refcheck=False)
        return Node(obj[0], obj[1], obj[2])

    @staticmethod
    def from_tuple(tuple cls):
        if len(cls) < 2:
            raise TypeError("Given list needs at least 2 entries for x and y")
        if len(cls) > 2:
            return Node(cls[0], cls[1], cls[2])
        else:
            return Node(cls[0], cls[1], 0)

    @staticmethod
    def from_list(list cls):
        if len(cls) < 2:
            raise TypeError("Given list needs at least 2 entries for x and y")
        if len(cls) > 2:
            return Node(cls[0], cls[1], cls[2])
        else:
            return Node(cls[0], cls[1], 0)

    @staticmethod
    def from_mem_slice(double[:] cls):
        if len(cls) < 2:
            raise TypeError("Given list needs at least 2 entries for x and y")
        if len(cls) > 2:
            return Node(cls[0], cls[1], cls[2])
        else:
            return Node(cls[0], cls[1], 0)

    @staticmethod
    def is_point_btw(Node a, Node b, Node point, tolerance=0.1):
        cdef double d_apb = a.dist_3d(point) + b.dist_3d(point)
        cdef double d_ab = a.dist_3d(b)
        return abs(d_ab - d_apb) < tolerance

    cpdef double dist_2d(self, other):
        if type(other) is Node:
            return math.sqrt((other.x() - self.x()) ** 2. + (other.y() - self.y()) ** 2.)
        elif type(other) is list:
            return self.dist_2d(Node.from_list(other))
        elif type(other) is cnp.ndarray:
            return self.dist_2d(Node.from_array(other))
        else:
            raise TypeError("Other's type is not supported")

    cpdef double dist_3d(self, other):
        if type(other) is Node:
            return math.sqrt((other.x() - self.x()) ** 2. + (other.y() - self.y()) ** 2. + (other.z() - self.z()) ** 2.)
        elif type(other) is list:
            return self.dist_3d(Node.from_list(other))
        elif type(other) is cnp.ndarray:
            return self.dist_3d(Node.from_array(other))
        else:
            raise TypeError("Other's type is not supported")

    def __add__(self, other):
        if other is None:
            return self
        if type(other) is Node:
            return Node((other.x() + self.x()), (other.y() + self.y()), (other.z() + self.z()))
        elif type(other) is list:
            return self.__add__(Node.from_list(other))
        elif type(other) is cnp.ndarray:
            return self.__add__(Node.from_array(other))
        else:
            raise TypeError("Other's type is not supported")

    def __sub__(self, other):
        if other is None:
            return self
        if type(other) is Node:
            return Node(self.x() - other.x(), self.y() - other.y(), self.z() - other.z())
        elif type(other) is list:
            return self.__sub__(Node.from_list(other))
        elif type(other) is cnp.ndarray:
            return self.__sub__(Node.from_array(other))
        else:
            raise TypeError("Other's type is not supported")

    def __truediv__(self, double other):
        return Node(self.x() / other, self.y() / other, self.z() / other)

    def __mul__(self, double other):
        return Node(self.x() * other, self.y() * other, self.z() * other)

    def __str__(self):
        return "{" + str(self.x()) + ";" + str(self.y()) + ";" + str(self.z()) + "}"

    def __hash__(self):
        return hash(self.__str__())

    def __eq__(self, other):
        if type(other) is list:
            other = Node.from_list(other)
        elif type(other) is tuple:
            other = Node.from_tuple(other)
        elif type(other) is cnp.ndarray:
            other = Node.from_array(other)
        return self.__hash__() == other.__hash__()

    def __getstate__(self):
        return self.x(), self.y(), self.z()

    def __setstate__(self, state):
        self.data[0], self.data[1], self.data[2] = state
