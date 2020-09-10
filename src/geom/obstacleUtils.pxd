cimport numpy as cnp
from src.geom.Node cimport Node


cpdef Node _get_intersect_with_path_3d(cnp.ndarray,
                                       cnp.ndarray[double, ndim=1],
                                       cnp.ndarray,
                                       double,
                                       double,
                                       bint)