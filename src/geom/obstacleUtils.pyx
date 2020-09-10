cimport numpy as cnp
import numpy as np
import math

from src.geom.BoundedPlane cimport _get_plane_normal, _get_plane_constant, _get_intersect_btw_plane_and_lseg
from src.geom.Node cimport Node, _dist_3d, _dist_2d
from src.geom.Lines cimport _get_shortest_line_btw_ray_and_line
from src.geom.Node import Node

cdef bint exit_early(cnp.ndarray[double, ndim=2] a_path_2d,
                     cnp.ndarray[double, ndim=2] a_obst_boundary_points,
                     cnp.ndarray[double, ndim=1] a_avg_v,
                     double tangent_dist_in,
                     bint only_hard_int):
    """
    Roughly checks if a given path and obstacle boundary intersect or come near each other.
    Abstracts both the path and boundary into 2 sub-parts down the middle. Then checks for 
    intersections between each boundary-part and path-part combination. Also uses 
    _get_shortest_line_btw_ray_and_line() between each of the sub-parts to check for 
    closeness.
    :return: True if intersect or closeness exists, otherwise False
    """
    cdef cnp.ndarray[double, ndim=1] p0 = a_path_2d[0]
    cdef cnp.ndarray[double, ndim=1] p1 = a_path_2d[int(len(a_path_2d)/2)]
    cdef cnp.ndarray[double, ndim=1] p2 = a_path_2d[-1]

    cdef cnp.ndarray[double, ndim=1] v0 = a_obst_boundary_points[0]
    cdef cnp.ndarray[double, ndim=1] v1 = a_obst_boundary_points[int(len(a_obst_boundary_points)/2)]
    cdef cnp.ndarray[double, ndim=1] v2 = a_obst_boundary_points[-1]

    cdef cnp.ndarray[double, ndim=1] plane_norm = _get_plane_normal(v0, v2, a_avg_v)
    cdef double plane_const = _get_plane_constant(v0, v2, plane_norm)
    cdef cnp.ndarray[double, ndim=1] plane_norm_0 = _get_plane_normal(v0, v1, a_avg_v)
    cdef double plane_const_0 = _get_plane_constant(v0, v1, plane_norm_0)
    cdef cnp.ndarray[double, ndim=1] plane_norm_1 = _get_plane_normal(v1, v2, a_avg_v)
    cdef double plane_const_1 = _get_plane_constant(v1, v2, plane_norm_1)

    cdef cnp.ndarray[double, ndim=2] closest_seg_0, closest_seg_1, closest_seg_2
    if not only_hard_int:
        # get shortest line btw obstacle rays and path linesgements
        closest_seg_0 = _get_shortest_line_btw_ray_and_line(v0, a_avg_v, p0, p1)
        closest_seg_1 = _get_shortest_line_btw_ray_and_line(v1, a_avg_v, p0, p1)
        closest_seg_2 = _get_shortest_line_btw_ray_and_line(v2, a_avg_v, p0, p1)
        if (not closest_seg_0 is None and _dist_3d(closest_seg_0[0], closest_seg_0[1]) < tangent_dist_in) or \
                (not closest_seg_1 is None and _dist_3d(closest_seg_1[0], closest_seg_1[1]) < tangent_dist_in) or \
                (not closest_seg_2 is None and _dist_3d(closest_seg_2[0], closest_seg_2[1]) < tangent_dist_in):
            return False
        closest_seg_0 = _get_shortest_line_btw_ray_and_line(v0, a_avg_v, p1, p2)
        closest_seg_1 = _get_shortest_line_btw_ray_and_line(v1, a_avg_v, p1, p2)
        closest_seg_2 = _get_shortest_line_btw_ray_and_line(v2, a_avg_v, p1, p2)
        if (not closest_seg_0 is None and _dist_3d(closest_seg_0[0], closest_seg_0[1]) < tangent_dist_in) or \
                (not closest_seg_1 is None and _dist_3d(closest_seg_1[0], closest_seg_1[1]) < tangent_dist_in) or \
                (not closest_seg_2 is None and _dist_3d(closest_seg_2[0], closest_seg_2[1]) < tangent_dist_in):
            return False

    # get intersect with line segment
    if  _get_intersect_btw_plane_and_lseg(v0, v2, plane_norm, a_avg_v,
                                            plane_const, p0, p1) is not None or \
        _get_intersect_btw_plane_and_lseg(v0, v2, plane_norm, a_avg_v,
                                            plane_const, p1, p2) is not None or \
        _get_intersect_btw_plane_and_lseg(v0, v2, plane_norm, a_avg_v,
                                            plane_const, p0, p2) is not None:
        return False
    if  _get_intersect_btw_plane_and_lseg(v0, v1, plane_norm_0, a_avg_v,
                                            plane_const_0, p0, p1) is not None or \
        _get_intersect_btw_plane_and_lseg(v0, v1, plane_norm_0, a_avg_v,
                                            plane_const_0, p1, p2) is not None or \
        _get_intersect_btw_plane_and_lseg(v0, v1, plane_norm_0, a_avg_v,
                                            plane_const_0, p0, p2) is not None:
        return False
    if  _get_intersect_btw_plane_and_lseg(v1, v2, plane_norm_1, a_avg_v,
                                            plane_const_1, p0, p1) is not None or \
        _get_intersect_btw_plane_and_lseg(v1, v2, plane_norm_1, a_avg_v,
                                            plane_const_1, p1, p2) is not None or \
        _get_intersect_btw_plane_and_lseg(v1, v2, plane_norm_1, a_avg_v,
                                            plane_const_1, p0, p2) is not None:
        return False
    return True


#@cython.boundscheck(False)
cpdef Node _get_intersect_with_path_3d(cnp.ndarray obst_boundary_points,
                                       cnp.ndarray[double, ndim=1] avg_v_3d,
                                       cnp.ndarray path_2d,
                                       double rob_speed, double tangent_dist_in,
                                       bint only_hard_int):
    """
    Finds an intersect between an obstacle boundary and a path which must be 2 or more points.
    Algorithmic optimisation techniques are indicated by '+++' and explained in the code.
    [Presorting points] First sorts path-obstacle points by ascending distance to each other
    [Early exit] Check if it is worth continuing
    [Plane caching] 
    :param obst_boundary_points: 
    :param avg_v_3d: 
    :param path_2d: 
    :param rob_speed: 
    :param tangent_dist_in: 
    :param only_hard_int: 
    :return: 
    """
    if len(obst_boundary_points) <= 1 or len(path_2d) <= 1:
        return None

    # explicitly specify dimensions for faster processing
    cdef cnp.ndarray[double, ndim=2] a_obst_boundary_points = obst_boundary_points
    cdef cnp.ndarray[double, ndim=2] a_path_2d = path_2d
    cdef cnp.ndarray[double, ndim=1] a_avg_v = avg_v_3d

    # declare all local variables
    cdef cnp.ndarray[double, ndim=1] p0, p1, v0, v1, lower_a, lower_b, plane_dir, plane_norm, intersect
    cdef cnp.ndarray closestLSeg
    cdef double dist, plane_const
    cdef int i,j, t_i, t_j

    """ 
    +++ presort path-obstacle points by ascending distance
    - compute z component of points in a_path_2d extending the path into 3D space-time
    - make a matrix of path-obstacle distances and assign into dist_matrix array
    - sort x-y indeces pairs of dist_matrix in ascending order
    """
    # compute z component of points
    p0 = a_path_2d[0]
    for i in range(1, len(a_path_2d)):
        dist = math.sqrt((a_path_2d[i][0] - p0[0]) ** 2. + (a_path_2d[i][1] - p0[1]) ** 2.)
        a_path_2d[i][2] = p0[2] + (dist / rob_speed)  # set z = t = d/s
        p0 = a_path_2d[i]

    # make matrix of distances
    cdef cnp.ndarray[double, ndim=2] dist_matrix = np.zeros([len(a_path_2d), len(a_obst_boundary_points)])
    for i in range(len(a_path_2d)):
        for j in range(len(a_obst_boundary_points)):
            dist_matrix[i, j] = _dist_2d(a_path_2d[i], a_obst_boundary_points[j])

    # clever sorting of indices
    cdef cnp.ndarray[long long, ndim=2] sorted_indices = np.dstack(np.unravel_index(np.argsort(dist_matrix.ravel()), (len(a_path_2d), len(a_obst_boundary_points))))[0]

    """
    +++ Early exit if points too far apart and no rough intersect exists
    - check if the closest distance in the dist_matrix is greater than the tangent distance given
    - only if it is, then check roughly if path and obstacle boundary intersect or come near each other
    """
    # check if points are too far apart and rough estimates do not intersect
    if dist_matrix[sorted_indices[0, 0], sorted_indices[0, 1]] > tangent_dist_in and exit_early(a_path_2d, a_obst_boundary_points, a_avg_v, tangent_dist_in, only_hard_int):
        return None

    """
    +++ Plane caching
    - do not recompute plane normals and plane constants since they use numpy dot products to calculate
    """
    cdef cnp.ndarray[double, ndim=2] plane_norm_l = np.repeat(math.nan, len(a_obst_boundary_points)*3).reshape(len(a_obst_boundary_points), 3)
    cdef cnp.ndarray[double, ndim=1] plane_const_l = np.repeat(math.nan, len(a_obst_boundary_points))

    # loop obstacle points and path points in ascending order by distance using presorted indeces.
    for i in range(len(sorted_indices)):
        t_i = sorted_indices[i, 0] # path
        t_j = sorted_indices[i, 1] # boundary_points
        if t_i == 0:
            continue # cannot go backwards from first path point. We'll check from the second one later
        p0 = a_path_2d[t_i - 1]
        p1 = a_path_2d[t_i]
        v0 = a_obst_boundary_points[t_j - 1]
        v1 = a_obst_boundary_points[t_j]
        # make plane
        lower_a = v0
        lower_b = v1
        plane_dir = a_avg_v
        if not math.isnan(plane_const_l[t_j]):
            plane_norm = plane_norm_l[t_j]
            plane_const = plane_const_l[t_j]
        else:
            plane_norm = _get_plane_normal(lower_a, lower_b, plane_dir)
            plane_const = _get_plane_constant(lower_a, lower_b, plane_norm)
            plane_norm_l[t_j] = plane_norm
            plane_const_l[t_j] = plane_const
        # get intersect with line segment
        intersect = _get_intersect_btw_plane_and_lseg(lower_a, lower_b, plane_norm, plane_dir,
                                                      plane_const, p0, p1)
        if intersect is None and not only_hard_int:
            closestLSeg = _get_shortest_line_btw_ray_and_line(v1, a_avg_v, p0, p1)
            if not closestLSeg is None and _dist_3d(closestLSeg[0], closestLSeg[1]) < tangent_dist_in:
                return Node.from_mem_slice(closestLSeg[1])
        elif not intersect is None:
            return Node.from_mem_slice(intersect)
    return None