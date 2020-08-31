cimport numpy as cnp
import numpy as np
import math
from src.geom.Cone cimport Cone
from src.geom.BoundedPlane cimport BoundedPlane
from src.geom.BoundedPlane import BoundedPlane
from src.geom.Lines cimport Ray, LineSegment
from src.geom.Lines import Ray, LineSegment
from src.geom.Node cimport Node, F_Node
from src.geom.Node import eq

def p_t(F_Node n):
    if type(n) is memoryview:
        print("mem slice")
    if type(n) is Node:
        print("node")
    print("--")

cpdef Node _get_intersect_with_path_3d(list obst_boundary_points, Node avg_v_3d, Node rob_pos_3d, list path_2d, double rob_speed, double tangent_dist_in): #  -> Node or None
    """Path starting with rob_pos_3d continuing to path_2d"""
    """global last_p
    cdef Ray projected_bp_ray
    cdef LineSegment closest_seg
    cdef BoundedPlane plane
    cdef Node v0, v1, intersect
    last_p = rob_pos_3d
    
    cdef list path_3d = [to_line_segments(p, rob_speed) for p in path_2d if not last_p.as_node_2d() == p.as_node_2d()]

    if len(obst_boundary_points) == 1:
        # check if path passes too close to boundaryPoint ray
        projected_bp_ray = Ray(Node.from_list(path_2d[0]), avg_v_3d)
        for p_seg in path_3d:
            closest_seg = projected_bp_ray.get_shortest_line_to_seg(p_seg)
            if closest_seg is not None and closest_seg.length_3d() < tangent_dist_in:
                return closest_seg.end
    else:
        # loop boundary points
        v0 = obst_boundary_points[-1]
        for v1 in obst_boundary_points:
            plane = BoundedPlane(v0, v1, avg_v_3d)

            for p_seg in path_3d:
                intersect = plane.get_intersect_with_lseg(p_seg)

                if intersect is not None:
                    # path directly crossed into obstacle
                    return intersect
                else:
                    # check if path passes too close to boundaryPoint ray
                    projected_bp_ray = Ray(v0, avg_v_3d)
                    closest_seg = projected_bp_ray.get_shortest_line_to_seg(p_seg)
                    if closest_seg is not None and closest_seg.length_3d() < tangent_dist_in:
                        return closest_seg.end
            v0 = v1"""
    #print("a", obst_boundary_points[0])
    # cdef double[:] a = obst_boundary_points[0].data
    # cdef double[:] b =  Node.from_mem_slice(obst_boundary_points[0].data).data
    # print("a", a)
    # print("b", b)
    # p_t(a)
    # p_t(obst_boundary_points[0])
    # #p_t(a)
    #
    # print("e1", a[0] == b[0] and a[1] == b[1] and a[2] == b[2])
    # print("e1", a[0] is b[0] and a[1] is b[1] and a[2] is b[2])
    #print("b", Node.from_mem_slice(obst_boundary_points[0].data))
    #print("b", <S_Node> obst_boundary_points[0].get_data())
    #print("c", type(obst_boundary_points[0].data))
    #print("c", type(<S_Node> obst_boundary_points[0].get_data()))
    #print("d", type(obst_boundary_points[0].x()))
    #print("e", type(obst_boundary_points[0].data[0]))

    cdef double[:, :] a_obst_boundary_points = np.array([op.data for op in obst_boundary_points])
    cdef double[:, :] a_path_2d = np.array([op.data for op in path_2d])

    cdef Node intersect
    cdef double[:] p0, p1, v0, v1
    cdef LineSegment pSeg, closestLSeg
    cdef Ray projected_bp_ray
    cdef BoundedPlane plane
    cdef double dist
    
    # loop each path segment p0 - p1
    p0 = rob_pos_3d.data
    for p1 in a_path_2d:
        if not eq(p0, p1):
            pass
            
            # continue path in 3d
            dist = math.sqrt((p1[0] - p0[0]) ** 2. + (p1[1] - p0[1]) ** 2.)
            p1[2] = p0[2] + dist / rob_speed  # set z = t = d/s

            # make 3d Line segment from path points
            pSeg = LineSegment(Node.from_mem_slice(p0), Node.from_mem_slice(p1))

            if len(a_obst_boundary_points) == 1:
                # check if path passes too close to boundaryPoint ray
                projected_bp_ray = Ray(a_obst_boundary_points[0], avg_v_3d)
                closestLSeg = projected_bp_ray.get_shortest_line_to_seg(pSeg)
                if closestLSeg is not None and closestLSeg.length_3d() < tangent_dist_in:
                    return closestLSeg.end
            else:
                # loop boundary points
                v0 = a_obst_boundary_points[-1]
                for v1 in a_obst_boundary_points:
                    plane = BoundedPlane(Node.from_mem_slice(v0), Node.from_mem_slice(v1), avg_v_3d)

                    intersect = plane.get_intersect_with_lseg(pSeg)
                    if intersect is not None:
                        # path directly crossed into obstacle
                        return intersect
                    else:
                        pass
                        """
                        # check if path passes too close to boundaryPoint ray
                        projected_bp_ray = Ray(Node.from_mem_slice(v0), avg_v_3d)
                        closestLSeg = projected_bp_ray.get_shortest_line_to_seg(pSeg)
                        if closestLSeg is not None and closestLSeg.length_3d() < tangent_dist_in:
                            return closestLSeg.end"""
                    v0 = v1
            p0 = p1
    return None