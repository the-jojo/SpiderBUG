from src.geom.Node cimport Node

cpdef Node _get_intersect_with_path_3d(list obst_boundary_points, Node avg_v_3d, Node rob_pos_3d, list path_2d, double rob_speed, double tangent_dist_in)