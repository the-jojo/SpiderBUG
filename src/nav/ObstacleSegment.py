import math
import time

import numpy as np

from src.geom.Lines import Ray, LineSegment
from src.geom.Node import Node
from src.geom.obstacleUtils import _get_intersect_with_path_3d


class ObstacleSegment:
    """
    Represents visible segment of an obstacle
    Comprised of sensed obstacle boundary points, predicted velocity and a geometric center.
    Change in points of discontinuity/edge boundary points over time is recorded as a velocity. A
    mean over all velocities is used as the overall predicted obstacle velocity. It is assumed that
    obstacles have a constant speed and velocity, meaning that the predicted velocity will increase
    in accuracy over time.
    """

    def __init__(self, obst_id, sensed_points):
        self.id = obst_id
        self.boundary_points = sensed_points
        self.center = Node.from_array(np.mean(np.array([n.as_ndarray() for n in self.boundary_points]), axis=0))
        self.velocity = [np.array([0, 0, 1])]
        self.avg_v_3d = None
        self.last_update_time = time.time()

    def __find_corresponding__(self, new_sensed_points, tolerance):
        """Returns points belonging to same obstacle as this; first set of points where a point matched past ones"""
        if len(new_sensed_points) < 1:
            return None
        for obst_points in new_sensed_points:
            for obst_point in obst_points:
                for self_point in self.boundary_points:
                    if self_point.dist_2d(obst_point) < tolerance:
                        return obst_points
        return None

    def update(self, new_sensed_points, was_pasued):
        """
        Updates this obstacle's boundary points and velocity if new points include points near old ones.
        Called by sbPerception module. If None is returned, this obstacleSegment will be automatically discarded.
        :return: used boundary points if they were close to old ones or None if none were used
        """
        new_time = time.time()

        # find new points belonging to same ObstacleSegment
        update_tol = max((new_time - self.last_update_time) * 0.1, 0.15)
        new_boundary_points = self.__find_corresponding__(new_sensed_points, update_tol)

        if new_boundary_points is None:
            # no new points were close enough to old points; This obstacle went out of view
            return None

        # update this obstacle's velocity only if the simulation was not paused, since pausing will throw off
        # the time measurement which is critical to determining velocities
        if not was_pasued:
            new_v_0 = new_boundary_points[0] - self.boundary_points[0]
            new_v_0.z(new_time - self.last_update_time)
            new_v_1 = new_boundary_points[-1] - self.boundary_points[-1]
            new_v_1.z(new_time - self.last_update_time)

            # add measured velocity as difference in vectors of new boundary points to old
            self.velocity.append(new_v_0.as_ndarray())
            self.velocity.append(new_v_1.as_ndarray())

        # set new boundary points
        self.boundary_points = new_boundary_points
        
        # set center
        self.center = Node.from_array(np.mean(np.array([n.as_ndarray() for n in self.boundary_points]), axis=0))

        # set new update time
        self.last_update_time = new_time

        return new_boundary_points

    def get_avg_v_3d(self):
        """returns the mean velocity with values confined to known constraints"""
        n = Node.from_array(np.mean(self.velocity, axis=0)).as_unit_vector()
        if n.x() < -0.1:
            n.x(-0.1)
        elif n.x() > 0.1:
            n.x(0.1)
        if n.y() < -0.1:
            n.y(-0.1)
        elif n.y() > 0.1:
            n.y(0.1)
        return n

    def get_center(self):
        return self.center

    def get_boundary_points(self):
        """Returns every 20th boundary point or all if there are 20 or less points"""
        y = self.boundary_points[0::int(len(self.boundary_points) / min(20, len(self.boundary_points)))]
        y.append(self.boundary_points[-1])
        return y

    def get_tangent_points_3d(self, rob_cone, tangent_dist_out, tangent_dist_in, logger):
        """
        Finds the tangent points of this obstacle in 3D given a robot-cone and tangent-distances.
        :param rob_cone: robot's cone of movement
        :param tangent_dist_out: outer tangent distance
        :param tangent_dist_in: inner tangent distance
        :param logger: logger
        :return: list of nodes
        """
        res = []
        center = self.get_center()
        avg_v = self.get_avg_v_3d()
        discontinuity_points = [self.boundary_points[0], self.boundary_points[-1]]

        """ 
        loops the 2 discontinuity points
        for each 
            create a ray with start point the point and gradient the obstacle's estimated velocity. 
            Then finds the intersect between the ray and the cone. 
                Any intersects further than 10 units away from the cone's minimum point are thrown away, 
                since they are outside the scene's configuration space. 
            For each intersect
                Find tangent points and add the one further away from the obstacle's center to the res list to return 
        """
        for p_node in discontinuity_points:
            p_ray = Ray(p_node, avg_v)
            p_ints = rob_cone.get_intersect_with_ray(p_ray)
            if len(p_ints) == 0:
                continue
            for p_int_3d in p_ints:
                if p_int_3d.dist_2d(rob_cone.min) > 10:
                    # skip irrelevant intersects
                    continue
                p_int_2d = p_int_3d.as_node_2d()
                new_center = center + (p_int_2d - p_node)
                rob_dist = p_int_2d.dist_2d(rob_cone.min)

                t_dist = min(rob_dist, tangent_dist_out)
                t_dist = max(t_dist, tangent_dist_in)
                if rob_dist == t_dist:
                    t_dist -= 0.01
                if tangent_dist_out >= rob_dist > tangent_dist_in:
                    # robot inside outer tangent dist
                    if logger.tangent_warning_outer <= 0:
                        logger.warning("ObstSeg.get_tangent_points_3d(): Robot inside outer tangent distance!")
                        logger.tangent_warning_outer = 5
                elif rob_dist <= tangent_dist_in:
                    if logger.tangent_warning_inner <= 0:
                        logger.critical("ObstSeg.get_tangent_points_3d(): Robot inside inner tangent distance!")
                        logger.tangent_warning_inner = 5
                    t_dist = rob_dist - 0.01

                # robot outside circle
                th = math.acos(t_dist / rob_dist)
                d = math.atan2(rob_cone.min.y() - p_int_2d.y(),
                               rob_cone.min.x() - p_int_2d.x())
                d1 = d + th
                d2 = d - th

                t1 = Node.from_list([p_int_2d.x() + t_dist * math.cos(d1),
                                     p_int_2d.y() + t_dist * math.sin(d1)])
                t2 = Node.from_list([p_int_2d.x() + t_dist * math.cos(d2),
                                     p_int_2d.y() + t_dist * math.sin(d2)])

                # only add tangent points further away from the center (outer points)
                # (unless there is only one point of discontinuity)
                t1.z(rob_cone.min.z() + (rob_cone.min.dist_2d(t1) / rob_cone.gradient))
                t2.z(rob_cone.min.z() + (rob_cone.min.dist_2d(t2) / rob_cone.gradient))
                if len(self.boundary_points) == 1:
                    # add both points
                    if t1.dist_2d(rob_cone.min) <= 10:
                        res.append(t1)
                    if t2.dist_2d(rob_cone.min) <= 10:
                        res.append(t2)
                elif new_center.dist_2d(t1) < new_center.dist_2d(t2):
                    # t1 is closer
                    if t2.dist_2d(rob_cone.min) <= 10:
                        res.append(t2)
                else:
                    # t2 is closer
                    if t1.dist_2d(rob_cone.min) <= 10:
                        res.append(t1)

        return res

    def get_intersect_with_path_3d(self, rob_pos_3d, path_2d, rob_speed, tangent_dist_in, only_hard_int=False):
        """
        Finds an intersect between the path and this obstacle's boundary.
        Further details in @_get_intersect_with_path_3d()
        """
        if rob_pos_3d is None or path_2d is None or len(path_2d) == 0 or rob_speed is None:
            return None
        path_2d = [rob_pos_3d] + path_2d
        return _get_intersect_with_path_3d(np.array([op.as_ndarray() for op in self.get_boundary_points()]),
                                           self.get_avg_v_3d().as_ndarray(),
                                           np.array([op.as_ndarray() for op in path_2d]),
                                           rob_speed, tangent_dist_in, only_hard_int)

    def get_tangent_points_2d(self, cur_pos, tangent_dist_out, tangent_dist_in, logger):
        """Finds the tangent points of this obstacle in 2D given the robot's position and tangent-distances.
        :param cur_pos: robot's current position
        :param tangent_dist_out: outer tangent distance
        :param tangent_dist_in: inner tangent distance
        :param logger: logger
        :return: list of nodes
        """
        res = []

        c_p = self.get_center()
        # loop points of discontinuity
        for d_p in [self.boundary_points[0], self.boundary_points[-1]]:
            rob_point_dist = cur_pos.dist_2d(d_p)
            t_dist = min(rob_point_dist, tangent_dist_out)
            t_dist = max(t_dist, tangent_dist_in)
            if rob_point_dist == t_dist:
                t_dist -= 0.01
            if tangent_dist_out >= rob_point_dist > tangent_dist_in:
                # robot inside outer tangent dist
                if logger.tangent_warning_outer <= 0:
                    logger.warning("ObstSeg.get_tangent_points_2d(): Robot inside outer tangent distance!")
                    logger.tangent_warning_outer = 5
            elif rob_point_dist <= tangent_dist_in:
                if logger.tangent_warning_inner <= 0:
                    logger.critical("ObstSeg.get_tangent_points_2d(): Robot inside inner tangent distance!")
                    logger.tangent_warning_inner = 5
                t_dist = rob_point_dist - 0.01

            th = math.acos(t_dist / rob_point_dist)
            d = math.atan2(cur_pos.y() - d_p.y(), cur_pos.x() - d_p.x())
            d1 = d + th
            d2 = d - th
            t1 = Node.from_list([d_p.x() + t_dist * math.cos(d1),
                      d_p.y() + t_dist * math.sin(d1)])
            t2 = Node.from_list([d_p.x() + t_dist * math.cos(d2),
                      d_p.y() + t_dist * math.sin(d2)])
            # only add outer tangent points further away from center
            if c_p.dist_2d(t1) < c_p.dist_2d(t2):
                # t1 is closer
                res.append(t2)
            else:
                res.append(t1)
        if len(res) == 0:
            logger.critical("ObstSeg.get_tangent_points_2d(): No tangent points found!")
            raise RuntimeError("No tangent points found on obstacle")
        return res

    def get_intersect_with_lseg(self, l_seg, tangent_dist_out, tangent_dist_in):
        """Finds the closest intersect between line and this obstacle"""
        intersects = []

        # loop boundary points
        prev_b_p = self.boundary_points[-1]
        for b_p in self.get_boundary_points():
            intersect = l_seg.get_intersect_with_lseg(LineSegment(prev_b_p, b_p))
            if intersect is None:
                # check closeness
                i1 = l_seg.get_point_closest_to_p(b_p)  # : Node or None
                if i1 is not None and i1.dist_2d(b_p) < tangent_dist_out:
                    intersects.append(i1)
            else:
                # direct collision
                intersects.append(intersect)
            prev_b_p = b_p
        # return intersect closest to lSeg start
        if len(intersects) > 0:
            min_dist = math.inf
            cls_ints = None
            for intersect in intersects:
                d = intersect.dist_2d(l_seg.start)
                if d <= min_dist:
                    min_dist = d
                    cls_ints = intersect
            return cls_ints
        else:
            return None
