import math

import numpy as np

from src.bot.ObstacleSegment import ObstacleSegment
from src.geom.Lines import LineSegment
from src.geom.Node import Node
from src.utils.DubinsPath import find_path_complete
from src.utils.config import Config
from src.utils.modes import Dir, NavMode
from src.utils.sbMath import get_point_towards, calc_angle, rotate, angle_between

logger = None
config_ = Config()

# should be ok
class BfWaypoint:
    def __init__(self, pos: Node, obst_to_follow: ObstacleSegment, orig_pos=None): # TODO large discrepency btw original pos and updated pos
        self.orig_pos = pos
        self.cur_pos = orig_pos if orig_pos is not None else pos

        _, self.dist_to_obst = self.__get_closest_to_obst__(obst_to_follow, pos)

    def update(self, obst_to_follow: ObstacleSegment):
        shortest_intersect, shortest_dist = self.__get_closest_to_obst__(obst_to_follow, self.cur_pos.as_node_2d())
        if shortest_dist is None:
            raise RuntimeError("obstacle with 0 boundary points should not exist")
        tmp = shortest_intersect + Node.from_list([1, 0])
        h = calc_angle(tmp.as_list_2d(), shortest_intersect.as_list_2d(), self.cur_pos.as_list_2d())
        self.cur_pos = Node.from_array(get_point_towards(shortest_intersect.as_ndarray(), h, self.dist_to_obst))

    def get_pos_change(self):
        return self.orig_pos.as_node_2d() - self.cur_pos.as_node_2d()

    @staticmethod
    def __get_closest_to_obst__(obstacle: ObstacleSegment, cur_pos: Node):
        shortest_dist = math.inf
        shortest_intersect = None
        if len(obstacle.boundary_points) == 1:
            return obstacle.boundary_points[0], obstacle.boundary_points[0].dist_2d(cur_pos)
        if len(obstacle.boundary_points) == 0:
            return None, math.inf
        v1 = obstacle.boundary_points[-1]
        for v2 in obstacle.get_boundary_points():
            if v1 == v2:
                continue
            ls = LineSegment(v1, v2)
            d = ls.get_distance_point_to_line(cur_pos)

            if d < shortest_dist:
                shortest_dist = d
                shortest_intersect = ls.get_point_lineseg_intersect_or_end(cur_pos)
            v1 = v2
        return shortest_intersect, shortest_dist


d_reach, v_diff, d_followed_rel = 0., 0., 0.
v_followed: Node or None = None
bf_waypoint: BfWaypoint or None = None
obst_id_to_follow: float = math.nan
obst_dir: Dir or None = None


def find_obst(id, all_obst):
    # find updated obstacle with obst_id_to_follow
    foo = np.vectorize(lambda t: t.id == id)

    # check if we lost the obstacle
    if len([o for o in all_obst if o.id == id]) == 0 or foo(all_obst).size == 0:
        return None
    return [o for o in all_obst if o.id == id][0]


# works
def get_dir_of_obst(cur_pos: Node, cur_heading: float, obst_to_follow: ObstacleSegment) -> Dir:
    """Returns the direction of nearest obstacle boundary point"""
    # find closest obstacle boundary point
    closest_dist = math.inf
    closest_bp = obst_to_follow.boundary_points[0]
    for bp in obst_to_follow.get_boundary_points():
        d = cur_pos.dist_2d(bp)
        if d <= closest_dist:
            closest_bp = bp
            closest_dist = d

    a = cur_pos.as_node_2d()
    b = cur_pos + Node.from_array(rotate(np.array([0, 0]), np.array([1, 0]), cur_heading))
    p = closest_bp.as_node_2d()

    # make a origin
    b = b - a
    p = p - a
    c_p = np.cross(b.as_ndarray_2d(), p.as_ndarray_2d())
    if np.all(c_p > 0):
        return Dir.LEFT
    else:
        return Dir.RIGHT


# works
def get_dir_of_obst_fut(obst_to_follow: ObstacleSegment, cur_pos: Node, next_pos: Node) -> Dir:
    """Finds the direction of an obstacle in the future"""
    h = angle_between(np.array([1, 0]), (next_pos - cur_pos).as_ndarray_2d())
    return get_dir_of_obst(next_pos, h, obst_to_follow)


# works
def init(obstacles: [ObstacleSegment], cur_pos: Node, cur_heading: float, goal_pos: Node,
         logger_, config) -> bool:
    """Initializes BF if needed - finds obstacle id to follow and selects bfWaypoint if none"""
    global obst_id_to_follow, obst_dir, bf_waypoint, logger, config_
    logger = logger_
    config_ = config

    obst_to_follow = None
    if not math.isnan(obst_id_to_follow):
        obst_to_follow = find_obst(obst_id_to_follow, obstacles)
    if obst_to_follow is None:
        if config_.ROB_MODEL >= 1:
            # sphero moves differently
            path_to_check = [cur_pos, goal_pos]
        else:
            path_to_check = find_path_complete(cur_pos, cur_heading, [goal_pos], config_.TURN_RADIUS,
                                           step_size=config_.PATH_RES_QUICK)
        # find obstacle to follow as nearest obstacle
        nearest_intersect_dist = math.inf
        for obstacle in obstacles:
            intersect = obstacle.get_intersect_with_path_3d(cur_pos, path_to_check, config_.ROB_SPEED,
                                                        config_.TANGENT_DIST_INNER)
            if intersect is not None:
                d = cur_pos.dist_2d(intersect)
                if d <= nearest_intersect_dist:
                    nearest_intersect_dist = d
                    obst_to_follow = obstacle
                    obst_id_to_follow = obstacle.id
        if obst_to_follow is None:
            # did not find an obstacle
            # TODO handle this exception
            logger.warning("BF.init(): Could not find an obstacle to follow")
            return False

    if obst_dir is None:
        obst_dir = get_dir_of_obst(cur_pos, cur_heading, obst_to_follow)

    if bf_waypoint is None:
        bf_waypoint = find_new_bf_waypoint(obst_to_follow, cur_pos)
        if bf_waypoint is None:
            return False
    return True


def find_new_bf_waypoint(obst_to_follow: ObstacleSegment, cur_pos: Node, orig_pos=None) -> BfWaypoint or None:
    """Selects new BfWaypoint from tangent points in that keep obstacle in the current direction"""
    global config_
    # find new tangent points
    tangent_points = obst_to_follow.get_tangent_points_2d(cur_pos, config_.TANGENT_DIST_OUTER, config_.TANGENT_DIST_INNER, logger)
    bf_waypoint_candidates = []

    # choose tangent point which continues in same direction
    for tp in tangent_points:
        next_obst_dir = get_dir_of_obst_fut(obst_to_follow, cur_pos, tp)
        if next_obst_dir == obst_dir:
            bf_waypoint_candidates.append(tp)

    if len(bf_waypoint_candidates) == 0:
        logger.critical("BF.find_new_bf_waypoint(): No tangent points found in current direction!")
        return None
        #raise RuntimeError("BF.find_new_bf_waypoint(): No tangent points found in current direction!")
    else:
        if len(bf_waypoint_candidates) > 1:
            logger.info("there were " + str(len(bf_waypoint_candidates)) + " bf_waypoint candidates.")
        # select tangent point as bf_waypoint on condition
        return BfWaypoint(bf_waypoint_candidates[0], obst_to_follow, orig_pos)


# should be ok
def plan(cur_pos: Node, goal_pos: Node, cur_heading: float, new_obst_segments: [ObstacleSegment]) \
        -> (NavMode, [Node]):
    """
    Boundary-Following Behaviour - continue in same direction around an obstacle.
    Choose bf waypoint in forward semicircle
    :param cur_pos: current robot position
    :param goal_pos: goal position
    :param cur_heading: current robot heading
    :param new_obst_segments: sensed obstacle segments
    :return: (nextMode, path)
    """
    global d_reach, v_followed, v_diff, d_followed_rel, obst_id_to_follow, bf_waypoint
    # find updated obstacle with obst_id_to_follow
    obst_to_follow = find_obst(obst_id_to_follow, new_obst_segments)

    # check if we lost the obstacle
    if obst_to_follow is None:
        # TODO remember followed obstacles in case you loose sight of them and then they reappear
        logger.info("BF.plan(): Lost obstacle segment; End of Routine")
        return NavMode.MTG, None
    else:
        # update BF Waypoint
        bf_waypoint.update(obst_to_follow)

        # Calculate d_reach, d_followed, v_diff, d_followed_rel
        v_diff = bf_waypoint.get_pos_change()
        d_reach = goal_pos.dist_2d(bf_waypoint.cur_pos)
        if v_followed is None or v_diff is None:
            print("whoaa")
        v_followed_rel = v_followed + v_diff
        d_followed_rel = v_followed_rel.dist_2d(goal_pos)

        # TODO check if we passed bfPoint?

        # if d_reach < d_followed_rel
        if d_followed_rel - d_reach > config_.D_TOL:
            # switch to MTG
            logger.info("BF: path length decreased by %.2f below original minimum; End of Routine"
                        % (d_followed_rel - d_reach))
            return NavMode.MTG, None
        else:
            # choose new BF waypoint
            bf_waypoint = find_new_bf_waypoint(obst_to_follow, cur_pos, None)
            if bf_waypoint is None:
                return NavMode.MTG, None

            # generate path to bfWaypoint
            path = [bf_waypoint.cur_pos, goal_pos]

            # update v_followed
            v_followed = v_followed_rel

            return NavMode.BF, path


# works
def reset():
    global obst_id_to_follow, bf_waypoint, v_followed, obst_dir
    obst_id_to_follow = math.nan
    bf_waypoint = None
    v_followed = None
    obst_dir = None
