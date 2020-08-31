import math

from src.bot.ObstacleSegment import ObstacleSegment
from src.geom.Node import Node
from src.nav.Navigation import DynamicWeb
from src.utils.config import Config
from src.utils.modes import NavMode
from src.utils.sbMath import reduce_path

web: DynamicWeb or None = None
h_last: float = math.inf
logger = None
config_ = Config()


def evaluate_path(cur_pos: Node, path: [Node]) -> float:
    """Returns the path distance in straight 2d lines from cur_pos"""
    if path is None:
        return math.inf
    length = 0
    v1 = cur_pos
    for v2 in path:
        length += v1.dist_2d(v2)
        v1 = v2

    return length


def init(cur_pos: Node, cur_heading: float, goal_pos: Node, logger_, config):
    """initializes MTG, creating web if necessary"""
    global web, logger, config_
    config_ = config
    logger = logger_
    if web is None:
        # init
        web = DynamicWeb(cur_pos, goal_pos, cur_heading, config_.ROB_SPEED, logger_)


# ok
def check_path_obstruction(cur_pos: Node, path: [Node], obstacles: [ObstacleSegment],
                           rob_speed: float) -> bool:
    """Checks if the path is obstructed by any obstacle"""
    global config_
    for obst in obstacles:
        v1 = obst.get_intersect_with_path_3d(cur_pos, path, rob_speed, config_.TANGENT_DIST_INNER)
        if v1 is not None:
            return True
    return False


# ok
def update_web(cur_pos: Node, cur_heading: float, new_obstacles: [ObstacleSegment], robot_speed: float, fut_path: [Node],
               tangent_dist_out: float, tangent_dist_in: float, turn_radius: float):
    """update-web subprocedure - if path is obstructed then it moves rob in web and updates connections and paths"""
    global web, config_

    path_to_check = reduce_path(fut_path, 100) \
        if len(fut_path) > 0 and fut_path[-1].as_node_2d() == web.goal_pos.as_node_2d() \
        else web.cur_path

    debug_b = 0

    # check each obstacle if it blocks the current path
    if debug_b or web.cur_path is None or check_path_obstruction(cur_pos, path_to_check, new_obstacles, robot_speed):
        # update pos of robot in web
        web.move_rob_to(cur_pos, cur_heading)

        # update web with new obstacles
        web.update(new_obstacles, cur_pos, robot_speed, tangent_dist_out, tangent_dist_in, config_.NODE_REMOVAL_DIST,
                   config_.ROB_RADIUS)

        # update shortest path
        web.cur_path = web.update_shortest_path(turn_radius, config_.ROB_MODEL >= 1)
    else:
        # remove near nodes in web path
        web.rem_near_nodes_from_path(cur_pos, config_.NODE_REMOVAL_DIST)


def plan(cur_pos: Node, cur_heading: float, rob_speed: float, goal_pos: Node, new_obstacles: [ObstacleSegment],
         fut_path: [Node]) -> (NavMode, [Node]):
    """updates web if necessary, returns current best path or None if exit-condition is met"""
    global web, h_last

    # update web
    update_web(cur_pos, cur_heading, new_obstacles, rob_speed, fut_path, config_.TANGENT_DIST_OUTER,
               config_.TANGENT_DIST_INNER, config_.TURN_RADIUS)

    # get current path
    path = web.cur_path

    # check exit condition
    h = evaluate_path(cur_pos, path)

    if math.isinf(h) or h - h_last > config_.H_TOL:
        # switch to BF
        logger.info("MTG: path length increased by %.2f from last time" %(h - h_last))
        if len(new_obstacles) > 0:
            logger.info("MTG: End of Routine")
            return NavMode.BF, None
        else:
            logger.info("MTG: No obstacle to follow - staying in MTG")
    h_last = h
    return NavMode.MTG, path


def reset():
    global web, h_last
    web = None
    h_last = math.inf
