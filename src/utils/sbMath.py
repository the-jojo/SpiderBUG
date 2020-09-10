import math
import numpy as np

from src.geom.Node import Node


def reduce_path(path, res):
    """
    Reduces path resolution by taking only every resth point
    :param res: resolution or resulting path
    :param path: list or numpy array with path points
    :return: reduced path
    """
    y = path[0::int(len(path) / min(res, len(path)))]
    y.append(path[-1])
    return y


def calc_angle(a: np.ndarray, b: np.ndarray, c: np.ndarray):
    """
    Gets an angle in radians between points a,b,c
    a - - - - b
        °   /
          /
        c
    :param a: outer point 1
    :param b: middle point
    :param c: outer point 2
    :return: angle in radians
    """
    ang = math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0])
    return ang


def rotate(origin: np.ndarray, point: np.ndarray, angle: float):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return np.array([qx, qy])


def angle_between(v1: np.ndarray, v2: np.ndarray):
    """Calculates the angle between 2 vectors around the orgin"""
    return calc_angle(v1, np.array([0, 0]), v2)


def dist_2d(x1, y1, x2, y2):
    """
    calculates euclidian distance between two points
    :param x1: point 1 x
    :param y1: point 1 y
    :param x2: point 2 x
    :param y2: point 2 y
    :return: scalar distance
    """
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist


def dist_2d_p(start, end):
    """
    calculates euclidian distance between two points
    :return: scalar distance
    """
    dist = dist_2d(start[0], start[1], end[0], end[1])
    return dist


def is_point_behind_ref(point: [float], ref: [float],
                        goal: [float], removal_dist):
    d_ref = dist_2d_p(ref, goal)
    d = dist_2d_p(point, goal)
    return d - d_ref > removal_dist * 5


def get_point_towards(point: np.ndarray, heading: float, distance: float) -> np.ndarray:
    length = distance * math.cos(heading)
    height = distance * math.sin(heading)
    return np.array([point[0] + length, point[1] + height])


def is_path_point_sensible(path_point, past_path, removal_dist):
    p_0 = Node.from_list(past_path[0])
    for p_1 in past_path[1:]:
        p_1 = Node.from_list(p_1)
        d_t = max(p_0.dist_2d(p_1), removal_dist)
        if p_1.dist_2d(path_point) < d_t:
            return False
        p_0 = p_1
    return True
