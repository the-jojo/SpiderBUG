import math

import dubins

from src.geom.Node import Node
from src.utils.config import default_config


def find_path(pos_0: Node, heading_0: float, pos_1: Node, pos_2: Node or None, turn_radius: float,
              step_size=default_config['PATH_RES'], resolution=math.pi / 64):
    """Finds the shortest dubins path that passes through the first 2 path points"""
    config_0 = (*pos_0.as_list_2d(), heading_0)
    pos_2 = pos_2.as_list_2d() if pos_2 is not None else None
    path_0, path_1 = dubins.shortest_paths_2(config_0, pos_1.as_list_2d(), pos_2, turn_radius, resolution)

    points, t = path_0.sample_many(step_size)
    if path_1 is not None:
        points_, t_ = path_1.sample_many(step_size)
        points = points + points_
        t = t + t_
    return points, t


def find_path_complete(pos_0: Node, heading_0: float, path_points: [Node], turn_radius: float,
                       step_size=default_config['PATH_RES']):
    assert len(path_points) > 0
    pos_1 = path_points[0]
    pos_2 = path_points[1] if len(path_points) > 1 else None
    points, _ = find_path(pos_0, heading_0, pos_1, pos_2, turn_radius, step_size)
    points = [Node.from_tuple(p) for p in points]
    for path_p in path_points[2:]:
        points.append(path_p)
    return points


def find_path_weight(pos_0: Node, heading_0: float, path_points: [Node], turn_radius: float, resolution=math.pi / 64):
    config_0 = (*pos_0.as_list_2d(), heading_0)
    assert len(path_points) > 0
    pos_1 = path_points[0].as_list_2d()
    pos_2 = path_points[1].as_list_2d() if len(path_points) > 1 else None
    path_0, path_1 = dubins.shortest_paths_2(config_0, pos_1, pos_2, turn_radius, resolution)
    length = path_0.path_length()
    if path_1 is not None:
        length += path_1.path_length()
    if len(path_points) > 2:
        p0 = path_points[2]
        for p1 in path_points[3:]:
            length += p0.dist_2d(p1)
            p0 = p1

    return length
