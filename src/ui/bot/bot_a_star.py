from typing import Tuple, List

import pygame, math
import numpy as np
import heapq

from src.ui.front_bot import Bot
from src.ui.utils import COLOR, MATH

SAFE_DIST = 5
STEP = 10

class Bot_A_Star(Bot):
    def __init__(self, position: (int, int) = (25, 25)):
        super(Bot_A_Star, self).__init__(position)

        self.velocity = (1,1)

    def reconstruct_path(self, cameFrom, current):
        path = [current]
        while current in cameFrom:
            current = cameFrom[current]
            path.append(current)
        path.reverse()
        return path

    def get_min_dist_of_obst(self, node):
        closest_o_point = min(self.visible_obstacles, key=lambda o_point: MATH.dist_2d(o_point, node))
        return MATH.dist_2d(closest_o_point, node)

    def get_neighbors(self, node):
        neighbors = [MATH.add(node, (STEP,0)), MATH.add(node, (0,STEP)), MATH.add(node, (-STEP,0)), MATH.add(node, (0,-STEP)),
                     MATH.add(node, (STEP,STEP)), MATH.add(node, (-STEP,STEP)), MATH.add(node, (-STEP,-STEP)), MATH.add(node, (STEP,-STEP))]
        return list(
            map(
                lambda i2: tuple(i2),
                filter(
                    lambda i1: self.get_min_dist_of_obst(i1) >= self.radius + SAFE_DIST,
                    neighbors
                )
            )
        )

    def a_star(self, start, goal, h):
        open_set = set()
        open_heap = []
        came_from = {}

        g_score_dict = {start: 0} # default value of Infinity
        def g_score(key):
            return g_score_dict.get(key, math.inf)

        f_score_dict = {start: h(start)}
        def f_score(key):
            return f_score_dict.get(key, math.inf)

        open_set.add(start)
        open_heap.append((0, start))

        while len(open_set) > 0:
            # node in openSet having the lowest fScore[] value
            current = min(open_set, key=lambda n: f_score(n))
            current = heapq.heappop(open_heap)[1]
            # list(map(lambda n: f_score(n), open_set))
            # current = open_set[open_set_f_scores.index(min(open_set_f_scores))]
            if MATH.dist_2d(goal, current) < STEP:
                return self.reconstruct_path(came_from, current)

            open_set.remove(current)

            # for each neighbor of current
            for neighbor in self.get_neighbors(current):
                # d(current,neighbor) is the weight of the edge from current to neighbor
                # tentative_gScore is the distance from start to the neighbor through current
                tentative_gScore = g_score(current) + MATH.dist_2d(current, neighbor)
                if tentative_gScore < g_score(neighbor):
                    # This path to neighbor is better than any previous one. Record it!
                    came_from[neighbor] = current
                    g_score_dict[neighbor] = tentative_gScore
                    f_score_dict[neighbor] = g_score(neighbor) + h(neighbor)
                    if not (neighbor in open_set):
                        open_set.add(neighbor)
                        heapq.heappush(open_heap, (f_score(neighbor), neighbor))
        return []

    def update(self, goal: Tuple[int, int], visible_obstacles: List[Tuple[int, int]]):
        self.visible_obstacles = visible_obstacles
        future_path = self.a_star(tuple(self.center), goal, lambda n: MATH.dist_2d(n, goal))
        if len(future_path) > 0:
            self.velocity = MATH.sub(future_path[1], self.center)
        super(Bot_A_Star, self).update(goal, visible_obstacles)

