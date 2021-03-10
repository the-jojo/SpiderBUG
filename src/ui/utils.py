import math
import numpy as np
from typing import Tuple


class COLOR:
    RED = (255, 0, 0),
    GREEN = (0, 149, 0),
    BLACK = (0, 0, 0),
    WHITE = (255,255,255),
    LIGHT_GREEN = (204,255,229)
    GREY = (70,70,70)

class MATH:
    def sub(a, b):
        return (np.array(a) - np.array(b)).tolist()

    def add(a, b):
        return (np.array(a) + np.array(b)).tolist()

    def mult(a, B):
        return (np.array(a) * B).tolist()

    def to_unit_vector(v: Tuple[float, float]):
        if v == (0,0):
            return [0.,0.]
        return (np.array(v) / np.linalg.norm(np.array(v))).tolist()

    def dist_2d(p1, p2):
        """
        calculates euclidian distance between two points
        :param x1: point 1 x
        :param y1: point 1 y
        :param x2: point 2 x
        :param y2: point 2 y
        :return: scalar distance
        """
        x1, y1 = p1
        x2, y2 = p2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def linesAreParallel(point_1, point_2, point_3, point_4):
        """ Return True if the given lines point_1-point_2 and
            point_3-point_4 are parallel """
        x1,y1 = point_1
        x2,y2 = point_2
        x3,y3 = point_3
        x4,y4 = point_4
        return ((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)) == 0

    def is_point_btw(a, b, p):
        return (MATH.dist_2d(a, p) + MATH.dist_2d(p , b)) - MATH.dist_2d(a , b) <= 0