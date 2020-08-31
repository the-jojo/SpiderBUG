import math

from src.geom_old.Utils import *


def is_equal(a: float, b: float, c: float, tolerance=0.001) -> bool:
    if math.isnan(a) or math.isnan(b) or math.isnan(c):
        return True
    return abs(a-b) < tolerance and abs(a-c) < tolerance


class LineSegment:
    def __init__(self, start: Node, end: Node):
        self.start = start
        self.end = end

    def to_ray(self):
        return Ray(self.start, self.end - self.start)

    def length(self):
        return dist_2d_p(self.start, self.end)

    def get_point_closest_to_p(self, point: Node) -> Node or None:
        """Get point on line closest to given point or None"""
        k = (((self.end[1] - self.start[1]) * (point[0] - self.start[0]) - (self.end[0] - self.start[0])
              * (point[1] - self.start[1])) / (math.pow(self.end[1] - self.start[1], 2)
              + math.pow(self.end[0] - self.start[0], 2)))
        x4 = point[0] - k * (self.end[1] - self.start[1])
        y4 = point[1] + k * (self.end[0] - self.start[0])
        intersect = Node([x4, y4])
        if is_point_btw(self.start, self.end, intersect): # TODO as2d()??
            return intersect
        else:
            return None

    def get_distance_point_to_line(self, point) -> float:
        x = self.get_point_closest_to_p(point)
        if x is not None:
            # x is on line segment
            return x.dist_2d(point)
        else:
            return min(dist_2d_p(self.start, point), dist_2d_p(self.end, point))

    def get_intersect_with_lseg(self, other) -> Node or None:
        """gets intersect of this and other line segment, None if they dont meet"""
        x1 = other.start[0]
        y1 = other.start[1]
        x2 = other.end[0]
        y2 = other.end[1]
        x3 = self.start[0]
        y3 = self.start[1]
        x4 = self.end[0]
        y4 = self.end[1]
        if ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)) == 0:
            print("aaah")
        uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))
        uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))

        if uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1:
            # collision
            intersect_x = x1 + (uA * (x2 - x1))
            intersect_y = y1 + (uA * (y2 - y1))
            return Node([intersect_x, intersect_y])
        else:
            return None

    def get_point_lineseg_intersect_or_end(self, p) -> Node:
        k = (((self.end[1] - self.start[1]) * (p[0] - self.start[0]) - (self.end[0] - self.start[0]) * (p[1] - self.start[1])) /
             (math.pow(self.end[1] - self.start[1], 2) + math.pow(self.end[0] - self.start[0], 2)))
        x4 = p[0] - k * (self.end[1] - self.start[1])
        y4 = p[1] + k * (self.end[0] - self.start[0])
        intersect = Node([x4, y4])
        if is_point_btw(self.start, self.end, intersect):
            return intersect
        if dist_2d_p(self.start, p) < dist_2d_p(self.end, p):
            return Node(self.start)
        else:
            return Node(self.end)


class Ray:
    def __init__(self, start, gradient):
        self.start = Node(start).as3d()
        self.gradient = gradient

    def is_point_on_ray(self, point) -> bool: # TODO doesnt work
        """Checks if a given point lies on this ray"""

        """p0 = self.start
        p1 = self.start + self.gradient
        if point == p0 or point == p1:
            return True

        try:
            a1 = (p1[1]-p0[1]) / (p1[0]-p0[0])
            a2 = (p1[1]-point[1]) / (p1[0]-point[0])
            b1 = (p1[2] - p0[2]) / (p1[0] - p0[0])
            b2 = (p1[2] - point[2]) / (p1[0] - point[0])
            return a1 == a2 and b1 == b2 and is_point_behind_ref()
        except RuntimeWarning as w:
            return False"""
        if abs(self.gradient[0]) < 0.01 and abs(self.gradient[1]) < 0.01:
            return abs(point[0] - self.start[0]) < 0.01 and abs(point[1] - self.start[1]) < 0.01 and np.sign((point - self.start)[2]) == np.sign(self.gradient[2])
        if abs(self.gradient[0]) == 0.01:
            return abs((point - self.start)[0]) < 0.01 and np.all(np.sign(point - self.start) == np.sign(self.gradient))
        if abs(self.gradient[1]) == 0.01:
            return abs((point - self.start)[1]) < 0.01 and np.all(np.sign(point - self.start) == np.sign(self.gradient))

        a = math.nan if self.gradient[0] == 0 else point[0] / self.gradient[0]
        b = math.nan if self.gradient[1] == 0 else point[1] / self.gradient[1]
        c = math.nan if self.gradient[2] == 0 else point[2] / self.gradient[2]
        # TODO have to be same sign as gradient too
        return is_equal(a, b, c)

    def get_shortest_line_to_seg(self, lseg: LineSegment) -> LineSegment or None:
        """Finds shortest line between given seg and this ray or None"""
        # turn seg into ray
        ray = lseg.to_ray()
        # get shortest dist between rays
        res = self.get_shortest_line_to_ray(ray)
        # check out of bounds for segment
        if not is_point_btw(lseg.start, lseg.end, res.end):
            return None
        return res

    def get_shortest_line_to_ray(self, ray) -> LineSegment:
        """
        Finds the points on the rays where they are closest.
        May be start points of ray or later.
        If rays are parallel, returns the start points of rays.
        :param ray:
        :return: LineSegment containing 2 points.
        First point on this ray, second point on given ray.
        """
        p0 = self.start
        g0 = self.gradient
        p1 = ray.start
        g1 = ray.gradient

        p_diff = p1 - p0

        n1 = (p_diff[0] * g0[0] + p_diff[1] * g0[1] + p_diff[2] * g0[2])
        n2 = (p_diff[0] * g1[0] + p_diff[1] * g1[1] + p_diff[2] * g1[2])
        n1 = -n1
        n2 = -n2

        s1 = g1[0] * g0[0] + g1[1] * g0[1] + g1[2] * g0[2]
        r1 = -g0[0] * g0[0] - g0[1] * g0[1] - g0[2] * g0[2]
        s2 = g1[0] * g1[0] + g1[1] * g1[1] + g1[2] * g1[2]
        r2 = -g0[0] * g1[0] - g0[1] * g1[1] - g0[2] * g1[2]

        s_scale = s2 / s1
        n1_scaled = n1 * s_scale
        s1_scaled = s1 * s_scale
        r1_scaled = r1 * s_scale

        n_ = n1_scaled - n2
        r_ = r1_scaled - r2

        r = n_ / r_
        s = (n1 - (r1 * r)) / s1

        A = p0 + (g0 * r)
        B = p1 + (g1 * s)
        A = np.nan_to_num(A)
        B = np.nan_to_num(B)

        # check points are not behind start points
        t = A - p0
        u = B - p1
        if g0[0] != 0 and g0[1] != 0 and g0[2] != 0 and not is_equal(t[0] / g0[0], t[1] / g0[1], t[2] / g0[2]):
            A = p0
        if g1[0] != 0 and g1[1] != 0 and g1[2] != 0 and not is_equal(u[0] / g1[0], u[1] / g1[1], u[2] / g1[2]):
            B = p1

        return LineSegment(A, B)
