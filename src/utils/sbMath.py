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


def normalize_angle(angle: float):
    angle = angle % (2 * math.pi)
    if angle < 0:
        angle += (2 * math.pi)
    return angle


def normalize_array(data: np.ndarray):
    """
    Normalizes a numpy array with min-max normalization
    :param data: input array
    :return:  normalized numpy array
    """
    return (data - np.min(data)) / (np.max(data) - np.min(data))


def linear_depth(depth_sample, z_near=0.1, z_far=500.0):
    """
    converts depth sample to linear depth
    :param depth_sample: e.g. depthTexture.r
    :param z_near: near plane
    :param z_far:  far plane
    :return: linear depth
    """
    depth_sample = 2.0 * depth_sample - 1.0
    z_linear = 2.0 * z_near * z_far / (z_far + z_near - depth_sample * (z_far - z_near))
    return z_linear


def filter_nan(arr: np.ndarray):
    """
    Removes NaN values from input array and empty sub-arrays
    :param arr: N-D numpy array
    :return: N-D numpy array without NaN or empty values
    """
    if arr.ndim > 1 or (arr.size > 0 and arr[0].ndim > 0):
        tmp = [filter_nan(value) for value in arr if value.size > 0]
        return [value for value in tmp if len(value) > 0]
    return [value for value in arr if not np.isnan(value)]


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


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


def is_point_btw(a, b, point, tolerance=0.1):
    d_apb = dist_2d_p(a, point) + dist_2d_p(b, point)
    d_ab = dist_2d_p(a, b)
    return abs(d_ab - d_apb) < tolerance


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


def is_equal(a: float, b: float, c: float, tolerance=0.001) -> bool:
    if math.isnan(a) or math.isnan(b) or math.isnan(c):
        return True
    return abs(a-b) < tolerance and abs(a-c) < tolerance


def circleRadiusFromPoints(A, B, C):
    if A == B or A == C or B == C:
        return math.inf
    yDelta_a = B[1] - A[1]
    xDelta_a = B[0] - A[0]
    yDelta_b = C[1] - B[1]
    xDelta_b = C[0] - B[0]
    center = [0, 0]

    aSlope = yDelta_a/xDelta_a
    bSlope = yDelta_b/xDelta_b

    AB_Mid = [(A[0]+B[0])/2, (A[1]+B[1])/2]
    BC_Mid = [(B[0]+C[0])/2, (B[1]+C[1])/2]

    if(yDelta_a == 0):
        center[0] = AB_Mid[0]
        if (xDelta_b == 0):
            center[1] = BC_Mid[1]
        else:
            center[1] = BC_Mid[1] + (BC_Mid[0] - center[0])/bSlope
    elif (yDelta_b == 0):
        center[0] = BC_Mid[0]
        if (xDelta_a == 0):
            center[1] = AB_Mid[1]
        else:
            center[1] = AB_Mid[1] + (AB_Mid[0]-center[0])/aSlope
    elif (xDelta_a == 0):
        center[1] = AB_Mid[1]
        center[0] = bSlope*(BC_Mid[1]-center[1]) + BC_Mid[0]
    elif (xDelta_b == 0):
        center[1] = BC_Mid[1]
        center[0] = aSlope*(AB_Mid[1]-center[1]) + AB_Mid[0]
    else:
        center[0] = (aSlope*bSlope*(AB_Mid[1]-BC_Mid[1]) - aSlope*BC_Mid[0] + bSlope*AB_Mid[0])/(bSlope-aSlope)
        center[1] = AB_Mid[1] - (center[0] - AB_Mid[0])/aSlope

    return math.sqrt((center[0] - A[0]) ** 2. + (center[1] - A[1]) ** 2.)
