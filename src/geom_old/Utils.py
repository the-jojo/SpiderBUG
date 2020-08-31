import math
import time
from enum import Enum

import numpy as np

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def is_point_btw(a, b, point, tolerance=0.1):
    d_apb = dist_2d_p(a, point) + dist_2d_p(b, point)
    d_ab = dist_2d_p(a, b)
    return abs(d_ab - d_apb) < tolerance

def dist_2d_p(a, b):
    return math.sqrt((b[0] - a[0]) ** 2. + (b[1] - a[1]) ** 2.)

class Node(np.ndarray):
    """
    Wrapper for numpy arrays. Forces float values of elements
    Allows hashing and equality.
    Allows conversion to 2d or 3d formats
    """
    def __new__(cls, a):
        obj = np.asarray(a).astype(float).view(cls).copy()
        obj.resize(3, refcheck=False)
        return obj

    def as2d(self):
        return Node(self[:2])

    def as3d(self):
        return Node(np.resize(self, (3,)))

    def dist_2d(self, other):
        if type(other) is Node:
            return dist_2d_p(self.as2d(), other.as2d())
        else:
            return dist_2d_p(self.as2d(), Node(other).as2d())

    def dist_3d(self, other):
        if type(other) is Node:
            return dist_2d_p(self.as3d(), other.as3d())
        else:
            return dist_2d_p(self.as3d(), Node(other).as3d())

    def __hash__(self):
        return hash(self.tostring())

    def __eq__(self, other):
        if type(other) != Node:
            other = Node(other)
        return self.__hash__() == other.__hash__()


def to_node(obj):
    if type(obj) == Node:
        return obj
    else:
        return Node(obj)

"""
class NavMode(Enum):  # Motion-to-Goal or Boundary-Following
    MTG = 1
    BF = 2


class Dir(Enum):  # direction of object which is followed in BF mode
    LEFT = 1
    RIGHT = 2


class ExMode(Enum):
    RUNNING = 0
    PAUSED = 1
    STEP = 2
    STOP = 3


class EnvObject:
    def __init__(self, start_pos: Node, start_orn: Node, file_name: str):
        self.start_pos = start_pos
        self.start_orn = start_orn
        self.file_name = file_name

    def instantiate(self, p):
        p.loadURDF(self.file_name, self.start_pos, p.getQuaternionFromEuler(self.start_orn))


class EnvState:
    def __init__(self, start_pos: Node, start_orn: Node, goal_pos: Node, env_objects: [EnvObject], speed: float):
        self.cur_pos = start_pos
        self.cur_orn = start_orn
        self.goal_pos = goal_pos
        self.bot = None
        self.env_objects = env_objects
        self.objects = []
        self.speed = speed
        self.past_path = []
        self.last_update_time = None
        self.real_speed = 0

    def instantiate(self, p):
        # init and connect to pybullet simulation
        p.connect(p.GUI)
        p.resetSimulation()
        p.setGravity(0, 0, -10.)

        plane = p.loadURDF("data\\plane.urdf")
        self.bot = SpiderRobot(self.cur_pos, p.getQuaternionFromEuler(self.cur_orn), self.speed)
        self.objects = [o.instantiate(p) for o in self.env_objects]

        self.past_path.append([*self.bot.get_pos_orn()[:2]])
        self.last_update_time = time.time()

        p.addUserDebugLine(self.goal_pos+Node([-0.2,-0.2,0.01]), self.goal_pos+Node([0.2,0.2,0.01]), [0,0,0], 4)
        p.addUserDebugLine(self.goal_pos+Node([0.2,-0.2,0.01]), self.goal_pos+Node([-0.2,0.2,0.01]), [0,0,0], 4)

        p.resetDebugVisualizerCamera(cameraDistance=8.5, cameraYaw=-180, cameraPitch=-120,
                                     cameraTargetPosition=self.goal_pos/2)

    def update(self):
        # get current config and time
        cur_config = self.bot.get_pos_orn()
        cur_time = time.time()
        cur_pos = Node([cur_config[0], cur_config[1]])
        cur_orn = cur_config[2]

        # add to past path if point not already in past path
        
        tmp_dist = np.array([dist_2d_p(x, cur_pos) for x in past_path])
        if np.min(tmp_dist) > 0.01:
            past_path.append(cur_pos[:2])
        
        self.past_path.append(cur_pos[:2])

        # update speed
        d_time = cur_time - self.last_update_time
        d_dist = dist_2d_p(self.past_path[-1], self.past_path[-2])
        if d_time <= 0:
            d_time = 0.001
        if len(self.past_path) > 1:
            self.real_speed = d_dist / d_time  # d/t

        self.last_update_time = cur_time
        self.cur_pos = cur_pos
        self.cur_orn = cur_orn

    def export_to_sbPlanner(self, future_path):
        posOrn = self.bot.get_pos_orn()
        return pickle.dumps((Node(posOrn[:2]), posOrn[2], self.speed, self.goal_pos, [Node(x) for x in future_path]))

"""

