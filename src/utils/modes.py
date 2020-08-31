from enum import Enum


class NavMode(Enum):  # Motion-to-Goal or Boundary-Following
    MTG = 1
    BF = 2


class Dir(Enum):  # direction of object which is followed in BF mode
    LEFT = 1
    RIGHT = 2


class ExMode(Enum):  # execution mode of runtime module
    RUNNING = 0
    PAUSED = 1
    STEP = 2
    STOP = 3


class IntersectType(Enum):
    HARD = 0
    SOFT = 1
