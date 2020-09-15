TURTLEBOT_RADIUS = 0.176


# loaded config
default_config = {
    'PLANE_URDF':           ".\\data\\py_data\\plane.urdf",
    'BLOCK_URDF':           ".\\data\\py_data\\boston_box.urdf",
    'BLOCK_DYN_URDF':       ".\\data\\py_data\\boston_box_fl.urdf",
    'TURTLEBOT_URDF':       ".\\data\\py_data\\turtlebot.urdf",
    'SPHERO_URDF':          ".\\data\\py_data\\sphero.urdf",

    'PORT_GUI_2_ALL':       17555,
    'PORT_SENSE_2_PLAN':    17556,
    'PORT_SENS_2_GUI':      17559,
    'PORT_PLAN_2_ROB':      17557,
    'PORT_PLAN_2_GUI':      17562,
    'PORT_ROB_2_SENS':      17558,
    'PORT_ROB_2_GUI':       17560,
    'PORT_ROB_2_PLAN':      17561,
    'PUB_PREFIX':           "{UUID:0000-0000-0000-0000-1111}",

    'TURN_RADIUS':          0.3,
    'TURN_RADIUS_MIN':      0.1,
    'TURN_RADIUS_MAX':      1.,
    'TURN_RADIUS_STEP':     0.1,

    'ROB_SPEED':            0.1,
    'ROB_SPEED_MIN':        0.1,
    'ROB_SPEED_MAX':        0.6,
    'ROB_SPEED_STEP':       0.1,
    'ROB_RADIUS':           TURTLEBOT_RADIUS,
    'ROB_MODEL':            0,

    'TANGENT_DIST_INNER_F':     2.,
    'TANGENT_DIST_OUTER_F':     4.5,
    'TANGENT_DIST_INNER_F_MIN': 1.1,
    'TANGENT_DIST_INNER_F_MAX': 7.1,
    'TANGENT_DIST_INNER_F_STEP':1,
    'TANGENT_DIST_OUTER_F_MIN': 1.1,
    'TANGENT_DIST_OUTER_F_MAX': 7.1,
    'TANGENT_DIST_OUTER_F_STEP':1,

    'H_TOL':                0.2,
    'D_TOL':                0.0,
    'H_TOL_MIN':            0.,
    'H_TOL_MAX':            1.,
    'H_TOL_STEP':           0.1,
    'D_TOL_MIN':            0.,
    'D_TOL_MAX':            1.,
    'D_TOL_STEP':           0.1,

    'NODE_REMOVAL_DIST':    0.15,

    'SCAN_EDGE_THRESHOLD':  0.35,

    'RES_HORIZONTAL':       480,
    'RES_VERTICAL':         70,
    'RES_HORIZONTAL_MIN':   480,
    'RES_HORIZONTAL_MAX':   4800,
    'RES_HORIZONTAL_STEP':  432,

    'PATH_RES':             0.05,
    'PATH_RES_QUICK':       0.3,

    'OBST_COURSE':          1,
    'SCENARIOS':            {'Clear': 0, 'Static Simple': 1, 'Static Cluttered': 2,
                             'Static C-Shape': 3, 'Dynamic Dodge-the-Bullet': 4,
                             'Dynamic Traffic-Lights': 5, 'Dynamic At-the-Mall': 6,
                             'Obstacle Experiment': 7},
    'ROBOTS':               {'TurtleBot 2': 0, 'SphereBot Deterministic': 1, 'SphereBot Random': 2,},
    'OBSTACLE_MOVEMENT':    3,  # only used in scenario 7

    'P_START':              [0., 0., 0.],
    'O_START':              [1., 0., 0.],
    'P_GOAL':               [8., 0., 0.],

    # Experiment config
    'PATH_FNAME':           0,
    'CONF_VALUES':          [],
    'SET_HEADER':           0,
    'ITERATION':            0,
    'EXP_TIMEOUT':          440
}


class Config(object):
    def __init__(self, given_conf=None):
        if given_conf is None:
            self._config = default_config
        else:
            self._config = given_conf

    def get_property(self, property_name):
        if property_name not in self._config.keys(): # we don't want KeyError
            return None  # just return None if not found
        return self._config[property_name]

    def set_property(self, property_name, property_value):
        self._config[property_name] = property_value

    @property
    def PORT_GUI_2_ALL(self):
        return self.get_property('PORT_GUI_2_ALL')

    @property
    def PORT_SENSE_2_PLAN(self):
        return self.get_property('PORT_SENSE_2_PLAN')

    @property
    def PORT_SENS_2_GUI(self):
        return self.get_property('PORT_SENS_2_GUI')

    @property
    def PORT_PLAN_2_ROB(self):
        return self.get_property('PORT_PLAN_2_ROB')

    @property
    def PORT_PLAN_2_GUI(self):
        return self.get_property('PORT_PLAN_2_GUI')

    @property
    def PORT_ROB_2_SENS(self):
        return self.get_property('PORT_ROB_2_SENS')

    @property
    def PORT_ROB_2_GUI(self):
        return self.get_property('PORT_ROB_2_GUI')

    @property
    def PORT_ROB_2_PLAN(self):
        return self.get_property('PORT_ROB_2_PLAN')

    @property
    def PUB_PREFIX(self):
        return self.get_property('PUB_PREFIX')

    @property
    def TURN_RADIUS(self):
        return self.get_property('TURN_RADIUS')

    @property
    def ROB_RADIUS(self):
        return self.get_property('ROB_RADIUS')

    @property
    def ROB_SPEED(self):
        return self.get_property('ROB_SPEED')

    @property
    def TANGENT_DIST_INNER(self):
        return self.get_property('ROB_RADIUS') * self.get_property('TANGENT_DIST_INNER_F')

    @property
    def TANGENT_DIST_OUTER(self):
        return self.get_property('ROB_RADIUS') * self.get_property('TANGENT_DIST_OUTER_F')

    @property
    def H_TOL(self):
        return self.get_property('H_TOL')

    @property
    def D_TOL(self):
        return self.get_property('D_TOL')

    @property
    def NODE_REMOVAL_DIST(self):
        return self.get_property('NODE_REMOVAL_DIST')

    @property
    def RES_HORIZONTAL(self):
        return self.get_property('RES_HORIZONTAL')

    @property
    def RES_VERTICAL(self):
        return self.get_property('RES_VERTICAL')

    @property
    def PATH_RES(self):
        return self.get_property('PATH_RES')

    @property
    def PATH_RES_QUICK(self):
        return self.get_property('PATH_RES_QUICK')

    @property
    def OBST_COURSE(self):
        return self.get_property('OBST_COURSE')

    @property
    def SCEN_CHOICES(self):
        return self.get_property('SCENARIOS')

    @property
    def ROB_CHOICES(self):
        return self.get_property('ROBOTS')

    @property
    def HEADLESS(self):
        return self.get_property('HEADLESS')

    @property
    def SCAN_EDGE_THRESHOLD(self):
        return self.get_property('SCAN_EDGE_THRESHOLD')

    @property
    def PLANE_URDF(self):
        return self.get_property('PLANE_URDF')

    @property
    def BLOCK_URDF(self):
        return self.get_property('BLOCK_URDF')

    @property
    def BLOCK_DYN_URDF(self):
        return self.get_property('BLOCK_DYN_URDF')

    @property
    def TURTLEBOT_URDF(self):
        return self.get_property('TURTLEBOT_URDF')

    @property
    def SPHERO_URDF(self):
        return self.get_property('SPHERO_URDF')

    @property
    def P_START(self):
        return self.get_property('P_START')

    @property
    def P_GOAL(self):
        return self.get_property('P_GOAL')

    @property
    def O_START(self):
        return self.get_property('O_START')

    @property
    def OBSTACLE_MOVEMENT(self):
        return self.get_property('OBSTACLE_MOVEMENT')

    @property
    def ROB_MODEL(self):
        return self.get_property('ROB_MODEL')

    def __getstate__(self):
        return self._config

    def __setstate__(self, state):
        self._config = state


class ExpConfig(Config):
    """
    'PATH_FNAME':           0,
    'CSV_FNAME':            "",
    'CONF_VALUES':          [],
    'SET_HEADER':           0,
    """
    @property
    def PATH_FNAME(self):
        return self.get_property('PATH_FNAME')

    @property
    def CONF_VALUES(self):
        return self.get_property('CONF_VALUES')

    @property
    def SET_HEADER(self):
        return self.get_property('SET_HEADER')

    @property
    def ITERATION(self):
        return self.get_property('ITERATION')

    @property
    def EXP_TIMEOUT(self):
        return self.get_property('EXP_TIMEOUT')