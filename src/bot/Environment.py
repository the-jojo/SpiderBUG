import math
import random
import time
import dill as pickle
import pybullet as p
import numpy as np

from src.bot.SphereBot import SphereBot
from src.bot.TurtleBot import TurtleBot
from src.geom.Node import Node
from src.utils.DubinsPath import find_path
from src.utils.sbMath import dist_2d_p, circleRadiusFromPoints


class Obstacle:
    def __init__(self, start_pos: Node, start_orn: Node, velocity: Node, urdf_path: str, v_change=None, const_v=False, rand_acc=False):
        self.start_pos = start_pos
        self.start_orn = start_orn
        self.velocity = velocity
        self.v_change = v_change
        self.rand_acc = rand_acc
        self.const_velocity = const_v
        self.urdf_path = urdf_path
        self.p_id = -1
        self.last_update_time = None

    def instantiate(self):
        self.p_id = p.loadURDF(self.urdf_path, self.start_pos.as_list_3d(),
                               p.getQuaternionFromEuler(self.start_orn.as_list_3d()))

    def update(self):
        new_time = time.time()
        if self.last_update_time is None:
            self.last_update_time = new_time
            return
        if self.v_change is not None:
            mag = self.velocity.mag()
            last_v = self.velocity
            self.velocity = self.velocity + self.v_change * (new_time - self.last_update_time)
            if self.const_velocity:
                self.velocity = self.velocity.as_unit_vector() * mag
            if self.rand_acc:
                rand_v = Node.from_list([random.uniform(-1, 1), random.uniform(-1, 1)]).as_unit_vector() * 0.05
                self.v_change = (self.v_change + rand_v).as_unit_vector() * random.uniform(0, 0.1)
                if mag > 0.15:
                    self.velocity = self.velocity.as_unit_vector() * 0.12
                (pos, orn) = p.getBasePositionAndOrientation(self.p_id)
                if pos[0] > 8.5 or pos[0] < 3 or pos[1] > 1 or pos[1] < -1:
                    self.velocity = Node.from_list([6.5, 0]) - Node.from_tuple(pos).as_node_2d()
                    self.velocity = self.velocity.as_unit_vector() * 0.1
        p.resetBaseVelocity(self.p_id, self.velocity.as_list_3d(), [0, 0, 0])

        self.last_update_time = new_time


class Scenario:
    def __init__(self, start_pos: Node, start_orn: Node, goal_pos: Node, env_objects: [Obstacle], config):
        self.goal_pos = goal_pos
        self.config_ = config
        if config.ROB_MODEL == 0:
            # Turtlebot
            self.bot = TurtleBot(start_pos, p.getQuaternionFromEuler(start_orn.as_list_3d()), self.config_)
        elif config.ROB_MODEL >= 1:
            # Sphere
            self.bot = SphereBot(start_pos, p.getQuaternionFromEuler(start_orn.as_list_3d()), self.config_)
        self.env_objects = env_objects
        self.left_vs, self.right_vs = np.array([0.]), np.array([0.])
        self.path_configurations = []
        self.path_configurations_arr = np.array([[],[]])
        self.state = 0

    def instantiate(self, headless=False):
        # init and connect to pybullet simulation
        if headless or p.isConnected():
            p.connect(p.DIRECT)
        else:
            p.connect(p.GUI)
        p.resetSimulation()
        p.setGravity(0, 0, -10.)

        p.loadURDF(self.config_.PLANE_URDF)
        self.bot.render()
        for o in self.env_objects:
            o.instantiate()

        p.addUserDebugLine(self.goal_pos.as_ndarray()+[-0.2, -0.2, 0.01],
                           self.goal_pos.as_ndarray()+[0.2, 0.2, 0.01], [0, 0, 0], 4)
        p.addUserDebugLine(self.goal_pos.as_ndarray()+[0.2, -0.2, 0.01],
                           self.goal_pos.as_ndarray()+[-0.2, 0.2, 0.01], [0, 0, 0], 4)

        p.addUserDebugLine(self.bot.pos.as_ndarray() + [-0.2, 0.2, 0.01],
                           self.bot.pos.as_ndarray() + [0.2, 0.2, 0.01], [0, 0, 0], 4)
        p.addUserDebugLine(self.bot.pos.as_ndarray() + [0.2, 0.2, 0.01],
                           self.bot.pos.as_ndarray() + [0.2, -0.2, 0.01], [0, 0, 0], 4)
        p.addUserDebugLine(self.bot.pos.as_ndarray() + [0.2, -0.2, 0.01],
                           self.bot.pos.as_ndarray() + [-0.2, -0.2, 0.01], [0, 0, 0], 4)
        p.addUserDebugLine(self.bot.pos.as_ndarray() + [-0.2, -0.2, 0.01],
                           self.bot.pos.as_ndarray() + [-0.2, 0.2, 0.01], [0, 0, 0], 4)

        p.resetDebugVisualizerCamera(cameraDistance=8.5, cameraYaw=-180, cameraPitch=-120,
                                     cameraTargetPosition=self.goal_pos.as_ndarray() / 2)

    def update(self):
        p.setGravity(0, 0, -10.)
        if self.state == 0:
            for envO in self.env_objects:
                c_points = p.getClosestPoints(self.bot.p_id, envO.p_id, 0.001)
                if c_points is not None and len(c_points) > 0:
                    self.state = 2
                    print("CRASH")
            if self.goal_pos.dist_2d(self.bot.pos) < self.config_.ROB_RADIUS * 1.5:
                print("GOAL")
                self.state = 1
        self.bot.update()
        for o in self.env_objects:
            o.update()

    def set_path(self, path_points):
        if path_points is not None and len(path_points) > 0 and self.state == 0:
            # create dubins path that passes through first 2 path points
            p1 = None
            if len(path_points) > 1:
                p1 = path_points[1]
            self.path_configurations, t = find_path(self.bot.pos, self.bot.orn,
                                               path_points[0], p1,
                                               self.config_.TURN_RADIUS)
            self.path_configurations_arr = np.array(self.path_configurations)
            t = np.array(t)
            x = self.path_configurations_arr[:, 0]  # desired x positions
            y = self.path_configurations_arr[:, 1]

            if self.config_.ROB_MODEL >= 1:
                # sphero moves differently
                self.bot.holonomic_drive(path_points[0].x(), path_points[0].y())
                return
            # normal
            self.left_vs, self.right_vs = self.bot.differential_drive(x, y, t)
        else:
            # stop the robot since we are at the goal or crashed
            if self.config_.ROB_MODEL >= 1:
                # sphero moves differently
                self.bot.holonomic_drive(None, None)
                return
            self.left_vs = np.array([0.])
            self.right_vs = np.array([0.])
        self.bot.set_wheel_speeds(self.left_vs[0], self.right_vs[0])

    def export_to_sbPlanner(self):
        if self.state == 2:
            return pickle.dumps(
                (None, 0, self.goal_pos.as_list_2d(), self.path_configurations_arr[:, :2]))
        else:
            pos_orn = self.bot.get_pos_orn()
            return pickle.dumps((pos_orn[:2], pos_orn[2], self.goal_pos.as_list_2d(), self.path_configurations_arr[:,:2]))

    def export_to_sbGUI(self):
        #print(self.path_configurations_arr, len(self.path_configurations_arr))
        if len(self.path_configurations_arr[0]) > 0:
            to_send = (self.path_configurations_arr[:, 0], self.path_configurations_arr[:, 1],
                       [x[0] for x in self.bot.past_path], [x[1] for x in self.bot.past_path],
                       self.evaluate())
        else:
            to_send = ([], [],
                       [x[0] for x in self.bot.past_path], [x[1] for x in self.bot.past_path],
                       self.evaluate())
        return pickle.dumps(to_send)

    def evaluate(self):
        return self.state

    def delete(self):
        #p.resetSimulation()
        p.disconnect()

def scen_0(config_):
    # Clear
    start_pos = Node.from_list(config_.P_START)
    goal_pos = Node.from_list([5, 0])
    start_orn = Node.from_list(config_.O_START)

    return Scenario(start_pos, start_orn, goal_pos, [], config_)

def scen_1(config_):
    # Static Simple
    start_pos = Node.from_list(config_.P_START)
    goal_pos = Node.from_list(config_.P_GOAL)
    start_orn = Node.from_list(config_.O_START)

    obj1 = Obstacle(Node.from_list([3.5, 0.5, 0.5]),
                    Node.from_list([0, 0, 0.75]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    """obj1 = Obstacle(Node.from_list([1, 0., 0.5]),
                    Node.from_list([0, 0, 0]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj1a = Obstacle(Node.from_list([1, 1., 0.5]),
                    Node.from_list([0, 0, 0]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj1b = Obstacle(Node.from_list([1, -1., 0.5]),
                    Node.from_list([0, 0, 0]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj2 = Obstacle(Node.from_list([0.2, 2, 0.5]),
                    Node.from_list([0, 0, 0.]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj3 = Obstacle(Node.from_list([0.2, -2, 0.5]),
                    Node.from_list([0, 0, 0.]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj4 = Obstacle(Node.from_list([-0.8, 1, 0.5]),
                    Node.from_list([0, 0, 0.]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj5 = Obstacle(Node.from_list([-0.8, -1, 0.5]),
                    Node.from_list([0, 0, 0.]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)"""

    return Scenario(start_pos, start_orn, goal_pos, [obj1], config_) # obj1a, obj1b, obj2, obj3, obj4, obj5

def scen_2(config_):
    # Static Cluttered
    start_pos = Node.from_list(config_.P_START)
    goal_pos = Node.from_list(config_.P_GOAL)
    start_orn = Node.from_list(config_.O_START)

    obj1 = Obstacle(Node.from_list([2.2, 1.90, 0.5]),
                    Node.from_list([0, 0, 0.75]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj2 = Obstacle(Node.from_list([5.8, 0.9, 0.5]),
                    Node.from_list([0, 0, 0]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj3 = Obstacle(Node.from_list([2.3, -0.85, 0.5]),
                    Node.from_list([0, 0, 0.1]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj4 = Obstacle(Node.from_list([6.5, -1.90, 0.5]),
                    Node.from_list([0, 0, 0.75]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj5 = Obstacle(Node.from_list([10., 0.8, 0.5]),
                    Node.from_list([0, 0, 0.]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)

    return Scenario(start_pos, start_orn, goal_pos, [obj1, obj2, obj3, obj4], config_)

def scen_3(config_):
    # Static C Shape
    start_pos = Node.from_list(config_.P_START)
    goal_pos = Node.from_list(config_.P_GOAL)
    start_orn = Node.from_list(config_.O_START)

    obj0 = Obstacle(Node.from_list([2.3, 2.7, 0.5]),
                    Node.from_list([0, 0, 00]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj1 = Obstacle(Node.from_list([3.5, 2.20, 0.5]),
                    Node.from_list([0, 0, 0.75]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj2 = Obstacle(Node.from_list([4, 1, 0.5]),
                    Node.from_list([0, 0, 0]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj3 = Obstacle(Node.from_list([4.1, 0, 0.5]),
                    Node.from_list([0, 0, 0]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj4 = Obstacle(Node.from_list([4, -1, 0.5]),
                    Node.from_list([0, 0, 0]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj5 = Obstacle(Node.from_list([3.5, -2.20, 0.5]),
                    Node.from_list([0, 0, 0.75]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)
    obj6 = Obstacle(Node.from_list([2.3, -2.7, 0.5]),
                    Node.from_list([0, 0, 00]),
                    Node.from_list([0, 0, 0]), config_.BLOCK_URDF)

    return Scenario(start_pos, start_orn, goal_pos, [obj0, obj1, obj2, obj3, obj4, obj5, obj6], config_)

def scen_4(config_):
    # Dynamic Dodge-the-Bullet
    start_pos = Node.from_list(config_.P_START)
    goal_pos = Node.from_list(config_.P_GOAL)
    start_orn = Node.from_list(config_.O_START)

    obj1 = Obstacle(Node.from_list([7, 0.2, 0.5]),
                    Node.from_list([0, 0, 0.7]),
                    Node.from_list([-0.1, 0, 0.0]), config_.BLOCK_DYN_URDF)

    return Scenario(start_pos, start_orn, goal_pos, [obj1], config_)

def scen_5(config_):
    # Dynamic Traffic-Lights
    start_pos = Node.from_list(config_.P_START)
    goal_pos = Node.from_list(config_.P_GOAL)
    start_orn = Node.from_list(config_.O_START)

    obj1 = Obstacle(Node.from_list([3, -1.5, 0.5]),
                    Node.from_list([0, 0, 0.]),
                    Node.from_list([0, 0.09, 0.0]), config_.BLOCK_DYN_URDF)
    obj2 = Obstacle(Node.from_list([3, -4.5, 0.5]),
                    Node.from_list([0, 0, 0.]),
                    Node.from_list([0, 0.09, 0.0]), config_.BLOCK_DYN_URDF)
    obj3 = Obstacle(Node.from_list([6, 4, 0.5]),
                    Node.from_list([0, 0, 0.25]),
                    Node.from_list([0, -0.09, 0.0]), config_.BLOCK_DYN_URDF)

    return Scenario(start_pos, start_orn, goal_pos, [obj1, obj2, obj3], config_)

def scen_6(config_):
    # Dynamic At-the-Mall
    start_pos = Node.from_list(config_.P_START)
    goal_pos = Node.from_list(config_.P_GOAL)
    start_orn = Node.from_list(config_.O_START)

    obj1 = Obstacle(Node.from_list([6.5, 1, 0.5]),
                    Node.from_list([0, 0, 0.]),
                    Node.from_list([-0.1, -0.05, 0.0]).as_unit_vector()*0.1, config_.BLOCK_DYN_URDF)
    obj2 = Obstacle(Node.from_list([9.5, -0.5, 0.5]),
                    Node.from_list([0, 0, 0.]),
                    Node.from_list([-0.05, 0., 0.0]), config_.BLOCK_DYN_URDF)
    obj3 = Obstacle(Node.from_list([1.5, 1.5, 0.5]),
                    Node.from_list([0, 0, 0.]),
                    Node.from_list([0., 0.05, 0.0]), config_.BLOCK_DYN_URDF)
    obj4 = Obstacle(Node.from_list([1.5, -1.5, 0.5]),
                    Node.from_list([0, 0, 0.75]),
                    Node.from_list([0., 0.05, 0.0]), config_.BLOCK_DYN_URDF)


    return Scenario(start_pos, start_orn, goal_pos, [obj1, obj2, obj3, obj4], config_)

def scen_7(config_):
    # obstacle experiment
    start_pos = Node.from_list(config_.P_START)
    goal_pos = Node.from_list(config_.P_GOAL)
    start_orn = Node.from_list(config_.O_START)

    o_p_start = Node.from_list([6, 0.5, 0.5]) # start position
    o_o_start = Node.from_list([0, 0, 0.]) # start orientation
    o_v_start = Node.from_list([0., 0., 0.]) # start velocity
    o_v_change = None
    o_v_const = True
    o_v_rand = False
    if config_.O_M == 0:
        # static
        pass
    elif config_.O_M == 1:
        # fixed speed
        o_v_start = Node.from_list([-2.5, -0.5, 0.0]).as_unit_vector() * 0.1
    elif config_.O_M == 2:
        # fixed speed, changing velocity direction
        o_v_start = Node.from_list([.25, -.5, 0]).as_unit_vector() * 0.1
        o_v_change = Node.from_list([-0.25, 0.25, 0]).as_unit_vector() * (0.1/9.)
    elif config_.O_M == 3:
        # accelerating
        o_v_start = Node.from_list([-2.5, -0.5, 0]).as_unit_vector() * 0.1
        o_v_change = Node.from_list([-2.5, -0.5, 0]).as_unit_vector() * 0.008
        o_v_const = False
        pass
    elif config_.O_M == 4:
        # decelerating
        o_v_start = Node.from_list([-2.5, -0.5, 0]).as_unit_vector() * 0.15
        o_v_change = Node.from_list([-2.5, -0.5, 0]).as_unit_vector() * -0.005
        o_v_const = False
    else:
        # random motion
        o_v_start = Node.from_list([-2.5, -0.5, 0]).as_unit_vector() * 0.1
        o_v_change = Node.from_list([-0.25, 0.25, 0]).as_unit_vector() * (0.1/9.)
        o_v_const = False
        o_v_rand = True
    obj1 = Obstacle(o_p_start,
                    o_o_start,
                    o_v_start, config_.BLOCK_DYN_URDF,
                    o_v_change, o_v_const, o_v_rand)

    return Scenario(start_pos, start_orn, goal_pos, [obj1], config_)
