import pybullet as p
import pyximport
import numpy

pyximport.install(language_level=3, setup_args={"include_dirs": numpy.get_include()})

from src.bot.Environment import Scenario, Obstacle
from src.geom.Node import Node
from src.utils.config import Config

config = Config()
config.set_property('PLANE_URDF', "..\\data\\plane.urdf")
config.set_property('BLOCK_URDF', "..\\data\\boston_box.urdf")
config.set_property('TURTLEBOT_URDF', "..\\data\\turtlebot.urdf")


def scen_1(config_):
    # static easy
    start_pos = Node.from_list([0, 0, 0])
    goal_pos = Node.from_list([5.5, 1.5, 0])
    rob_start_orn = Node.from_list([1, 0, 0])

    obj1 = Obstacle(Node.from_list([3.5, 0.5, 0.5]), Node.from_list([0, 0, 0]), Node.from_list([0, 0, 0]), config_.BLOCK_URDF)

    return Scenario(start_pos, rob_start_orn, goal_pos, [obj1], config_)

def scen_2(config_):
    # static cluttered
    start_pos = Node.from_list([0, 0, 0])
    goal_pos = Node.from_list([10, 0, 0])
    rob_start_orn = Node.from_list([1, 0, 0])

    obj0 = Obstacle(Node.from_list([4, -1.2, 0.5]), Node.from_list([0, 0, 0.75]), Node.from_list([0, 0, 0]),
                    config_.BLOCK_URDF)
    obj1 = Obstacle(Node.from_list([4.75, -0.5, 0.5]), Node.from_list([0, 0, 0.75]), Node.from_list([0, 0, 0]),
                    config_.BLOCK_URDF)
    obj2 = Obstacle(Node.from_list([1.5, 1.5, 0.5]), Node.from_list([0, 0, 0]), Node.from_list([0, 0, 0]),
                    config_.BLOCK_URDF)
    obj3 = Obstacle(Node.from_list([6.5, 2, 0.5]), Node.from_list([0, 0, 0.2]), Node.from_list([0, 0, 0]),
                    config_.BLOCK_URDF)

    return Scenario(start_pos, rob_start_orn, goal_pos, [obj0, obj1, obj2, obj3], config_)

env = scen_2(config)
env.instantiate(False)
p.setRealTimeSimulation(1)

while 1:
    pass