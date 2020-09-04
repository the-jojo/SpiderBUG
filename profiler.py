import logging
import numpy
import pyximport

pyximport.install(setup_args={"include_dirs": numpy.get_include()}, language_level=3)


from pycallgraph import PyCallGraph
from pycallgraph.output import GraphvizOutput
from pycallgraph import Config
from pycallgraph import GlobbingFilter

from src import sbPlanner
from src.nav.ObstacleSegment import ObstacleSegment
from src.geom.Node import Node
from src.nav import BoundaryFollowing
from src.utils.modes import NavMode

config = Config()
config.trace_filter = GlobbingFilter(exclude=[
 """
 'dot',
    'nan_to_num',
    'resize',
    'cross',
    'norm',
    'mean',
    'concatenate',
    'ravel',
    'reshape',
    'moveaxis',
    'isposinf',
    'copyto',
    'isneginf',
    '_handle_fromlist',
    'all'"""
])
graphviz = GraphvizOutput(output_file='plts\\MTG.png')

logger = logging.getLogger("test")

obst_boundary_0 = []
for i in range(200):
    obst_boundary_0.append(Node.from_list([2, i*0.01]))
obst_0 = ObstacleSegment(0, obst_boundary_0)
obst_boundary_1 = []
for i in range(200):
    obst_boundary_1.append(Node.from_list([4, 1+i*0.01]))
obst_1 = ObstacleSegment(0, obst_boundary_1)
curPos, curH, goalPos = Node.from_list([0,0,0]), 0, Node.from_list([5,0])

sbPlanner.plan(curPos, curH, 0.1, goalPos, [obst_0, obst_1], [], logger)
sbPlanner.nav_mode = NavMode.MTG

with PyCallGraph(output=graphviz, config=config):
    sbPlanner.plan(curPos, curH, 0.1, goalPos, [obst_0, obst_1], [], logger)

graphviz = GraphvizOutput(output_file='plts\\BF.png')

with PyCallGraph(output=graphviz, config=config):
    sbPlanner.nav_mode = NavMode.BF
    BoundaryFollowing.v_followed = Node.from_list([1.8, 0])
    sbPlanner.plan(curPos, curH, 0.1, goalPos, [obst_0, obst_1], [], logger)