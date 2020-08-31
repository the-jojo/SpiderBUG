import numpy
import pyximport

pyximport.install(setup_args={"include_dirs": numpy.get_include()}, language_level=3)

from src.geom.Node import Node
from src.utils import sbPlotter

obst_boundary_0 = []
for i in range(400):
    obst_boundary_0.append(Node.from_list([2, i*0.01]))

obst_boundary_1 = []
for i in range(400):
    obst_boundary_1.append(Node.from_list([6, i*0.01]))

path = []
for i in range(400):
    path.append(Node.from_list([-1 + i*0.01,1]))
curPos, curH, goalPos = Node.from_list([-1,1,0]), 0, Node.from_list([5,0])

sbPlotter.plot(n_pos=curPos, n_path=path, n_obst_l=[obst_boundary_0, obst_boundary_1],
               n_obst_v=[Node.from_list([-0.05, -0.006, 1]), Node.from_list([0., 0.065, 1])],
               b_path_to_3d=1, h_view_a=70)