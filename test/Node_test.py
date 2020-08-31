import pyximport
import numpy
pyximport.install(setup_args={"include_dirs": numpy.get_include()})

from src.geom.Node import Node

n0 = Node.from_array(numpy.array([0.,0.,0.]))
n1 = Node.from_array(numpy.array([1, 2, 2]))
n2 = Node.from_array(numpy.array([2,3]))

print(n0)
print(n1.as_node_2d())
print(n2)
