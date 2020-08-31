import pyximport
import numpy


pyximport.install(setup_args={"include_dirs": numpy.get_include()}, language_level=3)

from src.geom.BoundedPlane import BoundedPlane
from src.geom.Node import Node
from src.geom.Lines import LineSegment

p = BoundedPlane(Node(1.99949078, 1.00289189, 0.), Node(1.99855487, 0.03332494, 0.), Node(-1.65114420e-04, 3.37338175e-03, 9.99994297e-01))
# Node(1.99949078, 1.00289189, 0.), Node(1.99855487, 0.03332494, 0.), Node(-1.65114420e-04, 3.37338175e-03, 9.99994297e-01)

r = LineSegment(Node(0.28779185, 0.0236598, 0.), Node(5.5, 1.5, 14.76340195))
# Node(0.28779185, 0.0236598, 0.), Node(-5.5, 1.5, 14.76340195)

intersect = p.get_intersect_with_lseg(r)

print(intersect)