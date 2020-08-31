import pyximport
import numpy
import numpy as np


pyximport.install(setup_args={"include_dirs": numpy.get_include()}, language_level=3)

from src.geom.Cone import Cone
from src.geom.Node import Node
from src.geom.Lines import Ray

def foo(r, p):
    # (x1, y1, z1) = ray start
    # (x2, y2, z2) = point
    # (gx1, gy1, gz1) = ray gradient
    # (gx2, gy2, gz2) = point to ray start gradient
    (x1, y1, z1) = r.start.x(), r.start.y(), r.start.z()
    (x2, y2, z2) = p.x(), p.y(), p.z()
    (gx1, gy1, gz1) = r.gradient.x(), r.gradient.y(), r.gradient.z()
    (gx2, gy2, gz2) = x2-x1, y2-y1, z2-z1

    return np.all(np.sign((gx1, gy1, gz1)) == np.sign((gx2, gy2, gz2))) and Node(gx1, gy1, gz1).as_unit_vector().dist_3d(Node(gx2, gy2, gz2).as_unit_vector()) < 0.01


c = Cone(Node(1, 0, 0), .1)

r = Ray(Node(1, 1, 0), Node(0.1, 0.1, .99)) # {2.453107504826903;2.4023578805030676;0.0}, {0.0005347658735306172;0.06832338199970331;0.9976630841609722}

#print(foo(r, Node(0.6, 0.6, -4)))

intersects = c.get_intersect_with_ray(r)

for intersect in intersects:
    print(intersect)