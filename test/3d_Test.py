import unittest
from copy import deepcopy

import numpy as np
import pyximport

pyximport.install(setup_args={"include_dirs": np.get_include()}, language_level=3)

from src.bot.ObstacleSegment import ObstacleSegment
from src.geom.Node import Node
from src.utils.config import Config
from src.geom.Cone import Cone
from src.geom.Lines import Ray, LineSegment
from src.geom.BoundedPlane import BoundedPlane


config_ = Config()


class MyTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        # obstacle
        obst_boundary_0 = []
        obst_boundary_0.append(Node.from_list([3.4769695018346574, -2.5027077437127208, 0.0]))
        obst_boundary_0.append(Node.from_list([2.844279788348339, -2.4782763690249774, 0.0]))
        obst_boundary_0.append(Node.from_list([2.4997601876766207, -2.556617206782348, 0.0]))
        obst_boundary_0.append(Node.from_list([2.4997656353107205, -3.4724425634213674, 0.0]))

        self.obst_0 = ObstacleSegment(0, obst_boundary_0)
        self.obst_0.velocity = [np.array([0, 0.1, 1])]
        # path
        self.path = []
        self.path.append(Node.from_list([2.0344043292883764, -1.477422457061568, 25.14365141169851]))
        self.path.append(Node.from_list([8., 0, 0]))
        self.path_3d = deepcopy(self.path)
        p0 = self.path_3d[0]
        for p in self.path_3d[1:]:
            p.z(p0.z() + 1. * p.dist_2d(p0))

    def setUp(self):
        pass

    def test_plane_ray_int(self):
        a = Node.from_list([3.4769695, -2.50270774, 0.])
        b = Node.from_list([2.84427979, -2.47827637, 0.])
        d = Node.from_list([0., 0.1, 1.])
        p = BoundedPlane(a, b, d)
        x = Node.from_list([2.03440433, -1.47742246, 25.14365141])
        y = Node.from_list([8., 0., 86.60185576])
        res = p.get_intersect_with_lseg(LineSegment(x, y))
        print(res)
        self.assertIsNone(res)

    def test_3d_intersect(self):
        #v1 = obst.get_intersect_with_path_3d(start_pos_3d, path_2d, rob_speed, tangent_dist_in, True)
        res = self.obst_0.get_intersect_with_path_3d(self.path[0], self.path[1:], 0.1, config_.TANGENT_DIST_INNER)
        print(res)
        self.assertIsNone(res)


    def test_3d_intersect_3(self):
        obst_boundary_0 = []
        obst_boundary_0.append(Node.from_list([2.5573048132394987, -0.26418480318485815, 0.0]))
        obst_boundary_0.append(Node.from_list([1.9356298958007823, -1.437379542305598, 0.0]))
        obst_0 = ObstacleSegment(0, obst_boundary_0)
        path = []
        path.append(Node.from_list([0, 0, 0]))
        path.append(Node.from_list([8., 0, 0]))

        #v1 = obst.get_intersect_with_path_3d(start_pos_3d, path_2d, rob_speed, tangent_dist_in, True)
        res = obst_0.get_intersect_with_path_3d(path[0], path[1:], 0.1, config_.TANGENT_DIST_OUTER)
        print(res)
        self.assertIsNotNone(res)

    def test_3d_intersect_2(self):
        rob_cone = Cone(Node(-0.00013341125959357908, -2.993642239849689e-05, 0.0), 0.1)
        p_ray = Ray(Node(6.992201033647909, 0.8580221268391768, 0.), Node(-0.1, 0.0, 1.0))
        res = rob_cone.get_intersect_with_ray(p_ray)
        for r in res:
            print(r)
        self.assertIsNotNone(res)
        self.assertGreater(len(res), 0)


if __name__ == '__main__':
    unittest.main()
