from copy import deepcopy

import matplotlib.pyplot as plt
import numpy as np
import math

from src.geom.Node import Node


def plot(n_pos: Node = None, n_path: [Node] = None, b_path_to_3d: int = 0, s_rob_speed: float = .1,
         n_obst_l: [[Node]] = None, n_obst_v: [Node] = None, h_view_a: float = 60., v_view_a: float = 30):
    f = plt.Figure(figsize=(10, 20))  # dpi=100
    a1 = f.add_subplot(211)
    a1.set_aspect('equal')

    a2 = f.add_subplot(212, projection='3d')

    if n_path is not None:
        if b_path_to_3d:
            n_path = deepcopy(n_path)
            p0 = n_path[0]
            for p in n_path[1:]:
                p.z(p0.z() + p0.dist_2d(p) / s_rob_speed)
                p0 = p

    if n_obst_l is not None:
        for i, n_obst in enumerate(n_obst_l):
            a1.scatter([p.x() for p in n_obst], [p.y() for p in n_obst], s=5, color='red')
            a2.scatter([p.x() for p in n_obst], [p.y() for p in n_obst], [p.z() for p in n_obst], s=5, color='red')
            if n_obst_v is not None:
                if n_path is not None:
                    z_scale = max([p.z() for p in n_path])
                else:
                    z_scale = 1
                a2.plot([n_obst[0].x(), n_obst[0].x() + z_scale * n_obst_v[i].x()],
                        [n_obst[0].y(), n_obst[0].y() + z_scale * n_obst_v[i].y()],
                        [0, z_scale * n_obst_v[i].z()], linewidth=1.5, color='red')
                a2.plot([n_obst[int(len(n_obst)/2)].x(), n_obst[int(len(n_obst)/2)].x() + z_scale * n_obst_v[i].x()],
                        [n_obst[int(len(n_obst)/2)].y(), n_obst[int(len(n_obst)/2)].y() + z_scale * n_obst_v[i].y()],
                        [0, z_scale * n_obst_v[i].z()], linewidth=1.5, color='red')
                a2.plot([n_obst[-1].x(), n_obst[-1].x() + z_scale * n_obst_v[i].x()],
                        [n_obst[-1].y(), n_obst[-1].y() + z_scale * n_obst_v[i].y()],
                        [0, z_scale * n_obst_v[i].z()], linewidth=1.5, color='red')

    if n_path is not None:
        a1.plot([p.x() for p in n_path], [p.y() for p in n_path], linewidth=2, color='grey')
        a2.plot([p.x() for p in n_path], [p.y() for p in n_path], [p.z() for p in n_path], linewidth=2, color='grey')

    if n_pos is not None:
        a1.scatter(n_pos.x(), n_pos.y(), s=22, marker='x', color='green')
        a2.scatter(n_pos.x(), n_pos.y(), n_pos.z(), s=22, marker='x', color='green')

    a2.view_init(v_view_a, h_view_a)

    f.savefig(fname="tmp_plot.png")
    plt.close(f)

def to_gg(node_l: [Node]):
    res = "Execute[{"
    for i, n in enumerate(node_l):
        n_str = "\"(" + str(n.x()) + "," + str(n.y()) + "," + str(n.z()) + ")\""
        if i > 0:
            res = res + ","
        res = res + n_str
    return res + "}]"