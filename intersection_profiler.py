import numpy
import pyximport



pyximport.install(setup_args={"include_dirs": numpy.get_include()}, language_level=3)

from src.nav.ObstacleSegment import ObstacleSegment
from src.geom.Node import Node
from src.geom_old.Utils import Node as oNode

import pstats, cProfile

# create dummy objects

obst_boundary_0 = []
for i in range(400):
    obst_boundary_0.append(Node.from_list([2, i*0.01]))
obst_0 = ObstacleSegment(0, obst_boundary_0)

obst_boundary_1 = []
for i in range(400):
    obst_boundary_1.append(Node.from_list([6, i*0.01]))
obst_1 = ObstacleSegment(0, obst_boundary_1)

path = []
for i in range(400):
    path.append(Node.from_list([-1 + i*0.01,1]))
curPos, curH, goalPos = Node.from_list([-1,1,0]), 0, Node.from_list([5,0])

o_cur_pos = oNode(curPos.as_ndarray())
o_path = [oNode(p.as_ndarray()) for p in path]
o_boundary_0 = [oNode(p.as_ndarray()) for p in obst_boundary_0]
o_boundary_1 = [oNode(p.as_ndarray()) for p in obst_boundary_1]

def py_naive_intersecting():
    x = obst_0.giwp_3d_naive_py(o_boundary_0, o_cur_pos, o_path, 0.1, TANGENT_DIST_INNER)
    print(x)
def py_naive_not_intersecting():
    x = obst_1.giwp_3d_naive_py(o_boundary_1, o_cur_pos, o_path, 0.1, TANGENT_DIST_INNER)
    print(x)
def py_presorting_intersecting():
    x = obst_0.giwp_3d_presorting_py(o_boundary_0, o_cur_pos, o_path, 0.1, TANGENT_DIST_INNER)
    print(x)
def py_presorting_not_intersecting():
    x = obst_1.giwp_3d_presorting_py(o_boundary_1, o_cur_pos, o_path, 0.1, TANGENT_DIST_INNER)
    print(x)
def py_cacheing_intersecting():
    x = obst_0.giwp_3d_plane_cacheing_py(o_boundary_0, o_cur_pos, o_path, 0.1, TANGENT_DIST_INNER)
    print(x)
def py_cacheing_not_intersecting():
    x = obst_1.giwp_3d_plane_cacheing_py(o_boundary_1, o_cur_pos, o_path, 0.1, TANGENT_DIST_INNER)
    print(x)

def py_c_naive_intersecting():
    x = obst_0.giwp_3d_naive_py_c(curPos, path, 0.1, TANGENT_DIST_INNER)
    print(x)
def py_c_naive_not_intersecting():
    x = obst_1.giwp_3d_naive_py_c(curPos, path, 0.1, TANGENT_DIST_INNER)
    print(x)
def py_c_presorting_intersecting():
    x = obst_0.giwp_3d_presorting_py_c(curPos, path, 0.1, TANGENT_DIST_INNER)
    print(x)
def py_c_presorting_not_intersecting():
    x = obst_1.giwp_3d_presorting_py_c(curPos, path, 0.1, TANGENT_DIST_INNER)
    print(x)
def py_c_cacheing_intersecting():
    x = obst_0.giwp_3d_plane_cacheing_py_c(curPos, path, 0.1, TANGENT_DIST_INNER)
    print(x)
def py_c_cacheing_not_intersecting():
    x = obst_1.giwp_3d_plane_cacheing_py_c(curPos, path, 0.1, TANGENT_DIST_INNER)
    print(x)
def py_c_presorting_exit_intersecting():
    x = obst_0.giwp_3d_presorting_exit_py_c(curPos, path, 0.1, TANGENT_DIST_INNER)
    print(x)
def py_c_presorting_exit_not_intersecting():
    x = obst_1.giwp_3d_presorting_exit_py_c(curPos, path, 0.1, TANGENT_DIST_INNER)
    print(x)

def c_intersecting():
    x = obst_0.get_intersect_with_path_3d(curPos, path, 0.1, TANGENT_DIST_INNER)
    print(x)
def c_not_intersecting():
    x = obst_1.get_intersect_with_path_3d(curPos, path, 0.1, TANGENT_DIST_INNER)
    print(x)

"""to_run = [py_naive_intersecting, py_naive_not_intersecting, py_presorting_intersecting,
                py_presorting_not_intersecting,
          py_c_naive_intersecting, py_c_naive_not_intersecting, py_c_presorting_intersecting,
                py_c_presorting_not_intersecting, py_c_cacheing_intersecting, py_c_cacheing_not_intersecting]"""
to_run = [c_intersecting, c_not_intersecting]

for fun in to_run:
    print("PROFILING:", fun.__name__)

    cProfile.runctx(fun.__name__+"()", globals(), locals(), ".\\plts\\profiling\\Profile_1.prof")
    with open('.\\plts\\profiling\\prof_'+fun.__name__+".txt", 'w') as stream:
        stats = pstats.Stats(".\\plts\\profiling\\Profile_1.prof", stream=stream)
        stats.print_stats()

"""print(" +++ starting profiling 2 +++")
cProfile.runctx("to_profile_c()", globals(), locals(), ".\\plts\\profiling\\Profile_2.prof")

s = pstats.Stats("Profile_2.prof")
s.sort_stats("time").print_stats()"""
