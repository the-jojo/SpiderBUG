import errno
import math
import os
from typing import Tuple, List

import dill as pickle
import numpy as np


class COLOR:
    RED = (255, 0, 0),
    GREEN = (0, 149, 0),
    BLACK = (0, 0, 0),
    WHITE = (255,255,255),
    LIGHT_GREEN = (204,255,229)
    GREY = (70,70,70)

class MATH:
    @staticmethod
    def sub(a, b):
        return (np.array(a) - np.array(b)).tolist()

    @staticmethod
    def add(a, b):
        return (np.array(a) + np.array(b)).tolist()

    @staticmethod
    def mult(a, B):
        return (np.array(a) * B).tolist()

    @staticmethod
    def to_unit_vector(v: Tuple[float, float]):
        if v == (0,0):
            return [0.,0.]
        return (np.array(v) / np.linalg.norm(np.array(v))).tolist()

    @staticmethod
    def dist_2d(p1, p2):
        """
        calculates euclidian distance between two points
        :param x1: point 1 x
        :param y1: point 1 y
        :param x2: point 2 x
        :param y2: point 2 y
        :return: scalar distance
        """
        x1, y1 = p1
        x2, y2 = p2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    @staticmethod
    def linesAreParallel(point_1, point_2, point_3, point_4):
        """ Return True if the given lines point_1-point_2 and
            point_3-point_4 are parallel """
        x1,y1 = point_1
        x2,y2 = point_2
        x3,y3 = point_3
        x4,y4 = point_4
        return ((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)) == 0

    @staticmethod
    def is_point_btw(a, b, p):
        return (MATH.dist_2d(a, p) + MATH.dist_2d(p , b)) - MATH.dist_2d(a , b) <= 0

class IO:
    @staticmethod
    def csv_path(base_dir, rob_name, n_iteration):
        return str(base_dir) + "\\" + rob_name + "_" + str(n_iteration) + ".csv"

    @staticmethod
    def po_path(base_dir, rob_name, n_iteration, n_path):
        return str(base_dir) + "\\po_files\\" + rob_name + "_" + str(n_iteration) + "_" + str(n_path) + ".po"

    @staticmethod
    def exists_csv(base_dir, rob_name, n_iteration):
        """Check if the csv corresponding to the experiment, scenario and iteration exists"""
        return os.path.isfile(IO.csv_path(base_dir, rob_name, n_iteration))

    @staticmethod
    def open_csv(base_dir, rob_name, n_iteration, mode="a+"):
        """
        Opens the csv corresponding to the experiment, scenario and iteration.
        Creates the csv and any folders leading to the csv if necessary
        """
        f_name = IO.csv_path(base_dir, rob_name, n_iteration)
        if not os.path.exists(os.path.dirname(f_name)):
            try:
                os.makedirs(os.path.dirname(f_name))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        return open(f_name, mode, newline=''), f_name

    @staticmethod
    def open_path(base_dir, rob_name, n_iteration, n_path):
        """
        Opens the ..._path.po file corresponding to the experiment, scenario, iteration and path number.
        Creates any folders leading to the file if necessary and opens in overwrite mode.
        """
        f_name = IO.po_path(base_dir, rob_name, n_iteration, n_path)
        if not os.path.exists(os.path.dirname(f_name)):
            try:
                os.makedirs(os.path.dirname(f_name))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        return open(f_name, 'wb'), f_name

    @staticmethod
    def write_headers_to_csv_if_needed(base_dir: str, rob_name: str, n_iteration: int, headers: List[str]):
        if not IO.exists_csv(base_dir, rob_name, n_iteration):
            # the csv for this scenario and iteration does not exist yet
            # open (create) the csv
            f, f_name = IO.open_csv(base_dir, rob_name, n_iteration, mode="w+")

            # print header of csv to console and to the csv
            for h in headers[:-1]:
                f.write(h + ",")
            f.write(headers[-1] + "\n")
            f.close()

    @staticmethod
    def write_path(base_dir: str, rob_name: str, n_iteration: int, n_path: int, path: List):
        f_path, f_name = IO.open_path(base_dir, rob_name, n_iteration, n_path)
        pickle.dump(path, f_path)
        f_path.close()
        print("Saved path to [" + f_name + "]")

    @staticmethod
    def write_info(base_dir: str, rob_name: str, n_iteration: int, run: int, time, final_state):
        f_csv, f_name = IO.open_csv(base_dir, rob_name, n_iteration)
        f_csv.write(str(run) + "," + str(time) + "," + str(final_state) + "\n")
        f_csv.close()
        print("Saved info to [" + f_name + "]")

