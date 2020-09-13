import math
import random

import pybullet as p
import numpy as np
from pyquaternion import Quaternion

from src.bot.Robot import Robot
from src.utils.sbMath import calc_angle, get_point_towards


class SphereBot(Robot):
    def __init__(self, pos, orn, config):
        super().__init__(pos, orn, config.SPHERO_URDF, config)

    def render(self):
        super().render()

    def drive_along(self, path_points):
        self.path_configurations_arr = np.array([[self.pos.x(), self.pos.y()]] + [[point.x(), point.y()] for point in path_points])
        #self.path_configurations_arr = np.insert(self.path_configurations_arr, 0, [self.pos.x(), self.pos.y()])
        self.holonomic_drive(path_points[0].x(), path_points[0].y())

    def holonomic_drive(self, target_x, target_y):
        # get orientation of first point and then drive towards orientation
        heading = calc_angle(self.pos.as_ndarray() + np.array([1, 0, 0]), self.pos.as_ndarray(),
                             np.array([target_x, target_y]))

        amount = .185  # magic number mimics turtlebot speed of 0.1

        if self._config.ROB_MODEL == 2:
            # random non-deterministic robot motion
            heading = heading + random.uniform(-1, 1)
            amount = random.uniform(0.16, 0.24)

        dir_v = get_point_towards(np.array([0, 0]), heading, amount)

        p.resetBaseVelocity(self.p_id, [dir_v[0], dir_v[1], -1], [0, 0, 0])

    def get_pos_orn(self):
        """
        Modified from standard implementation to retrieve heading from last - current position
        change, as the sphere rotates and the heading cannot be used from pybullet.
        :return: (pos_x, pos_y, heading)
        """
        if self.last_pos == self.pos:
            if type(self.orn) == tuple:
                heading = 0
            else:
                heading = self.orn
        else:
            heading = calc_angle(self.last_pos.as_ndarray() + np.array([1,0,0]), self.last_pos.as_ndarray(), self.pos.as_ndarray())

        (pos, orn) = p.getBasePositionAndOrientation(self.p_id)  # orn = [x,y,z,w]

        return (*pos[0:2], heading)

    def get_orn_vector(self):
        (pos, orn) = p.getBasePositionAndOrientation(self.p_id)  # orn = [x,y,z,w]
        orn_q = Quaternion(orn[3], orn[0], orn[1], orn[2])
        v = np.array([1.,0.,0.])
        v_ = orn_q.rotate(v) # use x,y part to determine orientation angle
        v_[2] = 0
        return v_

    def drive_stop(self):
        p.resetBaseVelocity(self.p_id, [[0, 0, -1]], [[0, 0, 0]])
