import math
import random

import pybullet as p
import numpy as np

from src.bot.Robot import Robot
from src.utils.sbMath import calc_angle, get_point_towards


class SphereBot(Robot):
    def __init__(self, pos, orn, config):
        super().__init__(pos, orn, config.SPHERO_URDF, config)

    def render(self):
        super().render()

    def update_cam(self):
        return super().update_cam()

    def holonomic_drive(self, target_x, target_y):
        if target_x is None:
            p.resetBaseVelocity(self.p_id, [[0, 0, -1]], [[0, 0, 0]])  # [-0.4, 0, -0.001]
            return

        # get orientation of first point and then drive towards orientation
        heading = calc_angle(self.pos.as_ndarray() + np.array([1, 0, 0]), self.pos.as_ndarray(), np.array([target_x, target_y])) #
        cur_p = self.get_pos_orn()
        cur_heading = cur_p[2]
        dif_heading = heading - cur_heading
        if dif_heading > 0:
            new_heading = cur_heading + min(0.05, dif_heading)
        elif dif_heading < 0:
            new_heading = cur_heading + max(-0.05, dif_heading)
        else:
            new_heading = cur_heading

        amount = .185
        if self._config.ROB_MODEL == 3:
            # non-deterministic
            heading = heading + random.uniform(-1, 1)
            amount = random.uniform(0.16, 0.24)
        dir_v = get_point_towards(np.array([0,0]), heading, amount)

        """poorn = self.get_pos_orn()
        dir_v = np.array([x[0] - poorn[0], y[0] - poorn[1], 0])
        if np.all(dir_v == 0):
            print("0 vector")
        else:
            dir_v = (dir_v / np.linalg.norm(dir_v)) * 1"""

        # dir_v[0] = dir_v[0]

        p.resetBaseVelocity(self.p_id, [dir_v[0], dir_v[1], -1], [0, 0, 0])  # [-0.4, 0, -0.001]

    def get_pos_orn(self):
        """
        Returns the robot's current position and orientation in radians
        Orientation is the angle between the 2d direction vector and [1,0]
        :return:
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

    def stop(self):
        p.setJointMotorControl2(self.p_id, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=1000)
        p.setJointMotorControl2(self.p_id, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=1000)

    def pause(self):
        p.setJointMotorControl2(self.p_id, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=1000)
        p.setJointMotorControl2(self.p_id, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=1000)
