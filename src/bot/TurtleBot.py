import math

import pybullet as p
import numpy as np

from src.bot.Robot import Robot
from src.utils.DubinsPath import find_path


class TurtleBot(Robot):
    def __init__(self, pos, orn, config):
        super().__init__(pos, orn, config.TURTLEBOT_URDF, config)
        self.wheel_radius = 0.0352
        self.axle_length = 0.230

    def render(self):
        super().render()

    def update_cam(self):
        return super().update_cam(29)

    def drive_along(self, path_points):
        p0 = path_points[0]
        p1 = None if len(path_points) <= 1 else path_points[1]
        path_configurations, t = find_path(self.pos, self.orn,
                                           p0, p1,
                                           self._config.TURN_RADIUS)
        self.path_configurations_arr = np.array(path_configurations)

        t = np.array(t)
        x = self.path_configurations_arr[:, 0]  # desired x positions
        y = self.path_configurations_arr[:, 1]  # desired y positions

        left_vs, right_vs = self.differential_drive(x, y, t)
        self.set_wheel_speeds(left_vs[0], right_vs[0])

    def differential_drive(self, x, y, t):
        """Convert x-y path points to differential drive robot wheel speeds"""
        r = self.wheel_radius
        l = self.axle_length / 2.

        while t[1] <= 0:
            t = t[1:]
            x = x[1:]
            y = y[1:]

        # derivatives with duplicated first element
        doty = np.insert(np.diff(y) / np.diff(t), -1, (np.diff(y) / np.diff(t))[-1])
        dotx = np.insert(np.diff(x) / np.diff(t), -1, (np.diff(x) / np.diff(t))[-1])
        ddoty = np.insert(np.diff(doty) / np.diff(t), -1, (np.diff(doty) / np.diff(t))[-1])
        ddotx = np.insert(np.diff(dotx) / np.diff(t), -1, (np.diff(dotx) / np.diff(t))[-1])

        v = np.sqrt(dotx * dotx + doty * doty)  # velocity
        kappa = (dotx * ddoty - doty * ddotx) / (v * v * v)  # curvature I think
        dotphi1 = (v / r) * (kappa * l + 1)  # right wheel speed
        dotphi2 = (v / r) * (-kappa * l + 1)  # left wheel speed

        return dotphi2, dotphi1

    def get_rob_pos_from_speeds(self, t, dotphi2, dotphi1, dt=0.1):
        N = len(t)  # number of samples
        r = self.wheel_radius
        L = self.axle_length / 2.

        xp = np.zeros((N))  # robot x positions
        yp = np.zeros((N))  # robot y positions
        th = np.zeros((N))  # robot theta angles

        # calculate robot path
        rob_pos = self.get_pos_orn()
        xp[0] = rob_pos[0]
        yp[0] = rob_pos[1]
        th[0] = rob_pos[2]
        for i in range(N - 1):
            xp[i + 1] = xp[i] + (r * dt / 2.0) * (dotphi1[i] + dotphi2[i]) * math.cos(th[i])
            yp[i + 1] = yp[i] + (r * dt / 2.0) * (dotphi1[i] + dotphi2[i]) * math.sin(th[i])
            th[i + 1] = th[i] + (r * dt / (2.0 * L)) * (dotphi1[i] - dotphi2[i])

    def set_wheel_speeds(self, l_speed, r_speed):
        p.setJointMotorControl2(self.p_id, 0, p.VELOCITY_CONTROL,
                                targetVelocity=l_speed*self._config.ROB_SPEED,
                                force=1000)
        p.setJointMotorControl2(self.p_id, 1, p.VELOCITY_CONTROL,
                                targetVelocity=r_speed*self._config.ROB_SPEED,
                                force=1000)

    def drive_stop(self):
        p.setJointMotorControl2(self.p_id, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=1000)
        p.setJointMotorControl2(self.p_id, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=1000)
