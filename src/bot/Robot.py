import time

import numpy as np
import pybullet as p
from pyquaternion import Quaternion

from src.geom.Node import Node
from src.utils.sbMath import angle_between


class Robot:
    """Robot superclass"""
    def __init__(self, position, orientation, urdf_path, config):
        self._config = config
        self.urdf_path = urdf_path
        self.p_id = -1

        self.pos = position
        self.last_pos = Node.from_list([0, 0])
        self.orn = 0
        self.real_speed = 0
        self.past_path = []
        self.past_path.append([*position.as_list_2d()])
        self.path_configurations_arr = np.array([[], []])

        self.cam_img = None

        self.last_update_time = time.time()

    def update(self):
        new_time = time.time()

        new_pos_orn = self.get_pos_orn()
        new_pos = Node.from_list([new_pos_orn[0], new_pos_orn[1]])
        new_orn = new_pos_orn[2]

        if not self.pos == new_pos:  # check that robot actually moved
            self.last_pos = self.pos
            self.pos = new_pos
            self.orn = new_orn

            # update past path
            self.past_path.append(self.pos.as_list_2d())

            # update speed
            d_time = new_time - self.last_update_time
            d_dist = new_pos.dist_2d(self.pos)
            if d_time <= 0:
                d_time = 0.001
            if len(self.past_path) > 1:
                self.real_speed = d_dist / d_time  # d/t

        self.last_update_time = new_time

    def render(self):
        # disable rendering to make adding the robot faster
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

        pos = self.pos.as_list_3d()
        pos[2] = 0.5

        self.p_id = p.loadURDF(self.urdf_path, pos)

        # reenable rendering
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    def update_cam(self, link_id=None):
        """Update depth camera; returns image, projection-matrix and view-matrix tuple"""
        if link_id == None:
            camPos = np.array((self.pos.x(), self.pos.y(), 0.2802))
            if type(self.orn) is tuple:
                camOrn = self.orn
            else:
                camOrn = Quaternion(axis=[0, 0, 1], angle=self.orn)
                camOrn = [camOrn.x, camOrn.y, camOrn.z, camOrn.w]
        else:
            ls = p.getLinkState(self.p_id, link_id, computeForwardKinematics=True)
            camPos = np.array(ls[0])
            camOrn = Quaternion(axis=[0, 0, 1], angle=self.orn)
            camOrn = [camOrn.x, camOrn.y, camOrn.z, camOrn.w]

        camMat = p.getMatrixFromQuaternion(camOrn)
        upVector = [0, 0, 1]
        forwardVec = np.array([camMat[0], camMat[3], camMat[6]])
        sideVec =  [camMat[1],camMat[4],camMat[7]]
        camUpVec = [camMat[2], camMat[5], camMat[8]]
        camTarget = [camPos[0] + forwardVec[0] * 20, camPos[1] + forwardVec[1] * 20, camPos[2] + forwardVec[2] * 20]
        camUpTarget = [camPos[0] + camUpVec[0], camPos[1] + camUpVec[1], camPos[2] + camUpVec[2]]
        camPos = camPos + forwardVec * 0.15
        self.viewMat = p.computeViewMatrix(camPos, camTarget, camUpVec)
        t = self._config.RES_VERTICAL / self._config.RES_HORIZONTAL
        self.projMat = p.computeProjectionMatrix(-1, 1, -t/2,
                                                 t/2, 0.12, 99)  # left, right, bottom, top
        # these below aren't quite right
        """self.projMat = p.computeProjectionMatrix(-1, 1, - self._config.RES_VERTICAL / self._config.RES_HORIZONTAL,
                                                 self._config.RES_VERTICAL / self._config.RES_HORIZONTAL, 0.12, 99)  
        # left, right, bottom, top"""
        """self.projMat = p.computeProjectionMatrixFOV(
                fov=150.0,
                aspect=2.5,
                nearVal=0.05,
                farVal=99)"""
        self.cam_img = p.getCameraImage(self._config.RES_HORIZONTAL, self._config.RES_VERTICAL,
                                        viewMatrix=self.viewMat,
                                        projectionMatrix=self.projMat,
                                        renderer=p.ER_BULLET_HARDWARE_OPENGL)
        return self.cam_img, self.projMat, self.viewMat

    def drive_along(self, path_points):
        pass

    def drive_stop(self):
        pass

    def get_orn_vector(self):
        (pos, orn) = p.getBasePositionAndOrientation(self.p_id)  # orn = [x,y,z,w]
        orn_q = Quaternion(orn[3], orn[0], orn[1], orn[2])
        v = np.array([1., 0., 0.])
        v_ = orn_q.rotate(v)  # use x,y part to determine orientation angle
        v_[2] = 0
        return v_

    def get_pos_orn(self):
        """
        Returns the robot's current position and heading in radians.
        Heading is the angle between the 2d direction vector and the vector [1,0]
        :return: (pos_x, pos_y, heading)
        """
        (pos, orn) = p.getBasePositionAndOrientation(self.p_id)  # orn = [x,y,z,w]
        # use pyquaternion
        orn_q = Quaternion(orn[3], orn[0], orn[1], orn[2])
        # get orientation vector
        unit_v = np.array([1., 0., 0.])
        orn_v = orn_q.rotate(unit_v)
        # discard z component
        orn_v[2] = 0.
        heading = angle_between(unit_v, orn_v)
        return (*pos[0:2], heading)
