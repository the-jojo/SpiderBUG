import logging
import pyximport
import numpy
import sys

from src.utils.config import Config, default_config

pyximport.install(setup_args={"include_dirs": numpy.get_include()})

from copy import deepcopy
import zmq
import time
import numpy as np
import pybullet as p
import dill as pickle

from src.utils.DubinsPath import find_path
from src.utils.modes import ExMode
from src.bot.Environment import *
from src.utils.sbMath import is_path_point_sensible
from src.geom.Node import Node

if not sys.warnoptions:
    import warnings
    warnings.simplefilter("error")


loop_count = 0
config_ = Config()
scenes = [scen_0, scen_1, scen_2, scen_3, scen_4, scen_5, scen_6, scen_7]


def load_env() -> Scenario:
    env_id = config_.OBST_COURSE

    return scenes[env_id](config_)


def start_env(env_id) -> Scenario:
    env = load_env()
    env.instantiate(config_.HEADLESS)
    return env


def main(logger, ):
    global loop_count, config_

    zmq_context = zmq.Context.instance()

    # setup zmq control channel
    ctrl_socket = zmq_context.socket(zmq.SUB)
    ctrl_socket.connect("tcp://127.0.0.1:%s" % config_.PORT_GUI_2_ALL)
    ctrl_socket.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])

    # setup zmq status channels
    state_socket_p = zmq_context.socket(zmq.PUB)
    state_socket_p.bind("tcp://*:%s" % config_.PORT_ROB_2_GUI)

    # setup zmq path channel
    path_socket = zmq_context.socket(zmq.SUB)
    path_socket.setsockopt(zmq.CONFLATE, 1)
    path_socket.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])
    path_socket.connect("tcp://127.0.0.1:%s" % config_.PORT_PLAN_2_ROB)

    # setup zmq sensing channel
    sens_socket = zmq_context.socket(zmq.PUB)
    sens_socket.bind("tcp://*:%s" % config_.PORT_ROB_2_SENS)

    # setup zmq position channel
    posh_socket = zmq_context.socket(zmq.PUB)
    posh_socket.bind("tcp://*:%s" % config_.PORT_ROB_2_PLAN)

    logger.info("READY")

    ex_mode = ExMode.PAUSED
    # wait for start
    while ex_mode == ExMode.PAUSED:
        a_msg = ctrl_socket.recv()
        (m_msg, config_) = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
        if "START" in m_msg or "RESTART" in m_msg:
            ex_mode = ExMode.RUNNING
            logger.info("STARTED, config ID: %s" % config_.OBST_COURSE)
            break
        if "SHUTDOWN" in m_msg:
            ex_mode = ExMode.STOP
            logger.info("SHUTTING DOWN")
            exit(0)
        elif "STEP" in m_msg:
            ex_mode = ExMode.STEP
            logger.info("STARTED STEP, config ID: %s" % config_.OBST_COURSE)
            break

    # setup zmq poller
    poller = zmq.Poller()
    poller.register(ctrl_socket, zmq.POLLIN)
    poller.register(path_socket, zmq.POLLIN)

    # init and instantiate to pybullet simulation
    env = start_env(config_.OBST_COURSE)
    p.setRealTimeSimulation(1)  # let it settle
    time.sleep(2.5)

    if ex_mode == ExMode.RUNNING:
        p.setRealTimeSimulation(1)
    else:
        p.setRealTimeSimulation(0)

    path_points: [Node] or None = None
    # loop while ()
    while ex_mode != ExMode.STOP:

        socks = dict(poller.poll(0))

        # poll control
        if ctrl_socket in socks and socks[ctrl_socket] == zmq.POLLIN:
            a_msg = ctrl_socket.recv()
            if a_msg is None: continue
            (m_msg, config_) = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
            if "SHUTDOWN" in m_msg:
                ex_mode = ExMode.STOP
                p.setRealTimeSimulation(0)
                break
            elif "STEP" in m_msg:
                ex_mode = ExMode.STEP
                p.setRealTimeSimulation(0)
            elif "PAUSE" in m_msg:
                ex_mode = ExMode.PAUSED
                p.setRealTimeSimulation(0)
            elif "RESTART" in m_msg:
                logger.info("+++ RESTARTING +++")
                loop_count = 0
                p.setRealTimeSimulation(0)
                env.delete()
                path_points = None

                env = start_env(config_.OBST_COURSE)
                p.setRealTimeSimulation(1)  # let it settle
                time.sleep(2.5)

                # empty path socket
                try:
                    path_socket.recv(flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass

                ex_mode = ExMode.RUNNING
                p.setRealTimeSimulation(1)
                continue
            elif "START" in m_msg:
                ex_mode = ExMode.RUNNING
                p.setRealTimeSimulation(1)

            logger.info("received ctrl msg: %s" % m_msg)

        if ex_mode == ExMode.RUNNING or ex_mode == ExMode.STEP:
            time_0 = time.perf_counter()

            # update env pos and speed
            env.update()

            # poll new path
            if path_socket in socks and socks[path_socket] == zmq.POLLIN:
                # receive path points
                a_msg = path_socket.recv()
                n_path_points = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
                if path_points is None:
                    logger.info("Received first path")
                path_points = n_path_points

                path_points = [Node.from_list(point_) for point_ in path_points]

            # remove covered node from path points if exists
            if path_points is not None:
                p_p = deepcopy(path_points)
                for i in range(len(p_p)):
                    if not is_path_point_sensible(p_p[i], env.bot.past_path, config_.NODE_REMOVAL_DIST):
                        path_points.remove(p_p[i])

            env.set_path(path_points)

            # update robot's camera
            img_data = env.bot.update_cam()
            sens_socket.send(default_config['PUB_PREFIX'].encode() + pickle.dumps(img_data))

            # send stuff to GUI
            state_socket_p.send(default_config['PUB_PREFIX'].encode() + env.export_to_sbGUI())

            # send position, heading, speed, planned path to sbPlanner
            posh_socket.send(default_config['PUB_PREFIX'].encode() + env.export_to_sbPlanner())

            loop_count += 1
            time_1 = time.perf_counter()
            if time_1 - time_0 > 2:
                logger.info("++ took : %d sec", time_1 - time_0)

        if ex_mode == ExMode.STEP:
            # end of step
            p.stepSimulation()
            ex_mode = ExMode.PAUSED
            time.sleep(0.04)
        else:
            time.sleep(0.001)


if __name__ == "__main__":
    # setup logging
    format = "%(name)s [%(loop_count)-4s] - %(levelname)-8s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO, )  # , level=logging.INFO, datefmt="%H:%M:%S"
    old_factory = logging.getLogRecordFactory()


    def record_factory(*args, **kwargs):
        record = old_factory(*args, **kwargs)
        record.loop_count = str(loop_count)
        return record


    logging.setLogRecordFactory(record_factory)

    main(logging.getLogger("sbRobot"))