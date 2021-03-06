import logging
import sys
import os
from copy import deepcopy

import numpy
import pyximport
import zmq
import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)  # allow ctrl + c quitting

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

pyximport.install(language_level=3, setup_args={"include_dirs": numpy.get_include()})

from src.utils.config import Config, default_config
from src.utils.modes import ExMode
from src.bot.Environment import *
from src.utils.sbMath import is_path_point_sensible
from src.geom.Node import Node

if not sys.warnoptions:
    import warnings
    warnings.simplefilter("error")

numpy.warnings.filterwarnings('ignore', category=numpy.VisibleDeprecationWarning)

loop_count = 0
config_ = Config()
scenes = [scen_0, scen_1, scen_2, scen_3, scen_4, scen_5, scen_6, scen_7]
zmq_context, ctrl_socket, path_socket, sens_socket, posh_socket, state_socket_p = None, None, None, None, None, None

def close_and_quit():
    global zmq_context, ctrl_socket, path_socket, sens_socket, posh_socket, state_socket_p
    print("QUITTING...")
    try:
        ctrl_socket.close()
        path_socket.close()
        sens_socket.close()
        posh_socket.close()
        state_socket_p.close()
        zmq_context.term()
    except:
        pass


def load_env() -> Scenario:
    """
    Runs the scenario configuration function corresponding to the obstacle course set in the config
    :return: Scenario
    """
    env_id = config_.OBST_COURSE

    return scenes[env_id](config_)


def start_env() -> Scenario:
    """
    Loads the environment and then instantiates it
    :return: instantiated Scenario
    """
    env = load_env()
    env.instantiate()
    return env


def main(logger):
    global loop_count, config_, \
        zmq_context, ctrl_socket, path_socket, sens_socket, posh_socket, state_socket_p

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
            logger.info("STARTED with scenario: %s" % config_.OBST_COURSE)
            break
        if "SHUTDOWN" in m_msg:
            ex_mode = ExMode.STOP
            logger.info("SHUTTING DOWN")
            exit(0)
        elif "STEP" in m_msg:
            ex_mode = ExMode.STEP
            logger.info("STARTED STEP with scenario: %s" % config_.OBST_COURSE)
            break

    # setup zmq poller
    poller = zmq.Poller()
    poller.register(ctrl_socket, zmq.POLLIN)
    poller.register(path_socket, zmq.POLLIN)

    # init and instantiate to pybullet simulation
    env = start_env()
    p.setRealTimeSimulation(1)  # let it settle
    time.sleep(2.5)

    if ex_mode == ExMode.RUNNING:
        p.setRealTimeSimulation(1)
    else:
        p.setRealTimeSimulation(0)

    path_points: [Node] or None = None
    while ex_mode != ExMode.STOP:

        socks = dict(poller.poll(0))

        # poll control
        if ctrl_socket in socks and socks[ctrl_socket] == zmq.POLLIN:
            a_msg = ctrl_socket.recv()
            if a_msg is None:
                continue
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

                env = start_env()
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

            logger.info(" > received ctrl msg: %s" % m_msg)

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
                    logger.debug("Received first path")
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
            state_socket_p.send(default_config['PUB_PREFIX'].encode() + env.export_to_gui())

            # send position, heading, speed, planned path to sbPlanner
            posh_socket.send(default_config['PUB_PREFIX'].encode() + env.export_to_planner())

            loop_count += 1
            time_1 = time.perf_counter()
            if time_1 - time_0 > 2:
                logger.info("ROB: took %d sec", time_1 - time_0)

        if ex_mode == ExMode.STEP:
            # end of step
            p.stepSimulation()
            ex_mode = ExMode.PAUSED
            time.sleep(0.04)
        else:
            time.sleep(0.001)


if __name__ == "__main__":
    # setup logging
    format_ = "%(name)s [%(loop_count)-4s] - %(levelname)-8s: %(message)s"
    logging.basicConfig(format=format_, level=logging.INFO, )  # , level=logging.INFO, datefmt="%H:%M:%S"
    old_factory = logging.getLogRecordFactory()


    def record_factory(*args, **kwargs):
        record = old_factory(*args, **kwargs)
        record.loop_count = str(loop_count)
        return record


    logging.setLogRecordFactory(record_factory)

    try:
        main(logging.getLogger("sbRobot"))
    finally:
        # clean up
        close_and_quit()
