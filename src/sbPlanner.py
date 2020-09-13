import logging
import sys
import time
import os

import dill as pickle
import numpy as np
import pyximport
import zmq

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.utils.config import Config, default_config

pyximport.install(language_level=3, setup_args={"include_dirs": np.get_include()})

from src.nav.ObstacleSegment import ObstacleSegment
from src.geom.Node import Node
import src.nav.BoundaryFollowing as bf
import src.nav.MotionToGoal as mtg
from src.utils.modes import NavMode, ExMode
from src.utils.sbMath import calc_angle, get_point_towards

if not sys.warnoptions:
    import warnings
    warnings.simplefilter("error")

nav_mode = NavMode.MTG
loop_count = 0
config_ = Config()
up_path = []


def plan(cur_pos_: Node, cur_heading_: float, goal_pos_: Node, new_obstacles_segments_: [ObstacleSegment],
         fut_path_: [Node], logger_) -> [Node]:
    """
    Plans a path from the current position and current heading to the goal position accounting for
    sensed obstacles. None is returned if the behaviour is switched between mtg and bf or back
    :param cur_pos_: current position
    :param cur_heading_: current heading
    :param goal_pos_: goal position
    :param new_obstacles_segments_: new sensed obstacle segments
    :param fut_path_: current future path
    :param logger_: logger
    :return: path or None
    """
    global nav_mode, config_, loop_count

    if nav_mode == NavMode.MTG:
        # +++ MOTION TO GOAL +++

        mtg.init(cur_pos_, cur_heading_, goal_pos_, logger_, config_)

        next_mode_, path_ = mtg.plan(cur_pos_, cur_heading_, config_.ROB_SPEED, goal_pos_, new_obstacles_segments_,
                                     fut_path_)

        if next_mode_ == NavMode.BF:
            # encountered C-shaped obstacle and switching next turn
            v_int = None
            v_obst = None
            for obst in new_obstacles_segments_:
                path_to_check = [cur_pos_, goal_pos_]
                v_int_new = obst.get_intersect_with_path_3d(cur_pos_, path_to_check, config_.ROB_SPEED,
                                                            config_.TANGENT_DIST_INNER, False)
                if v_int_new is not None and (v_int is None or v_int_new.dist_2d(cur_pos_) <= v_int.dist_2d(cur_pos_)):
                    v_int = v_int_new
                    v_obst = obst
            if v_int is not None:
                tmp = cur_pos_ + Node.from_list([1, 0])
                h = calc_angle(tmp.as_ndarray_2d(), cur_pos_.as_ndarray_2d(), v_int.as_ndarray_2d())
                d = cur_pos_.dist_2d(v_int) - config_.TANGENT_DIST_OUTER
                bf.v_followed = Node.from_array(get_point_towards(cur_pos_.as_ndarray_2d(), h, d))
                bf.obst_id_to_follow = v_obst.id
            if bf.v_followed is None:
                logger_.info("PLAN: No direct intersect with an obstacle was found. Continue in MTG")
                # there was no direct intersect btw path_to_goal and any obst, so we just continue
                next_mode_ = NavMode.MTG
            else:
                logger_.info("PLAN: Switching to BF")
                # reset current path and best paths in web
                mtg.reset()
    else:
        # +++ BOUNDARY FOLLOWING +++

        res = bf.init(new_obstacles_segments_, cur_pos_, cur_heading_, goal_pos_, logger_, config_)
        if res:
            next_mode_, path_ = bf.plan(cur_pos_, goal_pos_, cur_heading_, new_obstacles_segments_)
        else:
            # lost or escaped the obstacle; no need to do the planning
            next_mode_, path_ = NavMode.MTG, None

        if next_mode_ == NavMode.MTG:
            logger_.info("PLAN: Switching to MTG")

            # escaped obstacle switching next turn
            bf.reset()

    nav_mode = next_mode_
    return path_


def main(logger):
    """
    port_ctrl=PORT_GUI_2_ALL,
    port_path=PORT_PLAN_2_ROB,
    port_obst=PORT_SENSE_2_PLAN,
    port_posh=PORT_ROB_2_PLAN,
    port_state=PORT_PLAN_2_GUI
    """
    global config_, nav_mode, loop_count, up_path

    zmq_context = zmq.Context.instance()

    # setup zmq control channel as receiver
    ctrl_socket = zmq_context.socket(zmq.SUB)
    ctrl_socket.connect("tcp://127.0.0.1:%s" % config_.PORT_GUI_2_ALL)
    ctrl_socket.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])

    # setup zmq path channel as sender
    path_socket = zmq_context.socket(zmq.PUB)
    path_socket.bind("tcp://*:%s" % config_.PORT_PLAN_2_ROB)

    # setup zmq obstacle channel as receiver
    obst_socket = zmq_context.socket(zmq.SUB)
    obst_socket.setsockopt(zmq.CONFLATE, 1)
    obst_socket.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])
    obst_socket.connect("tcp://127.0.0.1:%s" % config_.PORT_SENSE_2_PLAN)

    # setup zmq pos channel as receiver
    posh_socket = zmq_context.socket(zmq.SUB)
    posh_socket.setsockopt(zmq.CONFLATE, 1)
    posh_socket.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])
    posh_socket.connect("tcp://127.0.0.1:%s" % config_.PORT_ROB_2_PLAN)

    # setup zmq status to gui channel
    state_socket = zmq_context.socket(zmq.PUB)
    state_socket.bind("tcp://*:%s" % config_.PORT_PLAN_2_GUI)

    cur_pos, goal_pos, cur_heading, cur_speed, fut_path = None, None, 0, 0, []
    sensed_obstacle_segments = []
    path = None
    nav_mode = NavMode.MTG
    ex_mode = ExMode.PAUSED

    logger.info("READY")

    # wait for start
    while ex_mode == ExMode.PAUSED:
        a_msg = ctrl_socket.recv()
        (m_msg, config_) = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
        if "START" in m_msg or "RESTART" in m_msg:
            ex_mode = ExMode.RUNNING
            logger.info(" > STARTED")
        elif "SHUTDOWN" in m_msg:
            ex_mode = ExMode.STOP
            logger.info(" > SHUTTING DOWN")
        elif "STEP" in m_msg:
            ex_mode = ExMode.STEP
            logger.info(" > STARTED STEP")

    poller = zmq.Poller()
    poller.register(ctrl_socket, zmq.POLLIN)
    poller.register(obst_socket, zmq.POLLIN)
    poller.register(posh_socket, zmq.POLLIN)
    socks = dict()

    logger.info("PLAN: Starting with MTG")
    logger.tangent_warning_outer = 0
    logger.tangent_warning_inner = 0

    time.sleep(1)

    def poll_all():
        """
        Polls sbPerception and sbRobot for new obstacles and new position
        :return:
        """
        nonlocal poller, socks, sensed_obstacle_segments, cur_pos, cur_heading, cur_speed, goal_pos, fut_path, ex_mode
        socks = dict(poller.poll(0))
        # receive obstacles
        if obst_socket in socks and socks[obst_socket] == zmq.POLLIN:
            _msg = obst_socket.recv()
            sensed_obstacle_segments = pickle.loads(_msg[len(default_config['PUB_PREFIX']):])

        # receive position, heading, future path
        if posh_socket in socks and socks[posh_socket] == zmq.POLLIN:
            _msg = posh_socket.recv()
            t_cur_pos, cur_heading, goal_pos, fut_path = pickle.loads(_msg[len(default_config['PUB_PREFIX']):])
            goal_pos = Node.from_list(goal_pos)
            fut_path = [Node.from_array(fp) for fp in fut_path]
            if t_cur_pos is None:
                return -1
            cur_pos = Node.from_tuple(t_cur_pos)
        return 0

    while ex_mode != ExMode.STOP:
        # poll control
        socks = dict(poller.poll(0))

        if ctrl_socket in socks and socks[ctrl_socket] == zmq.POLLIN:
            a_msg = ctrl_socket.recv()
            if a_msg is None:
                continue
            (m_msg, config_) = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
            if m_msg == "SHUTDOWN":
                ex_mode = ExMode.STOP
                break
            elif m_msg == "STEP":
                ex_mode = ExMode.STEP
            elif m_msg == "PAUSE":
                ex_mode = ExMode.PAUSED
            elif m_msg == "START":
                ex_mode = ExMode.RUNNING
            elif m_msg == "RESTART":
                logger.info(" > Restarting Planner")
                ex_mode = ExMode.RUNNING
                cur_pos, goal_pos, cur_heading, cur_speed, fut_path, up_path = None, None, 0, 0, [], []
                sensed_obstacle_segments = []
                path = None
                nav_mode = NavMode.MTG
                loop_count = 0
                mtg.reset()
                bf.reset()

                # empty sockets
                try:
                    obst_socket.recv(flags=zmq.NOBLOCK)
                    posh_socket.recv(flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass

                while poll_all() == -1:
                    logger.info("Waiting for sbRobot reinitialisation...")
                    time.sleep(0.5)
                continue
            logger.info(" > Received ctrl msg: %s" % m_msg)

        if ex_mode == ExMode.RUNNING or ex_mode == ExMode.STEP:
            logger.tangent_warning_outer = logger.tangent_warning_outer - 1
            logger.tangent_warning_inner = logger.tangent_warning_inner - 1
            time_0 = time.perf_counter()
            tmp = poll_all()
            if tmp == -1:
                logger.error(" > sbRobot indicated crash. Pausing planner")
                ex_mode = ExMode.PAUSED
                continue

            if cur_pos is not None and goal_pos is not None and cur_pos.dist_2d(goal_pos) > 0.1:
                # plan path
                try:
                    path = plan(cur_pos, cur_heading, goal_pos, sensed_obstacle_segments, fut_path, logger)

                    pos = cur_pos.as_list_3d()
                    if nav_mode == NavMode.MTG:
                        pos.append("mtg")
                    else:
                        pos.append("bf")
                    if path is None:
                        pos.append(0)
                    else:
                        pos.append(1)
                    up_path.append(pos)
                except RuntimeError as e:
                    logger.error(e)
                    logger.warning(" > Pausing Planner to avoid more errors")
                    ex_mode = ExMode.PAUSED

            if path is not None:
                # send path to sbRobot
                path_socket.send(default_config['PUB_PREFIX'].encode() + pickle.dumps([p.as_list_2d() for p in path]))

                # send nodes and planning state to GUI
                if nav_mode == NavMode.MTG:
                    state_socket.send(default_config['PUB_PREFIX'].encode()
                                      + pickle.dumps((mtg.web.DG.nodes, mtg.web.DG.edges, cur_pos, up_path)))
                elif bf.bf_waypoint is not None:
                    state_socket.send(default_config['PUB_PREFIX'].encode()
                                      + pickle.dumps(([bf.bf_waypoint.cur_pos], [], cur_pos, up_path)))

            loop_count += 1
            time_1 = time.perf_counter()
            if time_1 - time_0 > 2:
                logger.warning("PLAN: took : %d sec", time_1 - time_0)

        if ex_mode == ExMode.STEP:
            # end step
            ex_mode = ExMode.PAUSED

        # sleep and loop
        time.sleep(0.01)


if __name__ == "__main__":
    # setup logging
    format_ = "%(name)s [%(loop_count)-4s] - %(levelname)-8s: %(message)s"
    logging.basicConfig(format=format_, level=logging.INFO)
    old_factory = logging.getLogRecordFactory()

    def record_factory(*args, **kwargs):
        record = old_factory(*args, **kwargs)
        record.loop_count = str(loop_count)
        return record
    logging.setLogRecordFactory(record_factory)

    main(logging.getLogger("sbPlanner"))
