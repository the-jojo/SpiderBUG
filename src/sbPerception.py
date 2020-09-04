import logging
import pyximport
import sys
import numpy as np

# cython: language_level=3
from src.utils.config import Config, default_config
from src.utils.modes import ExMode

pyximport.install(language_level=3, setup_args={"include_dirs": np.get_include()})

if not sys.warnoptions:
    import warnings
    warnings.simplefilter("error")

import time
import zmq
import dill as pickle
from itertools import groupby
from operator import itemgetter


"""from pycallgraph import PyCallGraph
from pycallgraph.output import GraphvizOutput
from pycallgraph import Config
from pycallgraph import GlobbingFilter
config = Config()
config.trace_filter = GlobbingFilter(exclude=[])"""

from src.geom.Node import Node
from src.nav.ObstacleSegment import ObstacleSegment
import matplotlib.pyplot as plt


def normalize_data(data: np.ndarray):
    return (data - np.min(data)) / (np.max(data) - np.min(data))


def filter_nan(arr: np.ndarray):
    if arr.ndim > 1 or (arr.size > 0 and arr[0].ndim > 0):
        tmp = [filter_nan(value) for value in arr if value.size > 0]
        return [value for value in tmp if len(value) > 0]
    return [value for value in arr if not np.isnan(value)]


config_ = Config()
obst_counter = 0
loop_count = 0
img, projMat, viewMat = None, None, None

def plt_depth_slice(db_depthSlice, x_p, y_p, edge_idx, edge_db_y):
    # plot slice of depthBuffer
    fig1, ax1 = plt.subplots()
    ax1.scatter(np.arange(0, len(db_depthSlice)), db_depthSlice, s=1)
    ax1.scatter(x_p, y_p, s=1)
    ax1.scatter(edge_idx, edge_db_y, s=35, marker="x", c="r")
    plt.show()


def scan_to_obst_segments(logger, img, projMat, viewMat, past_obst_segments: [ObstacleSegment], was_pasued) -> [ObstacleSegment]:
    global obst_counter

    logger.debug("converting img to obstacles")
    # TODO assert img, etc is not none

    img_height = img[1]
    img_width = img[0]

    # 2d depthimage [[0, 0.1, ...],
    #                [0, 0,   ...],
    #                [...        ]]
    depthBuffer = np.reshape(np.array(img[3]), (img[1], img[0]))

    # solution 1
    pointCloud = np.empty([np.int(img_height), np.int(img_width), 4])
    projectionMatrix = np.asarray(projMat).reshape([4, 4], order='F')
    viewMatrix = np.asarray(viewMat).reshape([4, 4], order='F')
    tran_pix_world = np.linalg.inv(np.matmul(projectionMatrix, viewMatrix))

    for h in range(int(img_height/2)-1, int(img_height/2)+2):
        for w in range(0, img_width):
            x = (2 * w - img_width) / img_width
            y = -(2 * h - img_height) / img_height
            z = 2 * depthBuffer[h, w] - 1
            pixPos = np.asarray([x, y, z, 1])
            position = np.matmul(tran_pix_world, pixPos)
            pointCloud[np.int(h), np.int(w), :] = position / position[3]

    # 2d array of x-y points [[9,  10, 10.5, ...],
    #                         [50, 30, 25.5, ...]]
    pc_depthSlice = pointCloud[int(img_height/2)-1, :, [0, 1]]  # math.floor(pointCloud.shape[0] / 2)
    # 1d array of distance values [1, 0.9, 1, ...]
    db_depthSlice = depthBuffer[int(img_height/2)-1, :]  # math.floor(depthBuffer.shape[0] / 2)

    # Exit early if no object in view
    if db_depthSlice.min() >= 1.0:
        return []

    # normalize depthbuffer slice
    if np.max(db_depthSlice) - np.min(db_depthSlice) == 0:
        logger.critical("blank depth slice")
    else:
        db_depthSlice = normalize_data(db_depthSlice)

    # get first derivative
    y_p = np.diff(db_depthSlice) / np.diff(np.arange(0, len(db_depthSlice)))
    x_p = (np.array(np.arange(0, len(db_depthSlice)))[:-1] + np.array(np.arange(0, len(db_depthSlice)))[1:]) / 2
    y_p = np.abs(y_p)

    new_edge = []
    on_obst = False
    on_back = True
    for i, (y_t, y_d) in enumerate(zip(y_p, db_depthSlice[1:])):
        if y_t > config_.SCAN_EDGE_THRESHOLD and on_obst:
            # pronounced edge after we saw obstacle
            if i+1 < len(y_p) and y_p[i+1] == 0:
                # background comes next
                new_edge.append(i + 1)
                on_obst = False
                on_back = True
            elif i+1 < len(y_p) and y_p[i+1] > 0:
                # another obstacle comes next
                new_edge.append(i + 1)
                on_obst = False
                on_back = False
            else:
                # edge of depth image. Nothing to do
                pass
        elif y_t > 0 and on_back:
            # start of obst from background
            new_edge.append(i)
            on_obst = True
            on_back = False
        elif y_d == 1 and on_obst:
            # end of obstacle and back to background
            new_edge.append(i)
            on_obst = False
            on_back = True

    # edges / points of discontinuity
    #edge_idx = np.argwhere((y_p >= config_.SCAN_EDGE_THRESHOLD)).flatten() # maybe get rid of +1
    #edge_idx = np.concatenate((edge_idx, np.array([0])))
    #edge_db_y = db_depthSlice[edge_idx]

    # background to filter out
    background_idx = np.argwhere((db_depthSlice >= 1.0) | (np.isnan(db_depthSlice))).flatten()
    background_db_y = db_depthSlice[background_idx]

    for k, g in groupby(enumerate(background_idx), lambda ix: ix[0] - ix[1]):
        b_e = list(map(itemgetter(1), g))
        #edge_idx = np.append(edge_idx, [b_e[int(len(b_e)/2)]])

    # 2d array of segment coordinates
    # [[[x1, x2],[x1, x2]], - x of segm 1, 2
    #  [[y1, y2],[y1, y2]], - y of segm 1, 2
    #  ...]
    pc_depthSlice[:, background_idx] = np.NaN
    split_x = np.split(pc_depthSlice[0], new_edge)
    #split_x = [t[0:] for t in split_x]
    split_y = np.split(pc_depthSlice[1], new_edge)
    #split_y = [t[0:] for t in split_y]
    split = np.vstack((split_x, split_y))
    split = filter_nan(split)
    # TODO after this all obstacle points are duplicated. no idea why
    if len(split[0]) > 20:
        logger.info("had to reencapsulate split NOT")
    #    split[0] = [split[0]]
     #   split[1] = [split[1]]

    #if len(split[1]) > 1 or len(split[1]) < 1:
    #    print("whaa")
    #    pass

    # loop all obstacleSegments and keep only updated ones
    new_obstacleSegments: [ObstacleSegment] = []
    res_boundaryPoints = []
    try:
        res_boundaryPoints  = [[Node.from_tuple(t) for t in list(zip(split[0][i], split[1][i]))] for i, _ in enumerate(split[0])]
    except:
        res_boundaryPoints  = [[Node.from_tuple((split[0][i], split[1][i])) for i, _ in enumerate(split[0])]]

    for obstS in past_obst_segments:
        used_points = obstS.update(res_boundaryPoints, was_pasued)
        if used_points is not None:
            new_obstacleSegments.append(obstS)
            res_boundaryPoints.remove(used_points)

    # create new obstacle segments for unused boundary points
    for obst_points in res_boundaryPoints:
        obstS = ObstacleSegment(obst_counter, obst_points)
        logger.info("FOUND OBST: %d" % obstS.id)
        obst_counter += 1
        new_obstacleSegments.append(obstS)

    if len(new_obstacleSegments) == 0:
        print("Lost obstacle")

    return new_obstacleSegments


def main(logger):
    """
    port_ctrl=PORT_GUI_2_ALL,
    port_obst=PORT_SENSE_2_PLAN,
    port_sens=PORT_ROB_2_SENS,
    port_state=PORT_SENS_2_GUI
    """
    global loop_count, img, projMat, viewMat, config_, obst_counter
    zmq_context = zmq.Context.instance()

    # setup zmq control channel
    ctrl_socket = zmq_context.socket(zmq.SUB)
    ctrl_socket.connect("tcp://127.0.0.1:%s" % config_.PORT_GUI_2_ALL)
    ctrl_socket.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])

    # setup zmq status channel
    state_socket = zmq_context.socket(zmq.PUB)
    state_socket.bind("tcp://*:%s" % config_.PORT_SENS_2_GUI)

    # setup zmq obst channel
    obst_socket = zmq_context.socket(zmq.PUB)
    obst_socket.bind("tcp://*:%s" % config_.PORT_SENSE_2_PLAN)

    # setup zmq sensing channel
    sens_socket = zmq_context.socket(zmq.SUB)
    sens_socket.setsockopt(zmq.CONFLATE, 1)
    sens_socket.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])
    sens_socket.connect("tcp://127.0.0.1:%s" % config_.PORT_ROB_2_SENS)

    logger.info("READY")

    ex_mode = ExMode.PAUSED
    # wait for start
    while ex_mode == ExMode.PAUSED:
        a_msg = ctrl_socket.recv()
        (m_msg, config_) = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
        if "START" in m_msg or "RESTART" in m_msg:
            ex_mode = ExMode.RUNNING
            logger.info("STARTED")
            break
        elif "SHUTDOWN" in m_msg:
            ex_mode = ExMode.STOP
            logger.info("SHUTTING DOWN")
            exit(0)
        elif "STEP" in m_msg:
            ex_mode = ExMode.STEP
            logger.info("STARTED STEP")
            break

    poller = zmq.Poller()
    poller.register(ctrl_socket, zmq.POLLIN)
    socks = dict()

    obstacle_segments: [ObstacleSegment] = np.array([])

    def recv_images():
        global img, projMat, viewMat
        # get depth image
        a_msg = sens_socket.recv()
        img_tmp = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
        if img_tmp is not None:
            (img, projMat, viewMat) = img_tmp
            logger.debug("Received images")

    was_paused = False

    # loop while ()
    while ex_mode != ExMode.STOP:
        was_paused = False

        socks = dict(poller.poll(0))
        # poll control
        if ctrl_socket in socks and socks[ctrl_socket] == zmq.POLLIN:
            a_msg = ctrl_socket.recv()
            if a_msg is None: continue
            (m_msg, config_) = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
            if m_msg == "SHUTDOWN":
                ex_mode = ExMode.STOP
                break
            elif m_msg == "STEP":
                ex_mode = ExMode.STEP
                was_paused = True
            elif m_msg == "PAUSE":
                ex_mode = ExMode.PAUSED
            elif "RESTART" in m_msg:
                logger.info("+++ RESTARTING +++")
                ex_mode = ExMode.RUNNING
                obst_counter = 0
                loop_count = 0
                img, projMat, viewMat = None, None, None
                obstacle_segments = []
                # empty sockets
                try:
                    sens_socket.recv(flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass
                obst_socket.send(default_config['PUB_PREFIX'].encode() + pickle.dumps(obstacle_segments))
                time.sleep(2)
                continue
            elif m_msg == "START":
                ex_mode = ExMode.RUNNING
                was_paused = True
            logger.info("received ctrl msg: %s" % m_msg)

        #graphviz = GraphvizOutput(
        #    output_file='.\\plts\\perf\\senser_' + str(loop_count) + '.png')
        #with PyCallGraph(output=graphviz, config=config):
        if ex_mode == ExMode.RUNNING or ex_mode == ExMode.STEP:
            time_0 = time.perf_counter()

            try:

                recv_images()

                # convert depth image to obstacle Segments
                if img is not None:
                    obstacle_segments = scan_to_obst_segments(logger, img, projMat, viewMat, obstacle_segments, was_paused)

                # send obstacle segments to GUI
                state_socket.send(default_config['PUB_PREFIX'].encode() + pickle.dumps(obstacle_segments))

                # send obstacle segments to Planner
                obst_socket.send(default_config['PUB_PREFIX'].encode() + pickle.dumps(obstacle_segments))
            except TypeError as e:
                logger.error("Ignored error:")
                logger.error(e)

            loop_count += 1
            time_1 = time.perf_counter()
            if time_1 - time_0 > 2:
                logger.info("++ took : %d sec", time_1 - time_0)

        if ex_mode == ExMode.STEP:
            # end step
            ex_mode = ExMode.PAUSED

        time.sleep(0.001)


if __name__ == "__main__":
    # setup logging
    format = "%(name)s [%(loop_count)-4s] - %(levelname)-8s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO,)
    old_factory = logging.getLogRecordFactory()

    def record_factory(*args, **kwargs):
        record = old_factory(*args, **kwargs)
        record.loop_count = str(loop_count)
        return record


    logging.setLogRecordFactory(record_factory)

    main(logging.getLogger("sbSenser"))
