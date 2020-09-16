import logging
import sys
import os
import time

import dill as pickle
import numpy as np
import pyximport
import zmq
import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)  # allow ctrl + c quitting

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.utils.config import Config, default_config
from src.utils.modes import ExMode

if not sys.warnoptions:
    import warnings
    warnings.simplefilter("error")

np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)

pyximport.install(language_level=3, setup_args={"include_dirs": np.get_include()})

from src.geom.Node import Node
from src.nav.ObstacleSegment import ObstacleSegment


def normalize_data(data: np.ndarray):
    """min-max normalises an np array"""
    return (data - np.min(data)) / (np.max(data) - np.min(data))


def filter_nan(arr: np.ndarray):
    """
    Recursively removes all nan values from sub arrays and removes empty arrays which may
    result from removing nan values
    """
    if arr.ndim > 1 or (arr.size > 0 and arr[0].ndim > 0):
        tmp = [filter_nan(value) for value in arr if value.size > 0]
        return [value for value in tmp if len(value) > 0]
    return [value for value in arr if not np.isnan(value)]


config_ = Config()
obst_counter = 0
loop_count = 0
zmq_context, ctrl_socket, obst_socket, sens_socket, state_socket = None, None, None, None, None


def close_and_quit():
    global zmq_context, ctrl_socket, obst_socket, sens_socket, state_socket
    print("QUITTING...")
    try:
        ctrl_socket.close()
        sens_socket.close()
        obst_socket.close()
        state_socket.close()
        zmq_context.term()
    except:
        pass


def scan_to_obst_segments(logger, depth_data, proj_mat, view_mat, past_obst_segments: [ObstacleSegment], was_pasued) \
        -> [ObstacleSegment]:
    """
    Converts a depth-image to obstacle segments.
     - gets world coordinates (point-clound) from depth-image
     - uses horizontal slice of point cloud and depth image (pc_slice and di_slice)
     - min-max normalise di_slice
     - get first derivative of di_slice into di_slice_deriv
     - from derivatives and di_slice elements which are 1, identify edge indices
     - filter background frompc_slice
     - divide pc_slice by edge indices into split
     - remove NaN values and empty arrays from split
     - create list of list of nodes from split (boundary_points)
     - ask each past obstacle segment to update its boundary points with new ones and remove them from
       boundary_points if they matched
     - any boundary points left over are assigned to new obstacle segments
    :param logger: logger
    :param depth_data: depth image + image height, width, etc
    :param proj_mat: projection matrix
    :param view_mat: view matrix
    :param past_obst_segments: previous obstacle segments (will be updated with new boundary points)
    :param was_pasued: if the simulation was paused in the last turn
    :return: list of obstacle segments
    """
    global obst_counter

    img_height = depth_data[1]
    img_width = depth_data[0]

    # 2d depth image [[0, 0.1, ...],
    #                [0, 0,   ...],
    #                [...        ]]
    depth_img = np.reshape(np.array(depth_data[3]), (depth_data[1], depth_data[0]))

    """
    Solution adapted from stackoverflow post by benbo yang
    at https://stackoverflow.com/questions/59128880/getting-world-coordinates-from-opengl-depth-buffer
    """
    point_cloud = np.empty([np.int(img_height), np.int(img_width), 4])
    proj_mat_a = np.asarray(proj_mat).reshape([4, 4], order='F')
    view_mat_a = np.asarray(view_mat).reshape([4, 4], order='F')
    tran_pix_world = np.linalg.inv(np.matmul(proj_mat_a, view_mat_a))

    for h in range(int(img_height/2)-1, int(img_height/2)+2):
        for w in range(0, img_width):
            x = (2 * w - img_width) / img_width
            y = -(2 * h - img_height) / img_height
            z = 2 * depth_img[h, w] - 1
            pixPos = np.asarray([x, y, z, 1])
            position = np.matmul(tran_pix_world, pixPos)
            point_cloud[np.int(h), np.int(w), :] = position / position[3]

    # 2d array of x-y points [[9,  10, 10.5, ...],
    #                         [50, 30, 25.5, ...]]
    pc_slice = point_cloud[int(img_height/2)-1, :, [0, 1]]
    # 1d array of distance values [1, 0.9, 1, ...]
    di_slice = depth_img[int(img_height/2)-1, :]

    # Exit early if no object in view
    if di_slice.min() >= 1.0:
        return []

    # normalize depth image slice
    if np.max(di_slice) - np.min(di_slice) == 0:
        logger.critical("blank depth slice")
    else:
        di_slice = normalize_data(di_slice)

    # get first derivative
    di_slice_deriv = np.diff(di_slice) / np.diff(np.arange(0, len(di_slice)))
    di_slice_deriv = np.abs(di_slice_deriv)

    edge_idx = []
    on_obst = False
    on_back = True
    # identify edge points
    for i, (y_deriv, y) in enumerate(zip(di_slice_deriv, di_slice[1:])):
        if y_deriv > config_.SCAN_EDGE_THRESHOLD and on_obst:
            # pronounced edge after we saw obstacle
            if i+1 < len(di_slice_deriv) and di_slice_deriv[i+1] == 0:
                # background comes next
                edge_idx.append(i + 1)
                on_obst = False
                on_back = True
            elif i+1 < len(di_slice_deriv) and di_slice_deriv[i+1] > 0:
                # another obstacle comes next
                edge_idx.append(i + 1)
                on_obst = False
                on_back = False
            else:
                # edge of depth image. Nothing to do
                pass
        elif y_deriv > 0 and on_back:
            # start of obst from background
            edge_idx.append(i+1)
            on_obst = True
            on_back = False
        elif y == 1 and on_obst:
            # end of obstacle and back to background
            edge_idx.append(i+1)
            on_obst = False
            on_back = True

    # background to filter out
    background_idx = np.argwhere((di_slice >= 1.0) | (np.isnan(di_slice))).flatten()

    # 2d array of segment coordinates
    # [[[x1, x2],[x1, x2]], - x of segm 1, 2
    #  [[y1, y2],[y1, y2]], - y of segm 1, 2
    #  ...]
    pc_slice[:, background_idx] = np.NaN
    split_x = np.split(pc_slice[0], edge_idx)
    split_y = np.split(pc_slice[1], edge_idx)

    split = np.vstack((split_x, split_y))
    # filter nan and empty
    split = filter_nan(split)

    # nodes from point x, y
    new_obstacle_segments: [ObstacleSegment] = []
    boundary_points = []
    try:
        boundary_points = [[Node.from_tuple(t) for t in list(zip(split[0][i], split[1][i]))] for i, _ in enumerate(split[0])]
    except:
        boundary_points = [[Node.from_tuple((split[0][i], split[1][i])) for i, _ in enumerate(split[0])]]

    # loop all obstacleSegments and keep only updated ones
    for obstS in past_obst_segments:
        used_points = obstS.update(boundary_points, was_pasued)
        if used_points is not None:
            new_obstacle_segments.append(obstS)
            boundary_points.remove(used_points)

    # create new obstacle segments for unused boundary points
    for obst_points in boundary_points:
        obstS = ObstacleSegment(obst_counter, obst_points)
        logger.info("FOUND OBST: %d" % obstS.id)
        obst_counter += 1
        new_obstacle_segments.append(obstS)

    return new_obstacle_segments


def main(logger):
    global loop_count, config_, obst_counter, \
        zmq_context, ctrl_socket, obst_socket, sens_socket, state_socket

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
    img, projMat, viewMat = None, None, None

    def recv_images():
        nonlocal img, projMat, viewMat
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
            if a_msg is None:
                continue
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

        if ex_mode == ExMode.RUNNING or ex_mode == ExMode.STEP:
            time_0 = time.perf_counter()

            try:

                recv_images()

                # convert depth image to obstacle Segments
                if img is not None:
                    obstacle_segments = \
                        scan_to_obst_segments(logger, img, projMat, viewMat, obstacle_segments, was_paused)

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
    format_ = "%(name)s [%(loop_count)-4s] - %(levelname)-8s: %(message)s"
    logging.basicConfig(format=format_, level=logging.INFO, )
    old_factory = logging.getLogRecordFactory()

    def record_factory(*args, **kwargs):
        record = old_factory(*args, **kwargs)
        record.loop_count = str(loop_count)
        return record


    logging.setLogRecordFactory(record_factory)

    try:
        main(logging.getLogger("sbPerception"))
    finally:
        # clean up
        close_and_quit()
