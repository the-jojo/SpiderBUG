import csv
import logging
import time
from copy import deepcopy

import pyximport
import dill as pickle
import zmq
import numpy as np
import os
import errno

pyximport.install(setup_args={"include_dirs": np.get_include()}, language_level=3)

from src.utils.config import Config, default_config, ExpConfig

base_dir = "..\\Jupyter\\exp_data\\"
s_experiment = "tolerances"
s_csv = "TOL"


def exists_csv(n_scenario, n_iteration):
    f_name = str(base_dir) + str(s_experiment) + "\\S_" + str(n_scenario) + "\\i_" \
        + str(n_iteration) + "\\" + str(s_csv) + "_" + str(n_scenario) + ".csv"
    return os.path.isfile(f_name)


def open_csv(n_scenario, n_iteration, mode="a+"):
    f_name = str(base_dir) + str(s_experiment) + "\\S_" + str(n_scenario) + "\\i_" \
        + str(n_iteration) + "\\" + str(s_csv) + "_" + str(n_scenario) + ".csv"
    if not os.path.exists(os.path.dirname(f_name)):
        try:
            os.makedirs(os.path.dirname(f_name))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    return open(f_name, mode, newline=''), f_name


def open_path(n_path, n_scenario, n_iteration):
    f_name = str(base_dir) + str(s_experiment) + "\\S_" + str(n_scenario) + "\\i_" \
        + str(n_iteration) + "\\py_objs\\" + str(n_path) + "_path.po"
    if not os.path.exists(os.path.dirname(f_name)):
        try:
            os.makedirs(os.path.dirname(f_name))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    return open(f_name, 'wb'), f_name


def send_ctr_cmd(socket, command, logger, arg):
    socket.send(default_config['PUB_PREFIX'].encode() + pickle.dumps((command, arg)))
    logger.info("Sent ctrl command: %s" % command)


def poll(poller, rob_socket, pla_socket, up_path=None):
    socks = dict(poller.poll(0))
    r_state, r_path = 0, None
    if rob_socket in socks and socks[rob_socket] == zmq.POLLIN:
        a_msg = rob_socket.recv()
        _, _, plt_x_past, plt_y_past, r_state = pickle.loads(
            a_msg[len(default_config['PUB_PREFIX']):])
        r_path = [plt_x_past, plt_y_past]
    if pla_socket in socks and socks[pla_socket] == zmq.POLLIN:
        a_msg = pla_socket.recv()
        _, _, _, up_path = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])

    return r_state, r_path, up_path


path_f_c = 0


def run_exp(conf_vals: [], conf_vals_set: [],
            ctrl_socket: zmq.Socket, rob_socket: zmq.Socket, pla_socket: zmq.Socket, poller: zmq.Poller,
            logger: logging.Logger, config_: Config):
    global path_f_c
    # base case
    if len(conf_vals) == 0:
        logger.info("Running Exp with: " + str(conf_vals_set))

        # write config to file
        f_csv, f_name = open_csv(config_.OBST_COURSE, config_.ITERATION)
        for sv in conf_vals_set:
            f_csv.write(str(sv) + ",")
        f_csv.close()
        #logger.info("Saving csv to    " + f_name)

        # send start
        send_ctr_cmd(ctrl_socket, "RESTART", logger, config_)
        logger.info("...")
        time.sleep(10)
        time_0 = time.perf_counter()
        # wait until done
        r_state, r_path, up_path = poll(poller, rob_socket, pla_socket)
        while r_state < 1:
            time.sleep(0.5)
            r_state, r_path, up_path = poll(poller, rob_socket, pla_socket, up_path)
            time_1 = time.perf_counter()
            if time_1 - time_0 > config_.EXP_TIMEOUT:
                logger.warning("timeout")
                r_state = 2

        send_ctr_cmd(ctrl_socket, "PAUSE", logger, config_)
        time.sleep(10)

        # write path to file
        f_path, f_name = open_path(path_f_c, config_.OBST_COURSE, config_.ITERATION)
        pickle.dump((r_path, up_path), f_path)
        f_path.close()
        logger.info("Saved path to    " + f_name)


        # write csv to file
        f_csv, f_name = open_csv(config_.OBST_COURSE, config_.ITERATION)
        f_csv.write(str(r_state) + "," + str(path_f_c) + "\n")
        f_csv.close()

        # increment path identifier
        path_f_c += 1
        return

    conf = conf_vals[0]
    # iterate config values
    for conf['cur'] in np.arange(conf['cur'], conf['max'], conf['step']):
        # set config value
        if '.' in str(conf['step']):
            conf['cur'] = round(conf['cur'], len(str(conf['step']).split('.')[1]))
        config_.set_property(conf['key'], conf['cur'])
        # recurse down
        run_exp(conf_vals[1:], conf_vals_set + [conf['cur']], ctrl_socket, rob_socket, pla_socket,
                poller, logger, config_)
        if len(conf_vals[1:]) > 0:
            for c in conf_vals[1:]:
                c['cur'] = c['min']


def start_exp(exp_config_: ExpConfig, logger: logging.Logger,
              ctrl_socket: zmq.Socket, rob_socket: zmq.Socket,
              pla_socket: zmq.Socket, poller: zmq.Poller):
    global path_f_c

    if not exists_csv(exp_config_.OBST_COURSE, exp_config_.ITERATION):
        f, f_name = open_csv(exp_config_.OBST_COURSE, exp_config_.ITERATION, mode="w+")
        for vtr in exp_config_.CONF_VALUES:
            logger.info("key: " + vtr['key'])
            f.write(vtr['key'] + ",")
        f.write("final_state,path\n")
        f.close()
        logger.info("Wrote headers to " + f_name)

        path_f_c = exp_config_.PATH_FNAME

        run_exp(deepcopy(exp_config_.CONF_VALUES), [],
                ctrl_socket, rob_socket, pla_socket, poller,
                logger, exp_config_)
    else:
        last_keys = []
        f, f_name = open_csv(exp_config_.OBST_COURSE, exp_config_.ITERATION, mode="r")
        csvReader = csv.reader(f, delimiter=',')
        for j, row in enumerate(csvReader):
            if j != 0:
                last_keys = row[:len(exp_config_.CONF_VALUES)]
                path_f_c = int(row[-1]) + 1

        new_conf_vals = []
        for lk, cv in zip(last_keys, exp_config_.CONF_VALUES):
            cv = deepcopy(cv)
            cv['cur'] = float(lk)
            new_conf_vals.append(cv)

        new_conf_vals[-1]['cur'] = new_conf_vals[-1]['cur'] + new_conf_vals[-1]['step']

        if new_conf_vals[-1]['cur'] > new_conf_vals[-1]['max']:
            logger.info("skipping..")
            return

        run_exp(new_conf_vals, [],
                ctrl_socket, rob_socket, pla_socket, poller,
                logger, exp_config_)


def main(exp_config_: ExpConfig, logger):
    global path_f_c, s_csv, s_experiment
    # setup zmq control channel
    zmq_context = zmq.Context()
    ctrl_socket = zmq_context.socket(zmq.PUB)
    # ctrl_socket.connect("tcp://127.0.0.1:%s" % config_.PORT_GUI_2_ALL)
    ctrl_socket.bind("tcp://*:%s" % exp_config_.PORT_GUI_2_ALL)


    # setup zmq state channel
    obs_socket = zmq_context.socket(zmq.SUB)
    obs_socket.setsockopt(zmq.CONFLATE, 1)
    obs_socket.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])
    obs_socket.connect("tcp://127.0.0.1:%s" % exp_config_.PORT_SENS_2_GUI)

    rob_socket = zmq_context.socket(zmq.SUB)
    rob_socket.setsockopt(zmq.CONFLATE, 1)
    rob_socket.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])
    rob_socket.connect("tcp://127.0.0.1:%s" % exp_config_.PORT_ROB_2_GUI)

    pla_socket = zmq_context.socket(zmq.SUB)
    pla_socket.setsockopt(zmq.CONFLATE, 1)
    pla_socket.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])
    pla_socket.connect("tcp://127.0.0.1:%s" % exp_config_.PORT_PLAN_2_GUI)

    poller = zmq.Poller()
    poller.register(obs_socket, zmq.POLLIN)
    poller.register(rob_socket, zmq.POLLIN)
    poller.register(pla_socket, zmq.POLLIN)

    logger.info("Waiting for modules to initialise...")
    time.sleep(8)

    """s_experiment = "resolutions"
    s_csv = "RES"
    exp_config_.set_property('CONF_VALUES',
                             [{'key': "RES_HORIZONTAL", 'cur': 60, 'min': 60, 'max': 1021, 'step': 120}])
    exp_config_.set_property('PATH_FNAME', 0)  # start path file identifier"""

    s_experiment = "control"
    s_csv = "SPB"
    exp_config_.set_property('CONF_VALUES',
                             [{'key': "X", 'cur': 0, 'min': 0, 'max': 0.1, 'step': 1}])
    exp_config_.set_property('PATH_FNAME', 0)  # start path file identifier
    exp_config_.set_property('ITERATION', 0)  # current iteration

    for scen in range(0, 7): #
        exp_config_.set_property('OBST_COURSE', scen)  # scenario to run
        # start experiment

        start_exp(exp_config_, logger, ctrl_socket, rob_socket, pla_socket, poller)
        logger.info("DONE with scenario #" + str(scen) + ", iteration #" + str(0))

    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



    send_ctr_cmd(ctrl_socket, "SHUTDOWN", logger, exp_config_)


if __name__ == "__main__":
    # setup logging
    format = "%(name)s - %(levelname)-8s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO)
    old_factory = logging.getLogRecordFactory()

    def record_factory(*args, **kwargs):
        record = old_factory(*args, **kwargs)
        return record

    logging.setLogRecordFactory(record_factory)

    main(ExpConfig(), logging.getLogger("sbExperiment"))