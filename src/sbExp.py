import csv
import errno
import logging
import os
import time
from copy import deepcopy

import dill as pickle
import numpy as np
import pyximport
import zmq

pyximport.install(setup_args={"include_dirs": np.get_include()}, language_level=3)

from src.utils.config import Config, default_config, ExpConfig

base_dir = "..\\Jupyter\\exp_data\\"
s_experiment = "tolerances"
s_csv = "TOL"
path_f_c = 0


def exists_csv(n_scenario, n_iteration):
    """Check if the csv corresponding to the experiment, scenario and iteration exists"""
    f_name = str(base_dir) + str(s_experiment) + "\\S_" + str(n_scenario) + "\\i_" \
        + str(n_iteration) + "\\" + str(s_csv) + "_" + str(n_scenario) + ".csv"
    return os.path.isfile(f_name)


def open_csv(n_scenario, n_iteration, mode="a+"):
    """
    Opens the csv corresponding to the experiment, scenario and iteration.
    Creates the csv and any folders leading to the csv if necessary
    """
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
    """
    Opens the ..._path.po file corresponding to the experiment, scenario, iteration and path number.
    Creates any folders leading to the file if necessary and opens in overwrite mode.
    """
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
    """
    Sends a control command down the control socket
    """
    socket.send(default_config['PUB_PREFIX'].encode() + pickle.dumps((command, arg)))
    logger.info("Sent ctrl command: %s" % command)


def poll(poller, rob_socket, pla_socket, up_path=None):
    """
    Polls the sockets connected to the sbRobot and sbPlanner modules for the executed path, robot state and planner
    update points.
    Will keep the last up_path (planner update points) if none are received from the socket
    :param poller: ZMQ poller
    :param rob_socket: socket connected to sbRobot
    :param pla_socket: socket connected to sbPlanner
    :param up_path: previously received planner update points
    :return: robot state, past path, update points
    """
    socks = dict(poller.poll(0))
    r_state, r_path = 0, None
    if rob_socket in socks and socks[rob_socket] == zmq.POLLIN:
        # receive executed path and robot state
        a_msg = rob_socket.recv()
        _, _, plt_x_past, plt_y_past, r_state = pickle.loads(
            a_msg[len(default_config['PUB_PREFIX']):])
        r_path = [plt_x_past, plt_y_past]
    if pla_socket in socks and socks[pla_socket] == zmq.POLLIN:
        # receive planner update points
        a_msg = pla_socket.recv()
        _, _, _, up_path = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])

    return r_state, r_path, up_path


def run_exp(conf_vals: [], conf_vals_set: [],
            ctrl_socket: zmq.Socket, rob_socket: zmq.Socket, pla_socket: zmq.Socket, poller: zmq.Poller,
            logger: logging.Logger, config_: ExpConfig):
    """
    Runs one experiment iteration on one scenario with all possible configuration values.
    Eg. if A in the range [1..4] and B in [1..2] should be run on 6 scenarios 3 times each, this function is called
    6 x 3 = 18 times. Each time it loops through A and then B in a kind of double for loop (recursively) such that
    each combination of A and B are tested.
    - Sets A and B in the config to their values,
    - writes the current config to the csv,
    - runs the simulation,
    - waits until robot indicates crash or success
    - writes result to csv and path file
    - continue with next combination
    :param conf_vals: the configuration values left to set. If empty then start experiment run
    :param conf_vals_set: the configuration values already set.
    :param ctrl_socket: control socket
    :param rob_socket: sbRobot socket
    :param pla_socket:sbPlanner socket
    :param poller: poller
    :param logger: logger
    :param config_: config
    """
    global path_f_c
    # base case
    if len(conf_vals) == 0:
        logger.info("Running Exp with: " + str(conf_vals_set))

        # write config to file
        f_csv, f_name = open_csv(config_.OBST_COURSE, config_.ITERATION)
        for sv in conf_vals_set:
            f_csv.write(str(sv) + ",")
        f_csv.close()

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


def setup_exp(exp_config_: ExpConfig, logger: logging.Logger,
              ctrl_socket: zmq.Socket, rob_socket: zmq.Socket,
              pla_socket: zmq.Socket, poller: zmq.Poller):
    """
    Sets the experiment up. Called for each scenario and iteration. Calls run_exp().
    Picks up the experiment where it was left off by consulting the csv. Won't run anything
    if all configuration values for this scenario and iteration were already tested.
    :param exp_config_: config
    :param logger: logger
    :param ctrl_socket: control socket
    :param rob_socket: sbRobot socket
    :param pla_socket: sbPlanner socket
    :param poller: poller
    """
    global path_f_c

    if not exists_csv(exp_config_.OBST_COURSE, exp_config_.ITERATION):
        # the csv for this scenario and iteration does not exist yet
        # open (create) the csv
        f, f_name = open_csv(exp_config_.OBST_COURSE, exp_config_.ITERATION, mode="w+")

        # print header of csv to console and to the csv
        for vtr in exp_config_.CONF_VALUES:
            logger.info("key: " + vtr['key'])
            f.write(vtr['key'] + ",")
        f.write("final_state,path\n")
        f.close()
        logger.info("Wrote headers to " + f_name)

        # reset the path counter
        path_f_c = exp_config_.PATH_FNAME

        # run the experiment
        run_exp(deepcopy(exp_config_.CONF_VALUES), [],
                ctrl_socket, rob_socket, pla_socket, poller,
                logger, exp_config_)
    else:
        # the csv exists, so we need to pick up were we left off

        # find the last tested configuration values
        last_conf_vals = []
        f, f_name = open_csv(exp_config_.OBST_COURSE, exp_config_.ITERATION, mode="r")
        csvReader = csv.reader(f, delimiter=',')
        for j, row in enumerate(csvReader):
            if j != 0:
                last_conf_vals = row[:len(exp_config_.CONF_VALUES)]
                path_f_c = int(row[-1]) + 1

        # build the new configuration values
        new_conf_vals = []
        for lk, cv in zip(last_conf_vals, exp_config_.CONF_VALUES):
            cv = deepcopy(cv)
            cv['cur'] = float(lk)
            new_conf_vals.append(cv)
        new_conf_vals[-1]['cur'] = new_conf_vals[-1]['cur'] + new_conf_vals[-1]['step']

        # check if all tests were already completed
        if new_conf_vals[-1]['cur'] > new_conf_vals[-1]['max']:
            logger.info("skipping..")
            return

        # run the experiment
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

    s_experiment = "models"  # folder name
    s_csv = "MOD"  # csv prefix
    exp_config_.set_property('CONF_VALUES',
             [{'key': "ROB_MODEL", 'cur': 0, 'min': 0, 'max': 2.1, 'step': 1}])  # config value range to test
    exp_config_.set_property('PATH_FNAME', 0)  # start path file identifier

    for scen in range(1, 7):  # loop all scenarios 1 to 6
        for iteration in range(0, 6):  # loop all iterations 0 to 5
            exp_config_.set_property('ITERATION', iteration)  # set current iteration in config
            exp_config_.set_property('OBST_COURSE', scen)  # set current scenario in config

            # start experiment
            setup_exp(exp_config_, logger, ctrl_socket, rob_socket, pla_socket, poller)
            logger.info("DONE with scenario #" + str(scen) + ", iteration #" + str(iteration))

    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    send_ctr_cmd(ctrl_socket, "SHUTDOWN", logger, exp_config_)


if __name__ == "__main__":
    # setup logging
    format_ = "%(name)s - %(levelname)-8s: %(message)s"
    logging.basicConfig(format=format_, level=logging.INFO)
    old_factory = logging.getLogRecordFactory()

    def record_factory(*args, **kwargs):
        record = old_factory(*args, **kwargs)
        return record

    logging.setLogRecordFactory(record_factory)

    main(ExpConfig(), logging.getLogger("sbExperiment"))
