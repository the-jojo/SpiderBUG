import logging
import os
import sys
import tkinter as tk
from tkinter import ttk

import dill as pickle
import matplotlib
import matplotlib.animation as animation
import numpy
import numpy as np
import pyximport
import zmq
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)  # allow ctrl + c quitting

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

pyximport.install(setup_args={"include_dirs": numpy.get_include()}, language_level=3)

from src.utils.config import Config, default_config
from src.nav.ObstacleSegment import ObstacleSegment
from src.utils.modes import NavMode
from src.utils.sbMath import reduce_path


matplotlib.use("TkAgg")
style.use("ggplot")

LARGE_FONT = ("Verdana", 12)

config_ = Config()

f = Figure(figsize=(5, 10), dpi=100)
a1 = f.add_subplot(211)
a1.set_aspect('equal')
a1.set_title("Overview")

a2 = f.add_subplot(212, projection='3d')
a2.set_title("Web in Space-Time")

l_obst_points = []
l_obst_velocities = []
l_path_fut = []
l_path_past = []
l_planner_nodes = []
l_planner_edges = []
r_state = -1
n_rob_pos = None
mode = NavMode.MTG
a2_angle = 0

zmq_context, ctrl_socket, state_socket_o, state_socket_p, state_socket_n = None, None, None, None, None


def close_and_quit():
    global zmq_context, ctrl_socket, state_socket_o, state_socket_p, state_socket_n
    print("QUITTING...")
    try:
        ctrl_socket.close()
        state_socket_o.close()
        state_socket_p.close()
        state_socket_n.close()
        zmq_context.term()
    except:
        pass


def send_ctr_cmd(socket, command, logger, arg=None):
    """
    Broadcasts a control command to all modules. Resets plotting variables if RESTART is issued
    :param socket: command socket
    :param command: one of "START", "RESTART", "PAUSE", "STEP", "SHUTDOWN"
    :param logger: logger to log command to
    :param arg: additional arguments to append the broadcast (current configuration)
    """
    global l_obst_points, l_obst_velocities, l_path_fut, l_path_past, l_planner_nodes, l_planner_edges, r_state, \
        n_rob_pos, mode, a2_angle, a1, a2
    if command == "RESTART":
        l_obst_points = []
        l_obst_velocities = []
        l_path_fut = []
        l_path_past = []
        l_planner_nodes = []
        l_planner_edges = []
        n_rob_pos = None
        mode = NavMode.MTG
        a2_angle = 0
        a1.clear()
        a2.clear()

    socket.send(default_config['PUB_PREFIX'].encode() + pickle.dumps((command, arg)))
    logger.info("Sent ctrl command: %s" % command)


def poll_data(state_socket_o, state_socket_p, state_socket_n, poller, controller):
    """
    Polls data from other modules to display in live-view
    :param state_socket_o: socket to the sbPerception
    :param state_socket_p: socket to the sbRobot
    :param state_socket_n: socket to the sbPlanner
    :param poller: zmq poller to use
    :param controller: tk controller to schedule next polling
    """
    global l_obst_points, l_obst_velocities, l_path_fut, l_path_past, l_planner_nodes, l_planner_edges, r_state, \
        n_rob_pos, mode
    socks = dict(poller.poll(0))
    if state_socket_o in socks and socks[state_socket_o] == zmq.POLLIN:
        a_msg = state_socket_o.recv()
        # obstacles
        obst_segments: [ObstacleSegment] = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
        l_obst_points = []
        l_obst_velocities = []
        for obst in obst_segments:
            o_x = np.array([b.as_list_2d() for b in obst.get_boundary_points()])[:, 0].tolist()
            o_y = np.array([b.as_list_2d() for b in obst.get_boundary_points()])[:, 1].tolist()
            l_obst_points.append([o_x, o_y])
            l_obst_velocities.append(obst.get_avg_v_3d().as_list_3d())
    if state_socket_p in socks and socks[state_socket_p] == zmq.POLLIN:
        a_msg = state_socket_p.recv()
        # past and future path
        (plt_x_path, plt_y_path, plt_x_past, plt_y_past, r_state) = \
            pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
        l_path_fut = [plt_x_path, plt_y_path]
        l_path_past = [plt_x_past, plt_y_past]
    if state_socket_n in socks and socks[state_socket_n] == zmq.POLLIN:
        a_msg = state_socket_n.recv()

        l_planner_nodes, l_planner_edges, n_rob_pos, up_path = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
    controller.after(250, poll_data, state_socket_o, state_socket_p, state_socket_n, poller, controller)


def animate(_):
    global l_obst_points, l_obst_velocities, l_path_fut, l_path_past, l_planner_nodes, l_planner_edges, n_rob_pos, \
        a2_angle
    # clear plot
    a1.clear()
    if len(l_obst_points) > 0:
        # plot obstacle boundaries
        i = 0
        obst_col = ['orange', 'red', 'maroon', 'tomato', 'salmon', 'firebrick', 'crimson', 'darkorange']
        for obst_x, obst_y in l_obst_points:
            a1.scatter(obst_x, obst_y, s=2, color=obst_col[i % len(obst_col)])
            i += 1
    if len(l_planner_nodes) > 0:
        # plot nodes
        n_x = [n.x() for n in l_planner_nodes]
        n_y = [n.y() for n in l_planner_nodes]
        a1.scatter(n_x, n_y, s=22, marker='x', color='black')
    if len(l_planner_edges) > 0:
        # plot edges
        for edge in l_planner_edges:
            a1.plot([edge[0].x(), edge[1].x()], [edge[0].y(), edge[1].y()], linewidth=1, color='grey')
    if len(l_path_fut) > 0:
        # plot future path
        a1.plot(reduce_path(l_path_fut, 50)[0], reduce_path(l_path_fut, 50)[1], linewidth=1, color='navy')
    if len(l_path_past) > 0:
        # plot past path
        a1.scatter(reduce_path(l_path_past, 50)[0], reduce_path(l_path_past, 50)[1], s=1, color='turquoise')
    if n_rob_pos is not None:
        # plot robot pos
        a1.scatter(n_rob_pos.x(), n_rob_pos.y(), s=22, marker='x', color='green')

    if len(l_planner_nodes) > 0:
        # plot 3d nodes, edges
        n_x = [n.x() for n in l_planner_nodes]
        n_y = [n.y() for n in l_planner_nodes]
        n_z = [n.z() for n in l_planner_nodes]
        a2.clear()
        a2.scatter(n_x, n_y, n_z, color='black', s=22, marker='x')
        max_z = 2
        for edge in l_planner_edges:
            if edge[1].z() >= max_z:
                max_z = edge[1].z()
            a2.plot([edge[0].x(), edge[1].x()], [edge[0].y(), edge[1].y()], [edge[0].z(), edge[1].z()], linewidth=2,
                    color='grey')
        if n_rob_pos is not None:
            a2.scatter(n_rob_pos.x(), n_rob_pos.y(), n_rob_pos.z(), color='purple', s=22, marker='o')
        if len(l_obst_points) > 0:
            i = 0
            obst_col = ['orange', 'red', 'maroon', 'tomato', 'salmon', 'firebrick', 'crimson', 'darkorange']
            for (obst_x, obst_y), obst_v in zip(l_obst_points, l_obst_velocities):
                a2.scatter(obst_x, obst_y, np.zeros(len(obst_x)), s=5, color=obst_col[i % len(obst_col)])
                a2.plot([obst_x[0], obst_x[0]+max_z*obst_v[0]],
                        [obst_y[0], obst_y[0]+max_z*obst_v[1]],
                        [0, max_z*obst_v[2]],
                        linewidth=1.5, color=obst_col[i % len(obst_col)])
                a2.plot([obst_x[-1], obst_x[-1]+max_z*obst_v[0]],
                        [obst_y[-1], obst_y[-1]+max_z*obst_v[1]],
                        [0, max_z*obst_v[2]],
                        linewidth=1.5, color=obst_col[i % len(obst_col)])
                i += 1
        a2.view_init(30, a2_angle % 360)
        a2_angle += 2


class GuiApp(tk.Tk):

    def __init__(self, ctrl_socket, logger, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        self.ctrl_socket = ctrl_socket
        self.logger = logger

        tk.Tk.iconbitmap(self, default="data\\py_data\\spider.ico")
        tk.Tk.wm_title(self, "GUI Module")

        self.geometry("500x900-0+5")  # Width x Height + x_pos + y_pos

        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}

        for F in (StartPage, LiveViewPage, IndividualPage):
            frame = F(container, self, ctrl_socket, logger)

            self.frames[F] = frame

            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame(StartPage)

    def show_frame(self, cont):
        frame = self.frames[cont]
        frame.tkraise()

    def on_closing(self):
        self.logger.info("Closing GUI Module")
        send_ctr_cmd(self.ctrl_socket, "SHUTDOWN", self.logger)
        self.destroy()


class StartPage(tk.Frame):

    def __init__(self, parent, controller, ctrl_socket, logger):
        global config_
        self.ctrl_socket = ctrl_socket
        self.logger = logger

        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Main Menu", font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        but_indi = ttk.Button(self, text="Setup Simulation",
                              command=lambda: controller.show_frame(IndividualPage))
        but_indi.pack(pady=10, padx=10)

        but_view = ttk.Button(self, text="View Robot Live State",
                              command=lambda: controller.show_frame(LiveViewPage))
        but_view.pack(pady=10, padx=10)


class LiveViewPage(tk.Frame):

    def __init__(self, parent, controller, ctrl_socket, logger):
        global config_
        tk.Frame.__init__(self, parent)

        top_frame = tk.Frame(self, )
        button_frame = tk.Frame(self, )
        plot_frame = tk.Frame(self, bg="ghost white")

        top_frame.pack(side="top", fill=None, expand=True)
        button_frame.pack(side="top", fill=None, expand=False, pady=10)
        plot_frame.pack(side="bottom", fill="both", expand=True)

        label = tk.Label(top_frame, text="Robot's Live State", font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        button1 = ttk.Button(top_frame, text="Main Menu",
                             command=lambda: controller.show_frame(StartPage))
        button1.pack(side="left")
        button2 = ttk.Button(top_frame, text="Setup Simulation",
                             command=lambda: controller.show_frame(IndividualPage))
        button2.pack(side="right")

        but_start = ttk.Button(button_frame, text="Start", command=lambda: send_ctr_cmd(ctrl_socket, "START",
                                                                                        logger, config_))
        but_start.pack(side="left")

        but_pause = ttk.Button(button_frame, text="Pause", command=lambda: send_ctr_cmd(ctrl_socket, "PAUSE", logger))
        but_pause.pack(side="left")

        but_step = ttk.Button(button_frame, text="Step", command=lambda: send_ctr_cmd(ctrl_socket, "STEP",
                                                                                      logger, config_))
        but_step.pack(side="left")

        canvas = FigureCanvasTkAgg(f, plot_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        toolbar = NavigationToolbar2Tk(canvas, plot_frame)
        toolbar.update()
        canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)


class IndividualPage(tk.Frame):

    def __init__(self, parent, controller, ctrl_socket, logger):
        global config_
        tk.Frame.__init__(self, parent)

        # setup top, middle and bottom frame
        top_frame = tk.Frame(self, )
        button_frame = tk.Frame(self, )
        plot_frame = tk.Frame(self, bg="ghost white")
        top_frame.pack(side="top", fill="both", expand=False)
        button_frame.pack(side="top", fill=None, expand=False, pady=10)
        plot_frame.pack(side="bottom", fill="both", expand=True)

        label = tk.Label(top_frame, text="Setup Simulation", font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        button1 = ttk.Button(top_frame, text="Main Menu",
                             command=lambda: controller.show_frame(StartPage))
        button1.pack()

        # robot dropdown
        tk_var_robot = tk.StringVar(parent)
        tk_var_robot.set(next(key for key, value in config_.ROB_CHOICES.items() if value == config_.ROB_MODEL))
        popup_menu_robot = tk.OptionMenu(button_frame, tk_var_robot, *config_.ROB_CHOICES)
        lab_robot = tk.Label(button_frame, text="Choose a Robot")
        lab_robot.pack()
        popup_menu_robot.pack()
        def change_dropdown_2(*args):
            global config_
            config_.set_property('ROB_MODEL', config_.ROB_CHOICES[tk_var_robot.get()])
        tk_var_robot.trace('w', change_dropdown_2)

        # scenario dropdown
        tk_var_scenario = tk.StringVar(parent)
        tk_var_scenario.set(next(key for key, value in config_.SCEN_CHOICES.items() if value == config_.OBST_COURSE))
        popup_menu_scenario = tk.OptionMenu(button_frame, tk_var_scenario, *config_.SCEN_CHOICES)
        lab_setup = tk.Label(button_frame, text="Choose a Scenario")
        lab_setup.pack()
        popup_menu_scenario.pack()

        def change_dropdown_1(*args):
            global config_
            config_.set_property('OBST_COURSE', config_.SCEN_CHOICES[tk_var_scenario.get()])

        tk_var_scenario.trace('w', change_dropdown_1)

        but_start = ttk.Button(button_frame, text="Start",
                               command=lambda: send_ctr_cmd(ctrl_socket, "START", logger, config_))
        but_start.pack(pady=10, padx=10)

        but_rest = ttk.Button(button_frame, text="Restart",
                              command=lambda: send_ctr_cmd(ctrl_socket, "RESTART", logger, config_))
        but_rest.pack(pady=10, padx=10)

        but_pause = ttk.Button(button_frame, text="Pause", command=lambda: send_ctr_cmd(ctrl_socket, "PAUSE", logger))
        but_pause.pack(pady=10, padx=10)

        but_step = ttk.Button(button_frame, text="Step",
                              command=lambda: send_ctr_cmd(ctrl_socket, "STEP", logger, config_))
        but_step.pack(pady=10, padx=10)

        but_quit = ttk.Button(button_frame, text="Quit", command=controller.on_closing)
        but_quit.pack(pady=10, padx=10)

        but_view = ttk.Button(self, text="View Robot Live State",
                              command=lambda: controller.show_frame(LiveViewPage))
        but_view.pack(pady=10, padx=10)


def main(logger, is_main=0):
    global zmq_context, ctrl_socket, state_socket_o, state_socket_p, state_socket_n
    """
    port_ctrl=PORT_GUI_2_ALL,
    port_state_o=PORT_SENS_2_GUI,
    port_state_p=PORT_ROB_2_GUI,
    port_state_n=PORT_PLAN_2_GUI
    """
    # setup zmq control channel
    zmq_context = zmq.Context()
    ctrl_socket = zmq_context.socket(zmq.PUB)
    if is_main:
        ctrl_socket.bind("tcp://*:%s" % config_.PORT_GUI_2_ALL)
    else:
        ctrl_socket.connect("tcp://127.0.0.1:%s" % config_.PORT_GUI_2_ALL)

    # setup zmq state channel
    state_socket_o = zmq_context.socket(zmq.SUB)
    state_socket_o.setsockopt(zmq.CONFLATE, 1)
    state_socket_o.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])
    state_socket_o.connect("tcp://127.0.0.1:%s" % config_.PORT_SENS_2_GUI)

    state_socket_p = zmq_context.socket(zmq.SUB)
    state_socket_p.setsockopt(zmq.CONFLATE, 1)
    state_socket_p.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])
    state_socket_p.connect("tcp://127.0.0.1:%s" % config_.PORT_ROB_2_GUI)

    state_socket_n = zmq_context.socket(zmq.SUB)
    state_socket_n.setsockopt(zmq.CONFLATE, 1)
    state_socket_n.setsockopt_string(zmq.SUBSCRIBE, default_config['PUB_PREFIX'])
    state_socket_n.connect("tcp://127.0.0.1:%s" % config_.PORT_PLAN_2_GUI)

    poller = zmq.Poller()
    poller.register(state_socket_o, zmq.POLLIN)
    poller.register(state_socket_p, zmq.POLLIN)
    poller.register(state_socket_n, zmq.POLLIN)

    app = GuiApp(ctrl_socket, logger)

    app.protocol("WM_DELETE_WINDOW", app.on_closing)

    app.after(250, poll_data(state_socket_o, state_socket_p, state_socket_n, poller, app))
    ani = animation.FuncAnimation(f, animate, interval=250, fargs=())

    logger.info("READY")
    app.mainloop()


if __name__ == "__main__":
    # setup logging
    format_ = "%(name)s - %(levelname)-8s: %(message)s"
    logging.basicConfig(format=format_, level=logging.INFO)
    old_factory = logging.getLogRecordFactory()

    def record_factory(*args, **kwargs):
        record = old_factory(*args, **kwargs)
        return record

    logging.setLogRecordFactory(record_factory)

    try:
        main(logging.getLogger("sbGUI"), 1)
    finally:
        # clean up
        close_and_quit()
