import logging
import random

import pyximport
import numpy
import dill as pickle
import matplotlib
import zmq
import codecs
import numpy as np
import json

pyximport.install(setup_args={"include_dirs": numpy.get_include()}, language_level=3)

from src.utils.config import Config, default_config
from src.bot.ObstacleSegment import ObstacleSegment
from src.utils.modes import NavMode

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style

from tkinter import *
import tkinter as tk
from tkinter import ttk

LARGE_FONT = ("Verdana", 12)
style.use("ggplot")

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


def reduce_path(path):
    y = path[0::int(len(path) / min(50, len(path)))]
    y.append(path[-1])
    return y


def send_ctr_cmd(socket, command, logger, arg=None):
    global l_obst_points, l_obst_velocities, l_path_fut, l_path_past, l_planner_nodes, l_planner_edges, r_state, n_rob_pos, mode, a2_angle, a1, a2
    if command == "RESTART":
        l_obst_points = []
        l_obst_velocities = []
        l_path_fut = []
        l_path_past = []
        l_planner_nodes = []
        l_planner_edges = []
        r_eval = (-1, 0, 0)
        n_rob_pos = None
        mode = NavMode.MTG
        a2_angle = 0
        a1.clear()
        a2.clear()

    socket.send(default_config['PUB_PREFIX'].encode() + pickle.dumps((command, arg)))
    logger.info("Sent ctrl command: %s" % command)

def poll_data(state_socket_o, state_socket_p, state_socket_n, poller, controller):
    global l_obst_points, l_obst_velocities, l_path_fut, l_path_past, l_planner_nodes, l_planner_edges, r_eval, n_rob_pos, mode
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
        (plt_x_path, plt_y_path, plt_x_past, plt_y_past, r_state) = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
        l_path_fut = [plt_x_path, plt_y_path]
        l_path_past = [plt_x_past, plt_y_past]
    if state_socket_n in socks and socks[state_socket_n] == zmq.POLLIN:
        a_msg = state_socket_n.recv()

        l_planner_nodes, l_planner_edges, n_rob_pos, up_path = pickle.loads(a_msg[len(default_config['PUB_PREFIX']):])
    controller.after(250, poll_data, state_socket_o, state_socket_p, state_socket_n, poller, controller)


def animate(_):
    global l_obst_points, l_obst_velocities, l_path_fut, l_path_past, l_planner_nodes, l_planner_edges, n_rob_pos, a2_angle
    # clear plot
    a1.clear()
    if len(l_obst_points) > 0:
        # plot obstacle boundaries
        i = 0
        obst_col = ['orange', 'red', 'maroon', 'tomato', 'salmon', 'firebrick', 'crimson', 'darkorange']
        for obst_x, obst_y in l_obst_points:
            a1.scatter(obst_x, obst_y, s=2, color=obst_col[ i% len(obst_col)])
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
        a1.plot(reduce_path(l_path_fut)[0], reduce_path(l_path_fut)[1], linewidth=1, color='navy')
    if len(l_path_past) > 0:
        # plot past path
        a1.scatter(reduce_path(l_path_past)[0], reduce_path(l_path_past)[1], s=1, color='turquoise')
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
        a2.view_init(30, a2_angle%360)
        a2_angle += 2


class GuiApp(tk.Tk):

    def __init__(self, ctrl_socket, logger, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        self.ctrl_socket = ctrl_socket
        self.logger = logger

        tk.Tk.iconbitmap(self, default="data\\spider.ico")
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

    def onClosing(self):
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

        but_indi = ttk.Button(self, text="Run 1 Simulation",
                              command=lambda: controller.show_frame(IndividualPage))
        but_indi.pack(pady=10, padx=10)

        but_expe = ttk.Button(self, text="Run Experiments",
                              command=lambda: controller.show_frame(ExperimentPage))
        but_expe.pack(pady=10, padx=10)

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

        label = tk.Label(top_frame, text="View Robot's Live State", font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        button1 = ttk.Button(top_frame, text="Main Menu",
                             command=lambda: controller.show_frame(StartPage))
        button1.pack(side="left")
        button2 = ttk.Button(top_frame, text="Run 1 Simulation",
                             command=lambda: controller.show_frame(IndividualPage))
        button2.pack(side="right")

        but_start = ttk.Button(button_frame, text="Start", command=lambda: send_ctr_cmd(ctrl_socket, "START",
                                                                                        logger, config_))
        but_start.pack(side="left")

        but_pause = ttk.Button(button_frame, text="Pause", command=lambda: send_ctr_cmd(ctrl_socket, "PAUSE", logger))
        but_pause.pack(side="left")

        but_step = ttk.Button(button_frame, text="Step", command=lambda: send_ctr_cmd(ctrl_socket, "STEP", logger, config_))
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

        label = tk.Label(top_frame, text="Run 1 Simulation", font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        button1 = ttk.Button(top_frame, text="Main Menu",
                             command=lambda: controller.show_frame(StartPage))
        button1.pack()

        # Create a Tkinter variable
        tkvar = StringVar(parent)
        # Dictionary with options
        tkvar.set(next(key for key, value in config_.SCEN_CHOICES.items() if value == config_.OBST_COURSE))

        popupMenu = OptionMenu(button_frame, tkvar, *config_.SCEN_CHOICES)
        lab_setup = Label(button_frame, text="Choose a setup")
        lab_setup.pack()
        popupMenu.pack()

        # on change dropdown value
        def change_dropdown(*args):
            global config_
            config_.set_property('OBST_COURSE', config_.SCEN_CHOICES[tkvar.get()])

        # link function to change dropdown
        tkvar.trace('w', change_dropdown)

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

        but_quit = ttk.Button(button_frame, text="Quit", command=controller.onClosing)
        but_quit.pack(pady=10, padx=10)

        but_view = ttk.Button(self, text="View Robot Live State",
                              command=lambda: controller.show_frame(LiveViewPage))
        but_view.pack(pady=10, padx=10)


"""class ExperimentPage(tk.Frame):

    def __init__(self, parent, controller, ctrl_socket, logger):
        global config_
        tk.Frame.__init__(self, parent)

        top_frame = tk.Frame(self, )
        button_frame = tk.Frame(self, )
        bottom_frame = tk.Frame(self, bg="ghost white")

        top_frame.pack(side="top", fill="both", expand=False)
        button_frame.pack(side="top", fill=None, expand=False, pady=10)
        bottom_frame.pack(side="bottom", fill="both", expand=True, padx=10)

        label = tk.Label(top_frame, text="Setup Experiment to Run", font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        button1 = ttk.Button(top_frame, text="Main Menu",
                             command=lambda: controller.show_frame(StartPage))
        button1.pack()

        tk.Label(bottom_frame, text="Config Key").grid(row=0, column=0)
        tk.Label(bottom_frame, text="Value Min").grid(row=0, column=2)
        tk.Label(bottom_frame, text="Value Max").grid(row=0, column=3)
        tk.Label(bottom_frame, text="Step Size").grid(row=0, column=4)

        entry_grid = []
        n_entries = 3

        def auto_fill(i):
            min_val = config_.get_property(entry_grid[i - 1][0].get() + "_MIN")
            max_val = config_.get_property(entry_grid[i - 1][0].get() + "_MAX")
            step_val = config_.get_property(entry_grid[i - 1][0].get() + "_STEP")
            if min_val is not None:
                entry_grid[i - 1][2].delete(0, tk.END)
                entry_grid[i - 1][2].insert(0, min_val)
            if max_val is not None:
                entry_grid[i - 1][3].delete(0, tk.END)
                entry_grid[i - 1][3].insert(0, max_val)
            if step_val is not None:
                entry_grid[i - 1][4].delete(0, tk.END)
                entry_grid[i - 1][4].insert(0, step_val)

        for i in np.arange(1, n_entries):
            entry_grid.append([])
            tmp = entry_grid[i-1]
            tmp.append(tk.Entry(bottom_frame))
            tmp.append(tk.Button(bottom_frame, text='Fill', command=lambda i=i: auto_fill(i)))
            tmp.append(tk.Entry(bottom_frame, width=5))
            tmp.append(tk.Entry(bottom_frame, width=5))
            tmp.append(tk.Entry(bottom_frame, width=5))
            tmp[0].grid(row=i, column=0, sticky='ew')
            tmp[1].grid(row=i, column=1, padx=2, sticky='ew')# sticky=tk.W
            tmp[2].grid(row=i, column=2)
            tmp[3].grid(row=i, column=3)
            tmp[4].grid(row=i, column=4)

        entry_grid[0][0].insert(0, "TURN_RADIUS")

        lab_runs = Label(bottom_frame, text="Runs per Config")
        entry_runs = tk.Entry(bottom_frame, width=5)
        lab_runs.grid(row=n_entries, column=0)
        entry_runs.grid(row=n_entries, column=1)
        entry_runs.insert(0, 1)

        # Create a Tkinter variable
        tkvar = StringVar(parent)
        # Dictionary with options
        tkvar.set(next(key for key, value in config_.SCEN_CHOICES.items() if value == config_.OBST_COURSE))
        popupMenu = OptionMenu(bottom_frame, tkvar, *config_.SCEN_CHOICES)
        lab_setup = Label(bottom_frame, text="Choose a setup")
        lab_setup.grid(row=n_entries+1, column=0)
        popupMenu.grid(row=n_entries+1, column=1, sticky='ew', columnspan=2)
        # on change dropdown value
        def change_dropdown(*args):
            global config_
            config_.set_property('OBST_COURSE', config_.SCEN_CHOICES[tkvar.get()])
        # link function to change dropdown
        tkvar.trace('w', change_dropdown)

        lab_fname = Label(bottom_frame, text="Output File name")
        entry_fname = tk.Entry(bottom_frame)
        lab_fname.grid(row=n_entries+2, column=0)
        entry_fname.grid(row=n_entries+2, column=1, columnspan=2)
        entry_fname.insert(0, "results")

        vals_to_run = []
        e_name = ""
        iterat = 1
        def start_experiment():
            global config_
            nonlocal iterat, vals_to_run
            logger.info("Starting Experiment")
            but_start['state'] = 'disabled'
            #config_.set_property('HEADLESS', True)
            for i, entries in enumerate(entry_grid):
                c_key = entries[0].get()
                if c_key is not None and c_key is not "":
                    c_min = float(entries[2].get())
                    c_max = float(entries[3].get())
                    c_step = float(entries[4].get())
                    vals_to_run.append({'key': c_key, 'cur': c_min-c_step, 'min': c_min, 'max': c_max, 'step': c_step})
            if len(vals_to_run) <= 0:
                logger.info("No configurations to check")
                but_start['state'] = 'normal'
                #config_.set_property('HEADLESS', False)
                return
            f = open("results\\" + entry_fname.get() + "_" + str(config_.get_property('OBST_COURSE')) + ".csv", "w+")
            if len(vals_to_run) > 1:
                f.write(vals_to_run[0]['key'] + ',' + vals_to_run[1]['key'] + ",iteration,final_state,path_length,smoothness,path\n")
            else:
                f.write(vals_to_run[0]['key'] + ",iteration,final_state,path_length,smoothness,path\n")
            f.close()
            iterat = 1
            run_experiment(True)

        def run_experiment(first_run=False):
            nonlocal vals_to_run, e_name, iterat
            global r_eval, l_path_past, config_

            if not first_run and (r_eval[0] != 1 or r_eval[0] != 2):
                # called by mistake
                pass

            f = open("results\\" + entry_fname.get() + "_" + str(config_.get_property('OBST_COURSE')) + ".csv", "a+")
            if not first_run:
                path_f_id = random.randrange(1, 10**7)
                f.write(str(iterat) + "," + str(r_eval[0]) + "," + str(r_eval[1]) + "," + str(r_eval[2]) + "," + str(path_f_id) + "\n")
                with open('results\\py_objs\\' + str(path_f_id) + '.po', 'wb') as pyf:
                    pickle.dump(l_path_past, pyf)
                r_eval = (-1, 0, 0)
                l_path_past = []

            if not first_run and iterat < int(entry_runs.get()):
                # more runs for this experiment
                logger.info("Next iteration for config")
                iterat += 1
                if len(vals_to_run) > 1:
                    e_name = str(vals_to_run[0]['cur']) + "," + str(vals_to_run[1]['cur']) + ","
                    f.write(e_name)
                else:
                    e_name = str(vals_to_run[0]['cur']) + ","
                    f.write(e_name)
                send_ctr_cmd(ctrl_socket, 'RESTART', logger, config_)
                f.close()
                return
            else:
                logger.info("Next config")

            v_con_0 = vals_to_run[0]
            v_key_0 = v_con_0['key']
            if len(vals_to_run) > 1:
                v_con_1 = vals_to_run[1]
                v_key_1 = v_con_1['key']
                if v_con_1['cur'] < v_con_1['max']:
                    # iterate through second var first
                    v_con_1['cur'] = round(v_con_1['cur'] + v_con_1['step'],
                                           len(str(v_con_1['step']).split('.')[1]))
                    config_.set_property(v_key_1, v_con_1['cur'])
                    iterat = 1

                    e_name = str(v_con_0['cur']) + "," + str(v_con_1['cur']) + ","
                    f.write(e_name)

                    send_ctr_cmd(ctrl_socket, 'RESTART', logger, config_)
                else:
                    # done iterating through sec. Change back to min and iterate first var once
                    v_con_1['cur'] = v_con_1['min']
                    config_.set_property(v_key_1, v_con_1['cur'])
                    if v_con_0['cur'] < v_con_0['max']:
                        v_con_0['cur'] = round(v_con_0['cur'] + v_con_0['step'],
                                               len(str(v_con_0['step']).split('.')[1]))
                        config_.set_property(v_key_0, v_con_0['cur'])
                        iterat = 1

                        e_name = str(v_con_0['cur']) + "," + str(v_con_1['cur']) + ","
                        f.write(e_name)

                        send_ctr_cmd(ctrl_socket, 'RESTART', logger, config_)
                    else:
                        # done
                        logger.info("Done with Experiments")
                        but_start['state'] = 'normal'
                        r_eval = (-1, 0, 0)
                        vals_to_run = []
                        iterat = 1
                        config_ = Config()

            else:
                if v_con_0['cur'] < v_con_0['max']:
                    # iterate through first var
                    v_con_0['cur'] = round(v_con_0['cur'] + v_con_0['step'],
                                           len(str(v_con_0['step']).split('.')[1]))
                    config_.set_property(v_key_0, v_con_0['cur'])
                    iterat = 1

                    e_name = str(v_con_0['cur']) + ","
                    f.write(e_name)

                    send_ctr_cmd(ctrl_socket, 'RESTART', logger, config_)
                else:
                    # done
                    logger.info("Done with Experiments")
                    but_start['state'] = 'normal'
                    config_ = Config()
                    vals_to_run = []
                    r_eval = (-1, r_eval[1], r_eval[2])
            f.close()

        but_start = tk.Button(bottom_frame,
                              text='Start',
                              command=lambda: start_experiment())
        but_start.grid(row=n_entries+3, column=0, sticky='ew', pady=4)

        lab_state_lab_0 = Label(bottom_frame, text="State")
        lab_state_lab_1 = Label(bottom_frame, text="P Length")
        lab_state_lab_2 = Label(bottom_frame, text="P Smooth")
        lab_state_lab_0.grid(row=n_entries+4, column=0, sticky='ew')
        lab_state_lab_1.grid(row=n_entries+4, column=1, sticky='ew')
        lab_state_lab_2.grid(row=n_entries+4, column=2, sticky='ew')
        lab_state_0 = Label(bottom_frame, text="N/A")
        lab_state_1 = Label(bottom_frame, text="0")
        lab_state_2 = Label(bottom_frame, text="0")
        lab_state_0.grid(row=n_entries+5, column=0, sticky='ew')
        lab_state_1.grid(row=n_entries+5, column=1, sticky='ew')
        lab_state_2.grid(row=n_entries+5, column=2, sticky='ew')

        def update_state():
            global r_eval, l_path_past
            if r_eval[0] == -1: lab_state_0['text'] = 'N/A'
            if r_eval[0] == 0: lab_state_0['text'] = 'running'
            if r_eval[0] == 1: lab_state_0['text'] = 'complete'
            if r_eval[0] == 2: lab_state_0['text'] = 'crashed'
            lab_state_1['text'] = "{:.3f}".format(r_eval[1])
            lab_state_2['text'] = "{:.3f}".format(r_eval[2])

            if (r_eval[0] == 1 or r_eval[0] == 2) and but_start['state'] == 'disabled':
                run_experiment()
                controller.after(3000, update_state)  # reschedule event in 3 seconds
            else:
                controller.after(500, update_state)

        controller.after(500, update_state())"""


def main(logger, is_main=0):
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

    app.protocol("WM_DELETE_WINDOW", app.onClosing)

    #ani1 = animation.FuncAnimation(f, poll_data, interval=250, fargs=())
    app.after(250, poll_data(state_socket_o, state_socket_p, state_socket_n, poller, app))
    ani2 = animation.FuncAnimation(f, animate, interval=250, fargs=())

    logger.info("READY")
    app.mainloop()

    """
        def update():
        print("hello")
        top.after(1000, update)  # reschedule event in 2 seconds
        top.after(1000, update())
    """


if __name__ == "__main__":
    # setup logging
    format = "%(name)s - %(levelname)-8s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO)
    old_factory = logging.getLogRecordFactory()

    def record_factory(*args, **kwargs):
        record = old_factory(*args, **kwargs)
        return record

    logging.setLogRecordFactory(record_factory)

    main(logging.getLogger("sbGUI"), 1)
