# SpiderBUG 
_Mobile Robot Navigation using Dynamic Webs_

## Summary

The problem addressed is that of an autonomous robotic agent navigating an unknown and dynamically changing 2-D environment without the use of a global map. The particular focus is the scenario of a winewaiter robot desiring smooth but short paths. An adaptive obstacle navigation algorithm (SpiderBUG) is presented overcoming perceptual limitations of sensors and real-time requirements. The Tangent-BUG algorithm is adapted to compensate for obstacles moving with unknown velocities. Also, dynamic webs are used and introduced in a way to span through space-time, reducing the problem in free-space to graph traversal. Smoothness in our new algorithm is guaranteed both through the nature of the incorporated Tangent-BUG algorithm as well as through the use of shift-curves. The simulation results are shown to support the theoretical design's intentions. Guarantees persist in the face of cluttered, static and dynamic environments.

## This Project

**This project code accompanies a dissertation presented in part fulfilment of the requirements of the Master of Science Computing Science of the University of Glasgow**

The SpiderBUG path-planner is implemented in Python using the Pybullet physics library and tested against a range of static and dynamic environments.

This following document explains how to run the code to reproduce results from the dissertation. Also, an overview of the code is presented, explaining how the designed algorithm maps onto the included code.

## Licence

All work presented in this repository is available under the MIT licence.

## Quickstart Guide

To run the simulation environment, first make sure all the required python packages are installed:

``` bash
> pip install Cython numpy
> pip install git+git://github.com/the-jojo/pydubins.git
> python setup.py install
```

Then run each of the commands below in a new shell window/tab:

``` bash
> python src/sbPlanner.py
> python src/sbRobot.py
> python src/sbPerception.py
> python src/sbGUI.py
```

![Example Run](https://raw.githubusercontent.com/the-jojo/SpiderBUG/master/md_files/overview.gif "Usage of Simulation Environment")

*Example run of app (sped up 150%)*

### Manual

Once `READY` is printed by each module to their console, use the GUI interface to start a scenario. Select "Setup Simulation", then select the robot model and scenario from the dropdown lists. The image below explains the interface options.

![Live View](https://raw.githubusercontent.com/the-jojo/SpiderBUG/master/md_files/setup_page.PNG "Setup Page Explanation")

*Setup interface page explanation*

Click "Start" to start the simulation. "Pause" can be used to pause all modules and "Step" to run one iteration of each module, including the PyBullet simulation.

Click "View Robot Live State" to view a graphical representation of the planner's internal state. The image below explains the available elements of this interface page. Please note that elements of the 3D plot of the dynamic web and the obstacle may overlap when they should be underlapping, thus making it hard to interpret. 

![Live View](https://raw.githubusercontent.com/the-jojo/SpiderBUG/master/md_files/live_view.PNG "Live View Page Explanation")

*Live view interface page explanation*

The robot can be either a Turtebot 2 model or a Spherebot and the PyBullet OpenGL simulation environment is depicted and explained in the image below.

![Simulation View](https://raw.githubusercontent.com/the-jojo/SpiderBUG/master/md_files/pybullet.PNG "Pybullet Simulation Explanation")

*Pybullet Simulation explanation* 

## Documentation

File Structure:
 
``` markdown
├── data  # experiment data and jupyter notebook to explore data
|   ├── exp_data    # raw experiment data
|   |   ├── control     # control paths as seen in figure 4.2; TurtleBot model run once on all scenarios
|   |   ├── models      # data for section 4.3 Applicability to other Vehicular Movement Models
|   |   ├── obstacles   # data for section 4.4 Applicability to other Dynamic Environments
|   |   ├── resolutions # data for section 4.1 Algorithm Hyper-parameters - h_tolerance, d_tolerance
|   |   └── tolerances  # data for section 4.1 Algorithm Hyper-parameters - h_resolution
|   |       |   ... each of the above folders contains subfolders similar to the structure below ...
|   |       └── S_x       # 1 folder per scenario
|   |           └── i_x     # 1 folder per iteration
|   |               ├── py_objs     # folder containing python objects of the executed path and update intervals 
|   |               └── TOL_x.csv   # csv of the parameter values tested, corresponding python object files and final state
|   ├── pics        # pictures used in the thesis
|   ├── py_data     # pybullet data such as urdf files
|   └── sbAnalysis.ipynb  # Juypter Notebook used for data exploration 
|                           and to generate the graphs in the thesis
├── md_files # images embedded in this readme
├── src      # python source code
|    ├── bot                      # code relating to the robot in the simulation and the setup of the scenarios
|    ├── geom                     # code relating to geometrical operations (SpiderBUG's MTG mode)
|    ├── nav                      # code used by the sbPlanner for both the MTG and BF modes
|    ├── utils                    # configuration files, utility functions and Dubins path code
|    ├── intersection_profiler.py # code used to profile the obstacleSegment.get_intersect_with_path_3d() function for section 3.3.3 of the thesis
|    ├── sbExp.py                 # code used to collect the raw data for all the experiments
|    ├── sbGUI.py                 # GUI module
|    ├── sbPerception.py          # Perception module
|    ├── sbPlanner.py             # Planner module
|    └── sbRobot.py               # robot module
├── README.md # this readme file 
└── setup.py  # setup python file includes pip package requirements
```

In-depth description of the source code can be found in the documentation of each file. 

## SpiderBUG run on all Scenarios

![Scenario 1](https://raw.githubusercontent.com/the-jojo/SpiderBUG/master/md_files/scen_1.gif "Scenario 1")

*Scenario 1*

![Scenario 2](https://raw.githubusercontent.com/the-jojo/SpiderBUG/master/md_files/scen_2.gif "Scenario 2")

*Scenario 2*

![Scenario 3](https://raw.githubusercontent.com/the-jojo/SpiderBUG/master/md_files/scen_3.gif "Scenario 3")

*Scenario 3*

![Scenario 4](https://raw.githubusercontent.com/the-jojo/SpiderBUG/master/md_files/scen_4.gif "Scenario 4")

*Scenario 4*

![Scenario 5](https://raw.githubusercontent.com/the-jojo/SpiderBUG/master/md_files/scen_5.gif "Scenario 5")

*Scenario 5*

![Scenario 6](https://raw.githubusercontent.com/the-jojo/SpiderBUG/master/md_files/scen_6.gif "Scenario 6")

*Scenario 6*
