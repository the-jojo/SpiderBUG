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
> python setup.py install
```

Then run each of the commands below in a new shell window/tab:

````bash
> python src/sbPlanner.py
> python src/sbRobot.py
> python src/sbPerception.py
> python src/sbGUI.py
````

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

Directories:
 - `Jupyter`    - 
 - `md_files`
 - `src`
 
 .
+-- _config.yml
+-- _drafts
|   +-- begin-with-the-crazy-ideas.textile
|   +-- on-simplicity-in-technology.markdown
+-- _includes
|   +-- footer.html
|   +-- header.html
+-- _layouts
|   +-- default.html
|   +-- post.html
+-- _posts
|   +-- 2007-10-29-why-every-programmer-should-play-nethack.textile
|   +-- 2009-04-26-barcamp-boston-4-roundup.textile
+-- _data
|   +-- members.yml
+-- _site
+-- index.html

## SpiderBUG run on all Scenarios

...
