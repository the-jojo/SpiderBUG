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

*Example run of app (sped up 150%)*

![Example]https://raw.githubusercontent.com/the-jojo/SpiderBUG/master/md_files/overview.gif "Usage of Simulation Environment")

### Manual

Once `READY` is printed by each module, use the GUI interface to start a scenario. Select ...

## Documentation

Code ...
