# memmo_planner_ros

## Prerequisites
- Ubuntu 20.04
- [rosdep](http://wiki.ros.org/rosdep)
- [vcstool](http://wiki.ros.org/vcstool)
- [Gurobi Optimizer](https://www.gurobi.com/downloads/)

## Installation
```
# Memmo Teleop
$ sudo pip3 install ds4drv
```

## Run
```
# Surface Planner
$ roslaunch surface_planner_ros surface_planner.launch

# Footstep Planner
$ roslaunch footstep_planner_ros footstep_planner.launch

# Memmo Teleop
$ sudo ds4drv
```
