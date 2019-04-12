# Multi_goal_navigation
Multiple goals navigation which is set by YAML file and the goals are send to Move_base with actionclient

# Introduction

This package is used to send a specific goal to a move_base topic which the goal is defined in a YAML file. This package is tested with 
husky simulator in UBUNTU 16.04 with ROS version Kinetic.

# How to Launch
For the husky simulation
```bash
$ sudo apt-get install ros-$YOUR_ROS_DISTRO-husky-navigation
```
in different terminal launch
```bash
$ roslaunch husky_gazebo husky_playpen.launch
```
```bash
$ roslaunch husky_viz view_robot.launch
```
```bash
$ roslaunch husky_navigation amcl_demo.launch 
```
for more information about husky, you can read in http://wiki.ros.org/husky_navigation


For the multi_goal_navigation
```bash
roslaunch multi_goal_driver multi_goal_driver.launch 

rosrun rviz rviz

```

#author
Joshua Phartogi https://github.com/Jphartogi
