# added function

基于这个[项目](https://github.com/Jphartogi/Multi_goal_navigation.git)，我改进了程序，删除了程序中多余的部分。现在我可以添加无限的航点(发布move_base期望位置)，对于无人机，我增加了程序执行后自动降落的功能。除此以外，程序中的一些小错误也得到了修复。

分界线下是这个项目的原README文件。

Based on this [project](https://github.com/Jphartogi/Multi_goal_navigation.git), I improved the program and deleted the redundant parts of the program. Now I can add unlimited waypoints, and for drones, I added the function to land automatically after the program is executed. In addition to this, some minor bugs in the program have been fixed.

Here is the readme file of the original program.👇

---
# Multi_goal_navigation
Multiple goals navigation which is set by YAML file and the goals are send to Move_base with actionclient

# Introduction

This package is used to send a specific goal to a move_base topic which the goal is defined in a YAML file. This package is tested with 
husky simulator in UBUNTU 16.04 with ROS version Kinetic and Melodic.

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

# Author
Joshua Phartogi https://github.com/Jphartogi
Mafumaful https://github.com/Mafumaful
