hubo_ros_core
=============

Core ROS support for the Hubo humanoid.

This includes message/action definitions, the basic interface to hubo-ach, and a trajectory action interface built atop hubo-motion.

Repository structure
--------------------
This repository contains a Catkin worksace for ROS Groovy+, and is incompatible with ROS Fuerte and earlier. Versions of this software for earlier versions of ROS will not be actively developed or maintained!

This repository is structured around 3 core packages:

1.  hubo_robot_msgs - Message and action definitions that cover the Hubo robot. Everything from joint states to actions for walking and trajectory execution. Note that several of these message and action definitions are based directly off/duplicate equivalent functionality for the PR2. This is to avoid any dependency on the PR2 software stack.

2.  hubo_ach_ros_bridge - Very basic joint-level control and state publishing. In addition, this package provides ROS clocking and robot model support for RVIZ.

3.  hubo_trajectory_interface - JointTrajectoryAction controller and interface to the Hubo. Contains all the code for the interface and the configuration parameters to load a full-body trajectory controller.

Stability and development status
--------------------------------
hubo_robot_msgs has been stable for several months. It should be considered stable for all uses. We do not intend to change any message/action interfaces provided unless required to do so by a critical bug, and if/when we do add additional functionality, compatibility with all original functionality will be maintained.

hubo_ach_ros_bridge is under semi-active development. The core functionality has been tested several times, both in simulation and the real robot. This package is updated as necessary to maintain compatibility with hubo-ach.

This package will be updated in the comming weeks to provide compatibility with DRC-Hubo and to replace the joint name-to-hubo-ach indexing currently used with a parameter approach to support both Hubo+ and DRC-Hubo.

hubo_trajectory_interface is under active development and will continue to be for the forseable future. As we use this package extensively for our own development, we will attempt to maintain functionality with both simulated and real Hubo platforms. Note that all external interfaces to this package should be considered stable, and further development should not effect the outwardly-visible behavior of this package.

Build and running instructions
------------------------------
To build all packages in this repository:

```
(in the hubo_ros_core directory)
$ catkin_make
```
To build a particular package in the repository:

```
(in the hubo_ros_core directory)
$ catkin_make <package name>
```
To use, you must source the workspace:

```
(in the hubo_ros_core directory)
$ source devel/setup.bash
```

For usage information and instructions on running components of these packages together, see the repository [Wiki](https://github.com/WPI-ARC/hubo_ros_core/wiki).
