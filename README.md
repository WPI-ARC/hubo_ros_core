hubo_ros_core
=============

Core ROS support for the Hubo humanoid.

This includes message/action definitions, the basic interface to hubo-ach, and a trajectory action interface built atop hubo-motion.

Repository structure
--------------------
Unlike earlier Catkinized software we have provided, this repository does not contain a Catkin workspace. As we expect that other teams will be well on their way to migrating to ROS Groovy, the difficulties of managing multiple workspaces do not justify the convenience of distributing these packages in their own workspace. As such, you will need to clone this repository inside the `src/` directory of an existing Catkin workspace.

Please note that this software is structured for ROS Groovy+, and is incompatible with ROS Fuerte and earlier. Versions of this software for earlier versions of ROS will not be actively developed or maintained, but historical versions may be available upon request.

This repository is structured around 5 core packages:

1.  `hubo_robot_msgs` - Message and action definitions that cover the Hubo robot. Everything from joint states to actions for walking and trajectory execution. Note that several of these message and action definitions are based directly off/duplicate equivalent functionality for the PR2. This is to avoid any dependency on the PR2 software stack.

2.  `hubo_sensor_msgs` - Message and service definitions for sensors on the Hubo robot. This is currently empty, as the basic sensors are already provided in `geometry_msgs/WrenchStamped` and `sensor_msgs/Imu`.

3.  `hubo_system_msgs` - Message and service definitions for the system software and configuration of the Hubo robot. This is currently empty and awaiting proposed message/service types.

3.  `hubo_ach_ros_bridge` - Very basic joint-level control and state publishing. In addition, this package provides ROS clocking and robot model support for RVIZ.

4.  `hubo_trajectory_interface` - JointTrajectoryAction controller and interface to the Hubo. Contains all the code for the interface and the configuration parameters to load a full-body trajectory controller.

In addition, it contains two "helper" packages that serve to simplfy the inclusion of hubo-ach and hubo-motion headers. Since not everyone will want (or be able) to install hubo-ach and hubo-motion to their system, these packages provide an alternate means of finding the headers needed from these projects.

1.  `hubo_components` - Wrapper around `hubo-ach/include` and `hubo-motion-rt/include`. This is done by making `hubo_components/include/hubo_components` contain all files in `hubo-ach/include` and `hubo-motion-rt/include` via symlink.

Stability and development status
--------------------------------
`hubo_robot_msgs` has been stable for several months. It should be considered stable for all uses. We do not intend to change any message/action interfaces provided unless required to do so by a critical bug, and if/when we do add additional functionality, compatibility with all original functionality will be maintained.

`hubo_sensor_msgs` is currently empty and awaiting Hubo-specific sensor message definitions. If there is a sensor aboard the Hubo for which no existing ROS message type matches, please propose an addition to this package, and it will be added. Please note that the IMU/tilt sensors are already supported by `sensor_msgs/Imu` (enter NAN if a particular value *inside* a field [i.e. Az or Wx] is not available from the sensor), and the force sensors are supported by `geometry_msgs/WrenchStamped` (once again, use NAN for elements *inside* a field not provided by the sensor [i.e. Fz or Tx]).

`hubo_system_msgs` is currently empty and awaiting Hubo-specific system control messages/actions/services. In particular, this package is intended to provide messages for system (computers,software, etc) status and conficuration and services for their reconfiguration. If there is a system control or status you want represented in an ROS message, propose it and it will be added to this package.

`hubo_ach_ros_bridge` is under semi-active development. The core functionality has been tested several times, both in simulation and the real robot. This package is updated as necessary to maintain compatibility with hubo-ach.

This package will be updated in the comming weeks to provide compatibility with DRC-Hubo and to replace the joint name-to-hubo-ach indexing currently used with a parameter approach to support both Hubo+ and DRC-Hubo.

`hubo_trajectory_interface` is under active development and will continue to be for the forseable future. As we use this package extensively for our own development, we will attempt to maintain functionality with both simulated and real Hubo platforms. Note that all external interfaces to this package should be considered stable, and further development should not effect the outwardly-visible behavior of this package.

Depencies
---------
1.  Full ROS Groovy installation - on Ubuntu systems: `$ sudo apt-get install ros-groovy-desktop-full`

2.  The ACH IPC library must be installed to your system - see instructions for Ubuntu/Debian install [here](http://www.golems.org/projects/ach.html).

3.  A copy of [hubo-ach](https://github.com/hubo/hubo-ach). It does not necessarily need to be *installed* to build these packages, but it is a runtime dependency.

4.  A copy of [hubo-motion-rt](https://github.com/hubo/hubo-motion-rt). Once again, it does not need to be installed to build, but it is a runtime dependency.

Build and usage instructions
----------------------------
First, you must update the symlinks in `hubo_components` to point to your copies.
```
$ roscd hubo_components/include/hubo_components
$ ln -s ../../relative/path/to/hubo/motion/include/* ./
$ ln -s ../../relative/path/to/hubo/ach/include/* ./
```
To build all packages in this repository:

```
(in the surrounding Catkin workspace directory)
$ catkin_make
```
To build a particular package in the repository:

```
(in the surrounding Catkin workspace directory)
$ catkin_make --pkg <package name>
```
To use, you must source the workspace:

```
(in the surrounding Catkin workspace directory)
$ source devel/setup.bash
```

For usage information and instructions on running components of these packages together, see the repository [Wiki](https://github.com/WPI-ARC/hubo_ros_core/wiki).
