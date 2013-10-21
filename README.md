hubo_ros_core
=============

Core ROS support for the Hubo humanoid.

This includes message/action definitions, the basic interface to hubo-ach, and the suite of launch files used to start ROS on the DRCHubo robot.

*While this code is freely licensed (2-clause BSD), we do ask that you send [us](mailto:calder.pg@gmail.com) an email so we can see who is using this software.*

Repository structure
--------------------
Unlike earlier Catkinized software we have provided, this repository does not contain a Catkin workspace. As we expect that other teams will be well on their way to migrating to ROS Groovy, the difficulties of managing multiple workspaces do not justify the convenience of distributing these packages in their own workspace. As such, you will need to clone this repository inside the `src/` directory of an existing Catkin workspace.

Please note that this software is structured for ROS Groovy+, and is incompatible with ROS Fuerte and earlier. Versions of this software for earlier versions of ROS will not be actively developed or maintained, but historical versions may be available upon request.

This repository is structured around 6 core packages:

1.  `hubo_robot_msgs` - Message and action definitions that cover the Hubo robot. Everything from joint states to actions for walking and trajectory execution. Note that several of these message and action definitions are based directly off/duplicate equivalent functionality for the PR2. This is to avoid any dependency on the PR2 software stack.

2.  `hubo_sensor_msgs` - Message and service definitions for sensors on the Hubo robot. This is contains service and action definitions for working with the DRCHubo LIDAR scanner.

3.  `hubo_system_msgs` - Message and service definitions for the system software and configuration of the Hubo robot. This is currently empty and awaiting proposed message/service types.

4.  `hubo_ach_ros_bridge` - Very basic joint-level control and state publishing. In addition, this package provides ROS clocking and robot model support for RVIZ.

5.  `hubo_launch` - Comprehensive suite of launch files for starting ROS components on the body computer, backpack computer, and user workstation. This package contains a custom version of `openni2_launch` modified to support "divorced" TF trees. Additionally, this package contains YAML configuration parameters for Hubo+ and DRCHubo.

6.  `hubo_description` - Wrapper around Xacro and URDF model files for the HuboPlus robot and the dae meshes for the URDF model.

Stability and development status
--------------------------------
`hubo_robot_msgs` has been stable for several months. It should be considered stable for all uses. We do not intend to change any message/action interfaces provided unless required to do so by a critical bug, and if/when we do add additional functionality, compatibility with all original functionality will be maintained.

`hubo_sensor_msgs` is currently empty and awaiting Hubo-specific sensor message definitions. If there is a sensor aboard the Hubo for which no existing ROS message type matches, please propose an addition to this package, and it will be added. Please note that the IMU/tilt sensors are already supported by `sensor_msgs/Imu` (enter NAN if a particular value *inside* a field [i.e. Az or Wx] is not available from the sensor), and the force sensors are supported by `geometry_msgs/WrenchStamped` (once again, use NAN for elements *inside* a field not provided by the sensor [i.e. f_x or t_z]).

`hubo_system_msgs` is currently empty and awaiting Hubo-specific system control messages/actions/services. In particular, this package is intended to provide messages for system (computers,software, etc) status and configuration and services for their reconfiguration. If there is a system control or status you want represented in an ROS message, propose it and it will be added to this package.

`hubo_ach_ros_bridge` is under semi-active development. The feedback half of the bridge has been tested with both Hubo+ and DRC-Hubo in both simulated and real environments and should be considered stable. It is part of the standard software configuration of the robot, allowing standard RVIZ display and TF support, along with the ability to use standard ROS logging tools. This package is updated as necessary to maintain compatibility with hubo-ach. This package has been updated to provide compatibility with DRC-Hubo and the old joint name-to-hubo-ach indexing has been replaced with a completely parametrized approach that supports both Hubo+ and DRC-Hubo.

`hubo_launch` is under semi-active development, with new launch files added as needed to support new hardware on the robot and existing files modified to change behavior. For most purposed, `hubo_launch` should be considered stable.

Depencies
---------
1.  Full ROS Groovy installation - on Ubuntu systems: `$ sudo apt-get install ros-groovy-desktop-full`

2.  ROS package `dynamixel_motor`, which **must** be built from [source](https://github.com/calderpg/dynamixel_motor) - note we are in the process of moving this dependency to a different package.

Build and usage instructions
----------------------------
First, clone this repository:
```
$ cd /your/catkin/workspace/src
$ git clone https://github.com/WPI-ARC/hubo_ros_core.git
$ rospack profile
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
