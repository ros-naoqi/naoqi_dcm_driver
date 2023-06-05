NAOqi DCM Driver
================

The hardware interface to connect to Nao, Romeo, and Pepper robots. This is the new version of `nao_dcm_driver <https://github.com/ros-naoqi/nao_dcm_robot/tree/master/nao_dcm_driver>`_ that is now common for all robots.
You can control the robot either by calling DCM commands, or using ALMotion (by default).
When calling DCM commands, you can achieve faster control frequency but it can make the robot shaking because of the concurrence between DCM and ALMotion. Especially when using DCM, please be careful and use it at your own risks.

What it does
++++++++++++

The package allows to control a robot from ROS, while communicating with Naoqi.
The package is inspired by nao_dcm_driver, however it does not require NAOqi SDK anymore and rather based on naoqi_libqi and naoqi_libqicore. The package is more generic, and it works with any of Nao, Romeo, and Pepper robots. It can run on a remote PC or locally on a robot (after compiling on OpenNAO VM, deploying to your robot, and using robot_ip:=127.0.0.1).

Development
+++++++++++

This project must be part of a catkin workspace.
Do not forget to source the workspace:

```sh
cd <catkin_ws>
source devel/setup.bash
```

Dependencies
------------

To build and run from source, the driver requires the `naoqi_libqi`, `naoqi_libqicore` and `naoqi_bridge_msgs` packages.
For ROS Noetic:

```sh
apt install ros-noetic-naoqi-libqi ros-noetic-naoqi-libqicore ros-noetic-naoqi-bridge-msgs
```

Additionally, `pepper_meshes` and/or `nao_meshes` can be useful if you try to display the robot in RViz.

You need to clone the following reperitories under `src/`:
- pepper_robot: https://github.com/linoxmo/pepper_robot.git
- pepper_dcm_robot: https://github.com/linoxmo/pepper_dcm_robot.git
- pepper_virtual: https://github.com/linoxmo/pepper_virtual.git
- naoqi_bridge: https://github.com/linoxmo/naoqi_bridge.git

For ROS Noetic, you also need to clone the following repositories:
- humanoid_msgs: https://github.com/ahornung/humanoid_msgs.git
- camera_info_manager_py: https://github.com/ros-perception/camera_info_manager_py.git

```sh
rosdep install -y --from-paths src -i src/pepper_dcm_robot/pepper_dcm_bringup/
```

The packages `python-rospkg` and `python-yaml` cannot be installed on Ubuntu 20.04,
install `python3-rospkg` and `python3-yaml` instead.

Check the missing packages with:

```sh
rosdep check --from-paths src -i src/pepper_dcm_robot/pepper_dcm_bringup/
```

You might need to install further dependencies:

```sh
apt install ros-noetic-rgbd-launch
apt install ros-noetic-gazebo-plugins ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-ros-control ros-noetic-ros-controllers python-rospkg python-yaml
```

Build
-----

Use `catkin_make` normally to build the packages.

Launch
------

The driver can be launched using the following command for NAOqi 2.5:

```sh
roslaunch pepper_dcm_bringup pepper_dcm_bringup_position.launch robot_ip:=<ip>
```

> You can set a specific port with `robot_port:=<port>`. It is set to `9559` by default.

For NAOqi 2.9, you must provide the robot's credentials:

```sh
roslaunch pepper_dcm_bringup pepper_dcm_bringup_position.launch robot_ip:=<ip> user:=<user> password:=<password>
```

> You can set a specific port with `robot_port:=<port>`. It is set to `9503` by default
  if `user` or `password` are provided.
