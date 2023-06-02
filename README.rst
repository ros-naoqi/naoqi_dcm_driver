NAOqi DCM Driver
================

The hardware interface to connect to Nao, Romeo, and Pepper robots. This is the new version of `nao_dcm_driver <https://github.com/ros-naoqi/nao_dcm_robot/tree/master/nao_dcm_driver>`_ that is now common for all robots.
You can control the robot either by calling DCM commands, or using ALMotion (by default). 
When calling DCM commands, you can achieve faster control frequency but it can make the robot shaking because of the concurrence between DCM and ALMotion. Especially when using DCM, please be careful and use it at your own risks.

What it does
============

The package allows to control a robot from ROS, while communicating with Naoqi. 
The package is inspired by nao_dcm_driver, however it does not require NAOqi SDK anymore and rather based on naoqi_libqi and naoqi_libqicore. The package is more generic, and it works with any of Nao, Romeo, and Pepper robots. It can run on a remote PC or locally on a robot (after compiling on OpenNAO VM, deploying to your robot, and using robot_ip:=127.0.0.1).

How to use
==========

## Dependencies
To run, the driver requires the `naoqi_libqi`, `naoqi_libqicore` and `naoqi_bridge_msgs` packages. Those can be installed using apt-get (if they have been released for your ROS distro) or from source. Additionally, `pepper_meshes` and/or `nao_meshes` can be useful if you try to display the robot in RViz.

Additionaly, you might need to install further dependencies with this command : apt install ros-noetic-gazebo-plugins ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-ros-control ros-noetic-ros-controllers python-rospkg python-yaml

## Reperitory 
You need to clone the following reperitorys: pepper_robot,pepper_dcm_robot,pepper_virtual,naoqi_bridge,humanoid_msgs,camera_info_manager_py  and build them in the same workspace. 
## Launch
The driver can be launched using the following command:
```sh
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=<ip> nao_port:=<port> network_interface:=<interface> username:=<name> password:=<passwd>

'''
source path/setup.bash 

roslaunch pepper_dcm_bringup pepper_dcm_bringup_position.launch robot_ip:=<ip> robot_port:=<port> network_interface:=<interface> username:=<user> password:=<password>
'''
       

```
Note that the username and password arguments are only required for robots running naoqi 2.9 or greater 

:warning: naoqi_driver for melodic and greater have to be used for robots running naoqi 2.9 and greater

## Build status

ROS Distro| Binary Status | Source Status | Github Build |
|-------------------|-------------------|-------------------|-------------------|
Noetic | [![Build Status](https://build.ros.org/job/Nbin_uf64__naoqi_driver__ubuntu_focal_amd64__binary/badge/icon)](https://build.ros.org/job/Nbin_uf64__naoqi_driver__ubuntu_focal_amd64__binary/) | [![Build Status](https://build.ros.org/job/Nsrc_uF__naoqi_driver__ubuntu_focal__source/badge/icon)](https://build.ros.org/job/Nsrc_uF__naoqi_driver__ubuntu_focal__source/) | [![ros-noetic-focal](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/noetic_focal.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/noetic_focal.yml)
Melodic | [![Build Status](https://build.ros.org/job/Mbin_ub64__naoqi_driver__ubuntu_bionic_amd64__binary/badge/icon)](https://build.ros.org/job/Mbin_ub64__naoqi_driver__ubuntu_bionic_amd64__binary/) | [![Build Status](https://build.ros.org/job/Msrc_uB__naoqi_driver__ubuntu_bionic__source/badge/icon)](https://build.ros.org/job/Msrc_uB__naoqi_driver__ubuntu_bionic__source/) | [![ros-melodic-bionic](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/melodic_bionic.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/melodic_bionic.yml)
Kinetic | ![passing](https://raw.githubusercontent.com/jenkinsci/embeddable-build-status-plugin/7c7eedc7617851f07a1f09629c33fee11cff50ab/src/doc/flat_unconfigured.svg) | ![passing](https://raw.githubusercontent.com/jenkinsci/embeddable-build-status-plugin/7c7eedc7617851f07a1f09629c33fee11cff50ab/src/doc/flat_unconfigured.svg) | [![ros-kinetic-xenial](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/kinetic_xenial.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/kinetic_xenial.yml) |


_


