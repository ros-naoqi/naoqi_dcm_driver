NAOqi DCM Driver
================

The hardware interface to connect to Nao, Romeo, and Pepper robots. This is the new version of `nao_dcm_driver <https://github.com/ros-naoqi/nao_dcm_robot/tree/master/nao_dcm_driver>`_ that is now common for all robots.
You can control the robot either by calling DCM commands, or using ALMotion (by default). 
When calling DCM commands, you can achieve faster control frequecy but it can make the robot shaking because of the concurrence between DCM and ALMotion. Especially when using DCM, please be careful and use it at your own risks.

What it does
============

The package communicates with NAOqi to enable a robot control with ROS through the following packages (please follow the links to get for following installation and launch instructions):

* `nao_dcm_bringup <http://wiki.ros.org/nao_dcm_bringup>`_

* `romeo_dcm_bringup <http://wiki.ros.org/romeo_dcm_bringup>`_

* `pepper_dcm_bringup <http://wiki.ros.org/pepper_dcm_bringup>`_

The package naoqi_dcm_driver is inspired by nao_dcm_driver, however it does not require NAOqi SDK anymore and rather based on naoqi_libqi and naoqi_libqicore.
The package is more generic and it works with any of Nao, Romeo, and Pepper robots. 
It can be runned on a remote PC or locally on a robot (after compiling on OpenNAO, and exporting NAO_IP=127.0.0.1 before launching a corresponding dcm_bringup package).
