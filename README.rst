NAOqi DCM Driver
================

The hardware interface to connect to Nao, Romeo, and Pepper robots. 

What it does
============

The package communicates with NAOqi to enable a robot control through the following packages (please follow links to get for following installation and launch instructions):

* `nao_dcm_bringup <http://wiki.ros.org/nao_dcm_bringup>`_

* `romeo_dcm_bringup <http://wiki.ros.org/romeo_dcm_bringup>`_

* `pepper_dcm_bringup <http://wiki.ros.org/pepper_dcm_bringup>`_

Tha package is inspired by nao_dcm_driver, however it does not require NAOqi SDK anymore and rather based on naoqi_libqi and naoqi_libqicore.
The package is independent on a robot (Nao, Romeo, or Pepper). It can be runned on a remote PC or locally on a robot (you need to compile on OpenNAO, and export NAO_IP=127.0.0.1 before launching a corresponding dcm_bringup package).
