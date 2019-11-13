# Hybrid Force Position Controllers for Kuka LWR

[![Hybrid Impedance Control (Kuka LWR 4+)](https://img.youtube.com/vi/0tVq7SOc8s8/0.jpg)](https://www.youtube.com/watch?v=0tVq7SOc8s8)

Authors: [Nicola Piga](https://github.com/xenvre) and [Giulio Romualdi](https://github.com/giulioromualdi)

## Overview
This package contains three controllers:
- [__cartesian_position_controller__](src/cartesian_position_controller.cpp): an operational space controller required to
move the end effector in a desired position using inverse kinematics;
- [__cartesian_inverse_dynamics_controller__](src/cartesian_inverse_dynamics_controller.cpp): an operational space controller
performing dynamics inversion (also wrenches at the end effector are compensated);
- [__hybrid_impedance_controller__](src/hybrid_impedance_controller.cpp): an operational space controller implementing the
hybrid impedance controller proposed by [Spong](http://ieeexplore.ieee.org/document/20440/) (it uses the
cartesian_inverse_dynamics_controller to perform dynamics inversion);
- [__ft_sensor_controller__](src/ft_sensor_controller.cpp): a JointStateInterface controller performing transformations
on the force/torque measurements provided by the sensor.

## External packages required
In order to run some external packages are required:
- [__controller_switcher__](https://github.com/xEnVrE/controller_switcher): GUI required to switch controllers and set commands;
- [__kdljacdot__](https://github.com/xEnVrE/kdljacdot): Jacobian derivative solver from KDL 1.4 (not available in ros up to now);
- [__kuka-lwr__](https://github.com/CentroEPiaggio/kuka-lwr): description of the Kuka LWR manipulator;
- [__vito-robot__](https://github.com/CentroEPiaggio/vito-robot): (vito_description only) required for vito robot's torso description and materials;
- [__ft-calib__](https://github.com/xEnVrE/ft_calib): FTCalib class from https://github.com/kth-ros-pkg/force_torque_tools exported as ROS library;
- [__gravity-compensation__](https://github.com/xEnVrE/force_torque_tools/tree/indigo/gravity_compensation): gravity-compensation node;
- [__qb-interface-node__](https://github.com/xEnVrE/qb_interface_node/tree/imu): qb_interface_node with support for IMU and compatible with the gravity-compensation node.

In order to compile the GUI you also need ros-*-qt-build.

## How to run a simulation

WARNING: in order to run the simulation you MUST set every \<damping\> and \<friction\> tag in [kuka_lwr.urdf.xacro](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_description/model/kuka_lwr.urdf.xacro)
to 0.0 and disable the \<collision\> section of the 7-th link.

1. roslaunch lwr_force_position_controllers single_lwr.launch

## How to run using the real robot

1. roslaunch lwr_force_position_controllers single_lwr.launch use_lwr_sim:=false lwr_powered:=true
