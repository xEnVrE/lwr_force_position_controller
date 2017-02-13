# Hybrid Force Position Controllers for Kuka LWR

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
- [__vito-robot__](https://github.com/CentroEPiaggio/vito-robot): (vito_description only) required for vito robot's torso description and materials.

In order to compile the GUI you also need ros-*-qt-build.

## How to run a simulation

WARNING: in order to run the simulation you MUST set every \<damping\> and \<friction\> tag in [kuka_lwr.urdf.xacro](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_description/model/kuka_lwr.urdf.xacro)
to 0.0 and disable the \<collision\> section of the 7-th link.

1. roslaunch lwr_force_position_controllers single_lwr.launch
