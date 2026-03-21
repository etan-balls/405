.. 405 Project documentation master file, created by
   sphinx-quickstart on Fri Mar 20 13:49:28 2026.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

405 Project Documentation
=========================

Welcome to the documentation for the 405 autonomous robot project.

This site provides an overview of the system architecture, hardware interfaces,
control strategy, and module-level implementation details for the embedded
robotics software.

Project Summary
---------------

This project implements an autonomous mobile robot capable of:

- Following a black line using an IR reflectance sensor array
- Measuring wheel motion with quadrature encoders
- Estimating heading and motion using IMU and observer-based estimation
- Controlling motors through PWM-based differential drive
- Running multiple real-time subsystems using a cooperative task scheduler

Documentation Contents
----------------------

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   overview
   architecture
   hardware
   software
   control

API Reference
-------------

.. toctree::
   :maxdepth: 2
   :caption: Module Reference:

   modules/motor
   modules/sensors
   modules/control
   modules/estimator
   modules/tasks