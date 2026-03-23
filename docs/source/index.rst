.. raw:: html

   <div class="hero-banner">
     <h1>Romi Autonomous Robot</h1>
     <p class="subtitle">
       ME405 Term Project — Cal Poly SLO, Spring 2026<br>
       Fredy Herrarte &amp; Ethan Liu
     </p>
     <span class="hero-badge">MicroPython</span>
     <span class="hero-badge">STM32 Nucleo-L476RG</span>
     <span class="hero-badge">Cooperative Multitasking</span>
     <span class="hero-badge">Luenberger Observer</span>
     <span class="hero-badge">Line Following</span>
   </div>

Overview
========

This project implements a fully autonomous mobile robot on the **Pololu Romi** chassis.
The software runs on a bare-metal STM32 microcontroller using MicroPython and a
hand-written cooperative scheduler — no RTOS required.

The robot can:

- Follow a black line using a 7-element IR reflectance array and weighted-centroid steering
- Stabilize heading with a BNO055 IMU (yaw-rate damping + heading hold)
- Estimate its full state (position, heading, wheel speeds) using a discretized Luenberger observer
- Detect obstacles with six bump sensors and execute an autonomous avoidance maneuver
- Navigate a complete obstacle course autonomously via a 14-state finite state machine
- Stream live telemetry over Bluetooth to a PuTTY terminal at 115,200 baud

.. raw:: html

   <div class="stats-grid">
     <div class="stat-card">
       <span class="stat-num">7</span>
       <span class="stat-label">Concurrent Tasks</span>
     </div>
     <div class="stat-card">
       <span class="stat-num">50 Hz</span>
       <span class="stat-label">Control Rate</span>
     </div>
     <div class="stat-card">
       <span class="stat-num">4×4</span>
       <span class="stat-label">Observer Matrix</span>
     </div>
     <div class="stat-card">
       <span class="stat-num">14</span>
       <span class="stat-label">FSM States</span>
     </div>
     <div class="stat-card">
       <span class="stat-num">18+</span>
       <span class="stat-label">Python Modules</span>
     </div>
   </div>

Quick Start
-----------

1. Flash all ``.py`` files to the Nucleo's filesystem via **Thonny** or ``rshell``.
2. Connect a PuTTY terminal to the HC-05 Bluetooth COM port at **115,200 baud**.
3. Press the blue user button (PC13) on the Nucleo, or send ``o`` over Bluetooth, to start the obstacle course.
4. Send ``f`` to run a timed line-follow tuning run. Send any other key to stop.

.. tip::
   Run ``bt_config.py`` once (with nothing else connected) to set the HC-05 name,
   password, and baud rate before first use.

----

.. toctree::
   :maxdepth: 2
   :caption: Documentation

   architecture
   hardware
   software/index
   observer
   control
   navigation
   api/index
