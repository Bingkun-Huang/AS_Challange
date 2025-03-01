# controller_pkg

This package implements a **geometric tracking controller** based on the paper:

> **[1] Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch.**  
> "Geometric tracking control of a quadrotor UAV on SE(3)."  
> *Decision and Control (CDC), 49th IEEE Conference on.* IEEE, 2010.

It uses **Eigen** for linear algebra, **tf** for transformations, and subscribes to trajectory plus odometry data to compute appropriate rotor speed commands for a quadrotor UAV.  

---

## Overview

- **Implements position and attitude control** for a simulated quadrotor using a geometric approach.  
- **Computes desired orientation** from the reference acceleration and yaw angle.  
- **Publishes propeller speeds** which can be used in a simulation (e.g., Unity).

---

## Features

- **Geometric Controller**: Follows the orientation error approach from [1].  
- **Velocity-Based Yaw**: Yaw is derived from the velocity direction when speed exceeds a threshold.  

---

## Topics

### Subscribed Topics
1. **`command/trajectory`**  
   - **Message Type**: `trajectory_msgs/MultiDOFJointTrajectory`  
   - **Usage**: Receives the desired position (`xd`), velocity (`vd`), and acceleration (`ad`) for the UAV.  
2. **`current_state_est`**  
   - **Message Type**: `nav_msgs/Odometry`  
   - **Usage**: Provides current UAV state (position, velocity, orientation) used for the control loop.

### Published Topics
1. **`rotor_speed_cmds`**  
   - **Message Type**: `mav_msgs/Actuators`  
   - **Usage**: Outputs the **computed propeller speeds**.