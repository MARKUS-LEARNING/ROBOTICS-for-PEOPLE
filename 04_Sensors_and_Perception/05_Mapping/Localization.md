---
title: Localization
description: Localization in robotics refers to the process by which a robot determines its position and orientation within its environment.
tags:
  - robotics
  - navigation
  - sensors
  - control
  - engineering
type: Robotic Concept
application: Determining a robot's position and orientation
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-04-29
permalink: /localization/
related:
  - "[[Robot_Design]]"
  - "[[Autonomous_Navigation]]"
  - "[[SLAM]]"
  - "[[Sensor_Fusion]]"
  - "[[Odometry]]"
  - "[[GPS]]"
  - "[[LIDAR]]"
  - "[[Kalman_Filter]]"
  - "[[Path_Planning]]"
---

# Localization

**Localization** in robotics refers to the process by which a robot determines its position and orientation within its environment. It is a fundamental aspect of autonomous navigation, enabling robots to understand their spatial relationship with the surroundings and plan their movements accordingly. Localization involves the use of various sensors and algorithms to estimate the robot's pose (position and orientation) accurately and reliably.

---

## Key Concepts in Localization

1. **SLAM (Simultaneous Localization and Mapping)**: A technique that allows a robot to build a map of its environment while simultaneously determining its location within that map. SLAM is crucial for autonomous navigation in unknown or dynamic environments.

2. **Sensor Fusion**: The integration of data from multiple sensors to improve the accuracy and reliability of localization. Sensor fusion combines information from various sources, such as lidar, cameras, and GPS, to provide a more robust estimate of the robot's pose.

3. **Odometry**: The use of motion sensors, such as wheel encoders or inertial measurement units (IMUs), to estimate the robot's position and orientation based on its movement over time. Odometry is essential for tracking the robot's pose but can accumulate errors over long distances.

4. **Landmarks**: Distinctive features in the environment that can be used as reference points for localization. Landmarks help robots to correct errors in their pose estimates by providing known locations that can be detected and matched.

5. **Kalman Filter**: A recursive filter that estimates the state of a dynamic system from a series of incomplete and noisy measurements. The Kalman filter is widely used in localization to fuse data from various sensors and provide an optimal estimate of the robot's pose.

---

## Key Equations

**Odometry Model**:

$$
\begin{bmatrix}
x_{t} \\
y_{t} \\
\theta_{t}
\end{bmatrix}
=**
\begin{bmatrix}
x_{t-1} \\
y_{t-1} \\
\theta_{t-1}
\end{bmatrix}
+
\begin{bmatrix}
\Delta s \cdot \cos(\theta_{t-1} + \Delta \theta) \\
\Delta s \cdot \sin(\theta_{t-1} + \Delta \theta) \\
\Delta \theta
\end{bmatrix}
$$

  where $(x_t, y_t, \theta_t)$ is the robot's pose at time $t$, $\Delta s$ is the distance traveled, and $\Delta$ $\theta$ is the change in orientation. This equation describes the odometry model for updating the robot's pose based on its movement.

**Kalman Filter Equations**

<img width="750" alt="image" src="https://github.com/user-attachments/assets/272ca099-1a5d-4114-8f11-0584f26731a6" />


**Notation:**
- $\hat{\mathbf{x}}$: State estimate
- $\mathbf{P}$: State covariance
- $\mathbf{F}$: State transition matrix
- $\mathbf{B}$: Control input matrix
- $\mathbf{u}$: Control vector
- $\mathbf{Q}$: Process noise covariance
- $\mathbf{H}$: Observation matrix
- $\mathbf{R}$: Measurement noise covariance
- $\mathbf{K}$: Kalman gain
- $\mathbf{y}$: Innovation (measurement residual)
- $\mathbf{z}$: Measurement vector

**SLAM Measurement Model**:

$$
z_t = h(x_t) + v_t
$$

  where $z_t$ is the measurement at time $t$, $h(x_t)$ is the measurement function that relates the state $x_t$ to the measurement, and $v_t$ is the measurement noise. This equation is used in SLAM to relate the robot's state to the sensor measurements.
  <br></br>

---

## Impact on Robotics

- **Autonomous Navigation**: Localization is essential for autonomous navigation, enabling robots to determine their position and orientation within the environment. This allows them to plan and execute paths safely and efficiently.

- **Environmental Mapping**: Accurate localization is crucial for building and updating maps of the environment, which are necessary for navigation and interaction with the surroundings.

- **Collision Avoidance**: Knowing the robot's precise location helps in avoiding collisions with obstacles and other entities in the environment, ensuring safe operation.

- **Task Execution**: Localization enables robots to perform tasks accurately by providing the necessary spatial information to interact with objects and navigate through the environment.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts
```dataview
LIST FROM #robotics OR #navigation WHERE contains(file.outlinks, [[Localization]])
