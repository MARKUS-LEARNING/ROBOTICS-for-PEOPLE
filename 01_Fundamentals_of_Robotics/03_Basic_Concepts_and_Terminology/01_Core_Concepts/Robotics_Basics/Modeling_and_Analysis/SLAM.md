---
title: SLAM (Simultaneous Localization and Mapping)
description: "Defines SLAM: The process by which a mobile robot builds a map of an unknown environment while simultaneously estimating its own pose within that map."
tags:
  - glossary-term
  - localization
  - mapping
  - estimation
  - navigation
  - mobile-robot
  - probabilistic-robotics
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /slam/
related:
  - "[[Localization]]"
  - "[[Mapping]]"
  - "[[Perception]]"
  - "[[Sensor_Fusion]]"
  - "[[Kalman_Filter]]"
  - "[[Particle_Filter]]"
  - "[[Graph Optimization]]"
  - "[[LIDAR]]"
  - "[[Camera_Systems]]"
  - "[[IMU_Sensors]]"
  - "[[Odometry]]"
  - "[[Loop_Closure]]"
  - "[[Data_Association]]"
  - "[[State_Estimation]]"
---

# SLAM (Simultaneous Localization and Mapping)

**SLAM** stands for **Simultaneous Localization and Mapping**. It is the computational problem in robotics where a mobile robot or autonomous agent, operating in a previously unknown environment, incrementally builds a consistent map of that environment while simultaneously determining its own location (pose: position and orientation) relative to the map being built.

SLAM addresses a fundamental "chicken-and-egg" problem: to build an accurate map, the robot needs precise knowledge of its own location, but to accurately localize itself, the robot typically needs a reliable map. Solving SLAM enables robots to navigate autonomously in unfamiliar spaces without relying on external infrastructure like GPS or pre-existing maps.

---
![image](https://github.com/user-attachments/assets/a3074408-2063-47d1-91b7-9ec8b1ea3c03)

<font size=1>*source: https://www.linkedin.com/pulse/beginners-guide-slam-robotics-vedant-nair-peeac/*</font>
---

## Purpose and Motivation

* **Autonomy in Unknown Environments**: Enables robots to operate where maps are unavailable, inaccurate, or dynamic.
* **Map Creation**: Generates spatial models of environments for navigation, interaction, or analysis.
* **Robust Localization**: Provides a way to maintain accurate pose estimates even when external localization systems (like GPS) fail or are unavailable (e.g., indoors, underwater, dense forests).

---

## Key Components and Challenges

SLAM involves several intertwined components and faces significant challenges:

* **Mapping**: Representing the environment. Common representations include feature-based maps (landmarks, lines, planes), occupancy grids, topological graphs, dense point clouds, surface meshes, and semantic maps.
  <br>

* **Localization**: Estimating the robot's pose relative to the map. This involves integrating motion estimates (from [[Odometry]] or [[IMU_Sensors]]) with observations of the environment.
  <br>

* **Data Association**: Correctly matching current sensor observations to previously mapped elements or landmarks. Incorrect associations can lead to catastrophic map corruption.
  <br>

* **Loop Closure**: Detecting when the robot revisits a previously mapped area. Correct loop closure allows for the correction of accumulated odometry drift, ensuring global map consistency. This is critical for long-term mapping accuracy.
  <br>

* **Computational Complexity & Scalability**: Managing the computational cost as the map size and trajectory length grow. Efficient algorithms and map representations are crucial for real-time performance and large-scale environments.
  <br>

* **Sensor Modeling**: Accurately representing sensor noise, biases, and detection characteristics.
  <br>

* **Dynamic Environments**: Dealing with moving objects (people, vehicles) or changes in the environment structure that are not part of the static map assumption common to many SLAM algorithms.
  <br>

---

## Major Approaches

Modern SLAM algorithms are predominantly probabilistic, explicitly modeling uncertainty in both the robot's pose and the map. The main paradigms include:

* **Filter-Based SLAM (Primarily for Online SLAM)**: These methods incrementally update the estimate of the current robot pose and the map as new sensor data arrives.
  * **[[Kalman Filter|EKF-SLAM]]**: Historically the first main approach. Uses an Extended Kalman Filter to represent the joint posterior probability distribution over the robot pose and all map features as a single large Gaussian. Captures correlations between robot pose and feature estimates. **Challenge**: Computational complexity scales quadratically with the number of map features, limiting scalability.
    <br>

  * **[[Particle Filter|Particle Filter SLAM (e.g., FastSLAM)]]**: Represents the posterior distribution using a set of weighted samples (particles). Typically employs Rao-Blackwellization: each particle represents a hypothesized robot trajectory, and maintains its own map estimate conditioned on that trajectory. This exploits the conditional independence of map features given the path. **Advantages**: Scales better than EKF-SLAM (often logarithmic in map size per particle), handles non-Gaussian uncertainties and data association problems more naturally. Grid-based FastSLAM is a popular variant using occupancy grids for maps. **Challenge**: Number of particles needed can become large, especially in environments with many loops or perceptual aliasing.
    <br>

* **Graph-Based SLAM (Primarily for Full SLAM)**: These methods formulate SLAM as a graph optimization problem.
  * **Representation**: Nodes in the graph typically represent robot poses at different times and/or landmark locations. Edges represent spatial constraints derived from robot motion (odometry) or sensor measurements (observations of landmarks or loop closures).
    <br>

  * **Optimization**: Solves for the configuration of nodes (poses and landmark locations) that best satisfies all constraints simultaneously, often using nonlinear least-squares optimization (e.g., Gauss-Newton, Levenberg-Marquardt). This finds the Maximum Likelihood or Maximum a Posteriori estimate for the entire map and trajectory.
    <br>

  * **Advantages**: Highly scalable due to sparse optimization techniques. The method of choice for producing globally consistent maps offline, especially for large environments.
    <br>

  * **Challenge**: Primarily batch (offline) methods, though online variants (e.g., using information filters or incremental optimization) exist. Data association is often handled separately or integrated within the optimization.
    <br>

---

## Sensor Modalities and Variants

SLAM algorithms are often characterized by the primary sensors used:

* **[[LIDAR]] SLAM**: Uses LiDAR sensors to build precise geometric maps, often relying on scan matching techniques like ICP. Frequently fused with [[IMU_Sensors]] (LIO/LINS).
  <br>

* **Visual SLAM (V-SLAM)**: Uses [[Camera_Systems]] (monocular, stereo, RGB-D) as primary sensors. Can be feature-based (e.g., ORB-SLAM), direct (using pixel intensities), or semi-dense. Often fused with [[IMU_Sensors]] (VIO - Visual-Inertial Odometry/SLAM) for robustness.
  <br>

* **Multi-Sensor Fusion SLAM**: Combines data from multiple sensor types (e.g., LiDAR-Visual-Inertial) via [[Sensor_Fusion]] techniques to leverage the strengths of each modality, improving robustness and accuracy, especially in challenging environments (e.g., textureless areas for vision, open spaces for LiDAR).
  <br>

---

## Advanced Topics

* **Semantic SLAM**: Integrates object recognition and understanding into the SLAM process, creating maps annotated with semantic labels (e.g., "door", "chair", "tree").
  <br>

* **Dense SLAM**: Aims to reconstruct detailed, dense 3D models (e.g., point clouds, meshes, Neural Radiance Fields - NeRFs) of the environment, rather than sparse feature maps.
  <br>

* **Multi-Robot SLAM**: Teams of robots collaboratively build a map, requiring solutions for communication, data sharing, and relative localization.
  <br>

* **Active SLAM**: The robot intelligently plans its movements to actively reduce uncertainty in its map or localization estimate.
  <br>

* **Long-Term SLAM**: Addresses challenges of maintaining consistent maps over long periods, potentially involving changes in the environment.
  <br>

SLAM is a cornerstone technology for enabling mobile robot autonomy in unknown spaces and remains an active and challenging area of research.

---

## Mathematical Representations

### Pose Estimation

The robot's pose can be represented as a vector $x = [x, y, \theta]^T$ in 2D or $x = [x, y, z, \phi, \theta, \psi]^T$ in 3D, where $(x, y, z)$ denotes the position and $(\phi, \theta, \psi)$ denotes the orientation.

### Mapping

The map can be represented as a set of landmarks $L = \{l_1, l_2, \ldots, l_n\}$, where each landmark $l_i$ has a position in the environment.

### Measurement Model

The measurement model relates the observed data $z_t$ at time $t$ to the robot's pose $x_t$ and the map $L$:

$$
z_t = h(x_t, L) + v_t
$$

where $h(\cdot)$ is the measurement function and $v_t$ is the measurement noise.

### State Estimation

The state estimation problem in SLAM can be formulated as finding the posterior distribution over the robot's pose and the map given the observations:

$$
p(x_t, L | z_{1:t})
$$

This posterior distribution can be approximated using various techniques, such as the Kalman Filter, Particle Filter, or graph-based optimization.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #localization OR #mapping WHERE contains(file.outlinks, [[SLAM]])
