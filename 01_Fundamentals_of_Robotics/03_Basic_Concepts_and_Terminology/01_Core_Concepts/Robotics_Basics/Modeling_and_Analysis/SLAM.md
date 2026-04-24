---
title: SLAM (Simultaneous Localization and Mapping)
description: "Defines SLAM: The process by which a mobile robot builds a map of an unknown environment while simultaneously estimating its own pose within that map."
tags:
  - localization
  - mapping
  - estimation
  - navigation
  - mobile-robot
  - probabilistic-robotics
layout: default
category: robotics
author: Jordan_Smith
date: 2025-05-02
permalink: /slam/
related:
  - "[[Sensors]]"
  - "[[Autonomous_Robots]]"
  - "[[Mobile_Robots]]"
  - "[[LIDAR]]"
  - "[[Probabilistic_Robotics]]"
---

# SLAM (Simultaneous Localization and Mapping)

**SLAM** stands for **Simultaneous Localization and Mapping**. It is the computational problem in robotics where a mobile robot or autonomous agent, operating in a previously unknown environment, incrementally builds a consistent map of that environment while simultaneously determining its own location (pose: position and orientation) relative to the map being built.

SLAM addresses a fundamental "chicken-and-egg" problem: to build an accurate map, the robot needs precise knowledge of its own location, but to accurately localize itself, the robot typically needs a reliable map. Solving SLAM enables robots to navigate autonomously in unfamiliar spaces without relying on external infrastructure like GPS or pre-existing maps.

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
  * **[[Kalman_Filter|EKF-SLAM]]**: Historically the first main approach. Uses an Extended Kalman Filter to represent the joint posterior probability distribution over the robot pose and all map features as a single large Gaussian. Captures correlations between robot pose and feature estimates. **Challenge**: Computational complexity scales quadratically with the number of map features, limiting scalability.
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

## EKF-SLAM: Detailed Formulation

EKF-SLAM maintains a joint state vector containing the robot pose and all landmark positions, with a full covariance matrix capturing the correlations between them.

### State Vector

For a 2D robot with $N$ point landmarks:

$$
\mathbf{x} = \begin{bmatrix} x_r \\ y_r \\ \theta_r \\ x_1 \\ y_1 \\ \vdots \\ x_N \\ y_N \end{bmatrix} \in \mathbb{R}^{3 + 2N}
$$

The covariance matrix $P \in \mathbb{R}^{(3+2N) \times (3+2N)}$ captures uncertainty in the robot pose and all landmark positions, plus their cross-correlations.

### Prediction Step

When the robot moves with control input $\mathbf{u}_t = [v_t, \omega_t]^T$ (linear and angular velocity):

**State prediction:**

$$
\hat{\mathbf{x}}_{t|t-1} = \begin{bmatrix} x_r + v_t \Delta t \cos(\theta_r + \omega_t \Delta t / 2) \\ y_r + v_t \Delta t \sin(\theta_r + \omega_t \Delta t / 2) \\ \theta_r + \omega_t \Delta t \\ x_1 \\ y_1 \\ \vdots \end{bmatrix}
$$

Note: landmarks do not move, so only the robot pose entries are updated.

**Covariance prediction:**

$$
P_{t|t-1} = F_t P_{t-1} F_t^T + G_t Q_t G_t^T
$$

where $F_t$ is the Jacobian of the motion model with respect to the state (identity for landmarks), $G_t$ maps noise to the state, and $Q_t$ is the process noise covariance.

### Update Step

When the robot observes landmark $j$ with measurement $\mathbf{z} = [r, \phi]^T$ (range and bearing):

**Innovation:**

$$
\mathbf{y} = \mathbf{z} - h(\hat{\mathbf{x}}_{t|t-1})
$$

where $h(\cdot)$ is the measurement model:

$$
h(\mathbf{x}) = \begin{bmatrix} \sqrt{(x_j - x_r)^2 + (y_j - y_r)^2} \\ \text{atan2}(y_j - y_r, \, x_j - x_r) - \theta_r \end{bmatrix}
$$

**Kalman gain:**

$$
K = P_{t|t-1} H^T (H P_{t|t-1} H^T + R)^{-1}
$$

where $H = \frac{\partial h}{\partial \mathbf{x}}$ is the measurement Jacobian and $R$ is the measurement noise covariance.

**State update:**

$$
\hat{\mathbf{x}}_t = \hat{\mathbf{x}}_{t|t-1} + K \mathbf{y}
$$

**Covariance update:**

$$
P_t = (I - K H) P_{t|t-1}
$$

### EKF-SLAM Pseudocode

```
EKF_SLAM(x, P, u, z):
    # --- Prediction ---
    x_robot = motion_model(x_robot, u)
    F = jacobian_motion(x, u)
    P = F * P * F^T + Q_augmented

    # --- Update (for each observation z_i) ---
    for each observed landmark j:
        if landmark j is new:
            initialize landmark position from (x_robot, z_j)
            augment x and P with new landmark

        y = z_j - measurement_model(x_robot, landmark_j)
        H = jacobian_measurement(x, j)
        S = H * P * H^T + R
        K = P * H^T * S^{-1}
        x = x + K * y
        P = (I - K * H) * P

    return x, P
```

---

## Computational Complexity Comparison

| Algorithm | Time per Step | Space | Strengths | Weaknesses |
|---|---|---|---|---|
| **EKF-SLAM** | $O(N^2)$ per observation ($N$ = landmarks) | $O(N^2)$ (dense covariance) | Full correlation, consistent estimates | Quadratic scaling limits to ~1000 landmarks; linearization errors |
| **FastSLAM (particle filter)** | $O(M \log N)$ ($M$ = particles) | $O(MN)$ | Handles non-Gaussian noise, multi-modal data association | Particle depletion in high dimensions; memory scales with particles |
| **Graph-Based SLAM** | $O(N^3)$ batch, but $O(N)$ per step with sparse solvers | $O(N)$ sparse | Scales to millions of poses; globally consistent | Primarily offline; requires good initial guess for optimization |
| **iSAM2 (incremental)** | $O(\text{affected variables})$ per step | $O(N)$ sparse | Online, efficient incremental updates | Implementation complexity |

**Practical note:** For modern applications, graph-based SLAM (via g2o, GTSAM, or Ceres) is the dominant approach. EKF-SLAM is mainly of pedagogical importance. FastSLAM remains useful for occupancy grid mapping (GMapping).

---

## Practical Sensor Recommendations

### LiDAR for SLAM

| Sensor | Type | Range | Points/sec | Price Range | Best For |
|---|---|---|---|---|---|
| RPLiDAR A1 | 2D, mechanical | 12 m | 8,000 | ~$100 | Low-cost indoor mobile robots |
| Hokuyo UST-10LX | 2D, scanning | 10 m | 40,000 | ~$1,600 | Research mobile robots |
| Velodyne VLP-16 (Puck) | 3D, 16 channels | 100 m | 300,000 | ~$4,000 | Autonomous vehicles, outdoor |
| Ouster OS1-64 | 3D, 64 channels | 120 m | 1,310,720 | ~$6,000 | High-res outdoor mapping |
| Livox Mid-360 | 3D, non-repetitive | 40 m | 200,000 | ~$750 | Cost-effective 3D SLAM |

**LiDAR SLAM software:** LOAM, LIO-SAM, FAST-LIO2 (for LiDAR-inertial), Cartographer (Google, 2D/3D).

### Cameras for Visual SLAM

| Sensor | Type | Resolution | FPS | Price Range | Best For |
|---|---|---|---|---|---|
| Intel RealSense D435i | Stereo RGB-D + IMU | 1280x720 (depth) | 30--90 | ~$300 | Indoor navigation, manipulation |
| Intel RealSense T265 (discontinued) | Stereo fisheye + IMU | 848x800 per eye | 30 | ~$200 | VIO/V-SLAM tracking |
| ZED 2i (Stereolabs) | Stereo RGB + IMU | 2208x1242 | 15--100 | ~$450 | Indoor/outdoor SLAM |
| OAK-D Pro (Luxonis) | Stereo + RGB + IMU | 1280x800 | 30--120 | ~$300 | Edge AI + SLAM |

**Visual SLAM software:** ORB-SLAM3 (feature-based, monocular/stereo/RGB-D + IMU), RTAB-Map (ROS 2 compatible, multi-session), VINS-Fusion (VIO), DSO (direct sparse).

**Practitioner tip:** For production mobile robots, LiDAR-inertial SLAM (e.g., FAST-LIO2 or LIO-SAM) offers the best reliability. Visual SLAM is suitable for cost-sensitive applications but struggles in textureless environments and changing lighting.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #localization OR #mapping WHERE contains(file.outlinks, [[SLAM]])
