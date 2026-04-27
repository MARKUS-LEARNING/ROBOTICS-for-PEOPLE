---
title: Localization
description: The process of estimating a robot's pose (position and orientation) within a known map or environment. Three flavors — pose tracking, global localization, and the kidnapped-robot problem — solved by a recurring family of probabilistic algorithms (EKF, MCL, AMCL, scan matching, factor graphs).
tags:
  - robotics
  - localization
  - state-estimation
  - probabilistic-robotics
  - mobile-robot
  - mapping
layout: default
category: robotics
author: Jordan_Smith
date: 2025-05-02
permalink: /localization/
related:
  - "[[Mapping]]"
  - "[[SLAM]]"
  - "[[Sensors]]"
  - "[[Perception]]"
  - "[[Sensor_Fusion]]"
  - "[[Kalman_Filter]]"
  - "[[GPS]]"
  - "[[LIDAR]]"
  - "[[IMU_Sensors]]"
  - "[[Particle_Filter]]"
---

# Localization

**Localization** is the process by which a robot determines its **pose** — position and orientation — within an environment. Given a map of the world and a stream of sensor measurements, the robot estimates "where am I, and which way am I facing, in the map's coordinate frame?" The answer is a probability distribution over poses, summarized at any instant by a single best estimate plus a measure of uncertainty.

> **Etymology.** From Latin *locus*, "place," via Late Latin *localis*, "of a place." The verb *to localize* originally meant "to assign to a place" (16th-century medical usage: localizing a disease). Robotics uses it in its modern sense — estimating the place — first standardized in the 1980s mobile-robot literature. Note the contrast with **localization** in the software-internationalization sense (translating to a locale): same Latin root, different abstraction. In robotics, "to localize the robot" means to *find* its place, not to translate it.

---

## Three problems hiding under one word

The robotics literature distinguishes three localization problems by how much the robot already knows about its pose.

### 1. Pose tracking (local localization)

**Initial pose is approximately known**, and the robot needs to keep track of it as it moves.

The error in pose tracking is local — the robot only has to maintain a tight Gaussian around the current pose, so this is the regime where the **Extended Kalman filter** shines. Almost every industrial AGV, drone, and self-driving car at steady state is doing pose tracking.

### 2. Global localization (kidnapped-robot problem)

**Initial pose is unknown.** The robot wakes up somewhere on a known map and has to figure out where. The posterior is *multi-modal* — the robot's first sensor reading may be consistent with many places on the map.

A unimodal Gaussian (Kalman filter) cannot represent this. **Particle filters** — Monte Carlo Localization (MCL), then Adaptive MCL (AMCL) — handle global localization by representing the posterior with thousands of weighted samples. As the robot moves and gets more readings, the inconsistent particles die off and the population converges to the true pose.

### 3. Kidnapping recovery (re-localization)

The robot is *tracking* successfully when someone picks it up and moves it ("kidnapped"). The pose-tracking filter is now wrong but confident. A robust localization system must detect that **the measurement innovations have become persistently large** and re-trigger global localization. AMCL handles this by injecting random particles whenever measurement likelihood drops.

---

## The probabilistic framework

Localization is a special case of Bayesian filtering on the state $\mathbf{x} = (\text{pose})$ given a sequence of observations $\mathbf{z}_{1:t}$, control inputs $\mathbf{u}_{1:t}$, and a known map $\mathbf{m}$:

$$
p(\mathbf{x}_t \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t}, \mathbf{m})
$$

The recursive update is the standard Bayes filter:

$$
\underbrace{\text{bel}(\mathbf{x}_t)}_{\text{posterior}} = \eta \, p(\mathbf{z}_t \mid \mathbf{x}_t, \mathbf{m}) \int p(\mathbf{x}_t \mid \mathbf{x}_{t-1}, \mathbf{u}_t) \, \text{bel}(\mathbf{x}_{t-1}) \, d\mathbf{x}_{t-1}
$$

with three pieces:

- **Motion model** $p(\mathbf{x}_t \mid \mathbf{x}_{t-1}, \mathbf{u}_t)$ — predicts the new pose from the old plus controls (typically wheel odometry or IMU).
- **Measurement model** $p(\mathbf{z}_t \mid \mathbf{x}_t, \mathbf{m})$ — what would the sensors read at this candidate pose given the map?
- **Map** $\mathbf{m}$ — the prior representation of the environment (occupancy grid, point cloud, feature map).

Each algorithm is a different way to *represent* and *update* this posterior:

- **Markov localization** — discretized grid over poses. Brute-force, expensive, complete.
- **Kalman / EKF localization** — single Gaussian over poses. Efficient, unimodal.
- **Monte Carlo localization (MCL)** — particle set. Multi-modal, sample-based.
- **Scan-matching localization** — frame-to-frame ICP/NDT against the map. Drift-free given good map and map-matching.
- **Factor-graph localization** — sliding-window optimization over recent poses + landmarks. State of the art for VIO/SLAM-based localization.

---

## The classic methods, in detail

### Dead reckoning

Use only proprioceptive sensors (wheel odometry, IMU integration). No external reference. Simple, but error grows unboundedly with distance traveled. A Roomba can dead-reckon for a few minutes; a delivery robot cannot. Always the *first* method tried, always the *last* one trusted.

### Trilateration / multilateration (GPS-style)

Multiple range or pseudorange measurements to known beacons. GPS is multilateration to satellites at known positions; UWB indoor systems do the same to fixed transmitters. Closed-form solutions exist for $\ge 3$ beacons in 2D, $\ge 4$ in 3D. Limitations: line-of-sight, multipath, requires infrastructure.

### EKF localization with landmarks

State = robot pose. Landmarks (known map features) provide measurements; the filter linearizes the measurement function and runs the standard Kalman update. Works beautifully when (a) features can be reliably re-observed, (b) the prior pose is approximately known, and (c) the world is approximately Gaussian. Failure modes: data association (which observed feature corresponds to which map feature?), kidnapping.

### Monte Carlo localization (MCL)

Represent the posterior with $N$ particles, each a hypothesis pose. At each step:

1. **Predict** — propagate each particle through the motion model + sample noise.
2. **Update** — weight each particle by the measurement likelihood $p(\mathbf{z}_t \mid \mathbf{x}_t, \mathbf{m})$.
3. **Resample** — draw $N$ new particles with replacement weighted by the importance weights.

The particles cluster around high-probability poses. A multimodal posterior is represented honestly: two clusters of particles in two rooms means the robot is uncertain between two rooms.

**AMCL (Adaptive MCL)** improves on MCL by:

- **KLD-sampling** — adaptively varying the number of particles based on the spread of the current belief.
- **Random-particle injection** — adding a small fraction of uniform-random particles to recover from kidnapping.

AMCL is the default localization stack in ROS 1 and ROS 2 (`nav2_amcl`), and is the workhorse for indoor mobile robots.

### Scan matching

Compare the current LIDAR scan to a stored 2D occupancy map (or 3D point cloud / mesh) and find the pose that maximizes the match. Variants: ICP, NDT, point-to-plane ICP, correlation scan matching.

For 2D LIDAR + 2D occupancy map this is essentially error-free in feature-rich environments. The combination of "AMCL initialization + scan-matching tracking" is what most ROS-based mobile robots use.

---

## What "the map" looks like

The localization algorithm depends on the map representation:

| Map type | Used by | Strengths |
|---|---|---|
| **Occupancy grid** | AMCL, Cartographer | Standard 2D indoor, cheap |
| **3D point cloud** | LIDAR-based AVs (Apollo, KISS-ICP) | Direct LIDAR scan match |
| **Feature/landmark map** | Earlier SLAM / EKF | Sparse, low memory |
| **Semantic map** | Modern AVs (Waymo, Cruise) | Lane lines, signs, traffic lights |
| **HD map** | Self-driving stacks | Centimeter-level lane geometry, signage |
| **Topological** | Service robots, museum tours | Graph of waypoints, no metric |
| **Implicit (NeRF, Gaussian splatting)** | Research, 2023-2025 | Photorealistic, differentiable |

See [[Mapping]] for how each is built.

---

## Coordinate frames — the operational reality

Practical localization in ROS works on a tree of frames (see `tf2`):

```
map → odom → base_link → sensor_link
```

- `base_link → sensor_link` — fixed by URDF (where each sensor is mounted).
- `odom → base_link` — provided by wheel/visual/inertial odometry. Continuous and smooth, but drifts.
- `map → odom` — provided by localization. Discontinuous (jumps when the localizer corrects) but globally accurate.

A planner that sends a goal in the `map` frame and a controller that runs in the `odom` frame is the standard split — the map frame holds the "true" position; the odom frame holds the smooth motion. AMCL publishes the `map → odom` transform; everything else flows from it.

---

## Performance metrics

| Metric | What it measures |
|---|---|
| **Translational ATE (Absolute Trajectory Error)** | RMS distance between estimated and ground-truth poses |
| **Rotational ATE** | RMS angular error |
| **Relative Pose Error (RPE)** | Drift over a fixed window; isolates short-term accuracy |
| **Convergence time** | Time from kidnapping to re-localization |
| **Localization confidence** | Effective sample size (MCL), trace of $P$ (EKF) |

Standard benchmarks: KITTI, EuRoC, TUM-RGBD, Newer College, MIT-Stata. New benchmarks for indoor localization: NCLT, M2DGR.

---

## Worked example — running AMCL on a TurtleBot

A standard ROS 2 indoor stack:

1. **Map building offline** — drive the TurtleBot around with `slam_toolbox` to build a 2D occupancy grid, save it as a `.pgm + .yaml` pair.
2. **AMCL initialization** — set `initial_pose` from RViz, AMCL seeds 500 particles around it.
3. **Pose tracking** — at each LIDAR scan (5 Hz), AMCL re-weights particles by scan likelihood, resamples, publishes `map → odom`.
4. **Failure recovery** — when innovation grows large, AMCL injects 10% random-uniform particles; if any survive resampling, the filter recovers.

The system localizes to within ~5 cm in a featureful indoor environment. In a long featureless corridor, AMCL fails — there's no LIDAR information to disambiguate position along the corridor. This is *the* canonical failure mode of geometric localization.

---

## Tooling

| Tool | Use |
|---|---|
| **ROS 2 `nav2_amcl`** | Standard 2D AMCL implementation |
| **`robot_localization`** | EKF/UKF for sensor-fusion-based localization |
| **Cartographer** | Google's 2D/3D real-time SLAM (also does pure localization) |
| **HDL Localization** | 3D LIDAR localization |
| **SLAM Toolbox** | Modern 2D LIDAR SLAM + localization |
| **OpenVSLAM, ORB-SLAM3** | Visual-feature SLAM with relocalization |
| **GTSAM, Ceres** | Factor-graph back-ends |

---

## Recommended reading

- Thrun, Burgard, Fox, *Probabilistic Robotics* (2005), Ch. 7-8 — Markov localization, EKF, MCL, AMCL
- Dellaert, Fox, Burgard, Thrun (1999), *Monte Carlo Localization for Mobile Robots* — the MCL paper
- Fox (2003), *Adaptive KLD-Sampling-Based Monte Carlo Localization* — AMCL
- Hess et al. (2016), *Real-Time Loop Closure in 2D LIDAR SLAM* — Cartographer
- Macenski et al. (2020), *The Marathon 2: A Navigation System* — modern Nav2 stack overview

---

## Dataview

```dataview
LIST FROM #localization OR #state-estimation WHERE contains(file.outlinks, [[Localization]])
```
