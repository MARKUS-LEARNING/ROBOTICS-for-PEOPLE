---
title: Advanced Sensor Fusion
description: "Combining several sensors into one better estimate: the inverse-variance principle (computed), Kalman-family filters, tightly and loosely coupled fusion, factor-graph smoothing for visual-inertial and lidar-inertial odometry, time synchronisation and calibration, and learned fusion in bird's-eye-view networks, with guidance on what robots use in 2026."
aliases:
  - Multi-Sensor Fusion
  - Visual-Inertial Odometry
  - VIO
tags:
  - sensor-fusion
  - state-estimation
  - kalman-filter
type: concept
level: advanced
subfield:
  - estimation-mapping
  - perception
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Sensor_Fusion]]"
  - "[[Probabilistic_Robotics]]"
related:
  - "[[SLAM_with_ROS]]"
  - "[[Event_Based_Sensing_and_Processing]]"
  - "[[Localization]]"
key_takeaways:
  - "Independent estimates combine by weighting each with the inverse of its variance: fusing variances 0.04 and 0.01 gives 0.008, better than either."
  - "Tightly coupled fusion (raw measurements in one estimator) beats loosely coupled fusion (combining finished estimates) when sensors are individually weak, at the cost of complexity; factor graphs are the standard for VIO and LIO."
  - "Calibration and time synchronisation are where most fusion systems fail; learned bird's-eye-view fusion now dominates multi-camera and lidar perception for driving."
sources:
  - "Thrun, S., Burgard, W. and Fox, D. (2005). Probabilistic Robotics. https://mitpress.mit.edu/9780262201629/probabilistic-robotics/"
  - "Qin, T., Li, P. and Shen, S. (2017). VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator. https://arxiv.org/abs/1708.03852"
  - "Geneva, P. et al. (2026). OpenVINS: open-source visual-inertial navigation. https://github.com/rpng/open_vins"
  - "Liu, Z. et al. (2022). BEVFusion: Multi-Task Multi-Sensor Fusion with Unified Bird's-Eye View Representation. https://arxiv.org/abs/2205.13542"
  - "Dellaert, F. and Kaess, M. (2017). Factor Graphs for Robot Perception. https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf"
---

# Advanced Sensor Fusion

Every sensor has blind spots:
- cameras fail in the dark or under motion blur;
- IMUs drift;
- GNSS bounces off buildings;
- lidar struggles with glass, rain and long featureless corridors.

**Sensor fusion** combines them so that the result is more accurate and more robust than any one alone. The basics are in [[Sensor_Fusion]]; this note covers the methods used in current robots.

> **Etymology.** Latin *fundere*, "to pour": fusion pours several streams into one.

## The principle, computed

Two independent, unbiased measurements of the same quantity, with variances $\sigma_1^2$ and $\sigma_2^2$, combine best by weighting each with the inverse of its variance:

$$\hat x = \frac{x_1/\sigma_1^2 + x_2/\sigma_2^2}{1/\sigma_1^2 + 1/\sigma_2^2}, \qquad \sigma^2 = \Big(\frac{1}{\sigma_1^2} + \frac{1}{\sigma_2^2}\Big)^{-1}.$$

With $\sigma_1^2 = 0.04$ m² (wheel odometry) and $\sigma_2^2 = 0.01$ m² (lidar matching), the fused variance is $1/(25 + 100) = 0.008$ m², better than either. The noisier sensor still contributes, with weight $25/125 = 0.2$. The Kalman filter applies this rule recursively, with a motion model in between.

## Architectures

| Approach | What is fused | Pros | Cons |
|---|---|---|---|
| **Loosely coupled** | Each sensor's finished estimate (e.g. a GNSS position and a VO pose) | Modular, simple | Throws away information; fails if one sub-estimate fails |
| **Tightly coupled** | Raw measurements (feature tracks, IMU samples, lidar points) in one estimator | More accurate and robust | Complex; needs a good joint model |
| **Filtering** (EKF, error-state EKF, UKF, MSCKF) | Current state only | Fast, constant cost | Linearisation errors accumulate |
| **Smoothing** (factor graphs, sliding-window optimisation) | A window or whole trajectory | Accurate, relinearises | More computation |

Visual-inertial odometry (VINS-Mono, OpenVINS), lidar-inertial odometry (FAST-LIO2, GLIM) and full SLAM back-ends (GTSAM, iSAM2) are tightly coupled factor-graph or filter systems (see [[SLAM_with_ROS]]).

## The unglamorous essentials

- **Extrinsic calibration**: where each sensor sits relative to the others; a 1° camera–IMU misalignment corrupts every estimate.
- **Intrinsic calibration**: lens distortion, IMU biases and scale factors.
- **Time synchronisation**: hardware triggering or precise timestamps. A 10 ms offset between camera and IMU, at 3 rad/s of rotation, is a 1.7° error.
- **Outlier rejection**: robust cost functions, consistency checks, gating.
- **Degeneracy detection**: noticing when a sensor carries no information (lidar in a featureless tunnel) and down-weighting it.

## Learned fusion (2026)

Autonomous-driving perception fuses multiple cameras and lidar into a shared **bird's-eye-view** representation inside one network (BEVFusion, 2022, and successors), learning what classical fusion encodes by hand. In robots, learned components increasingly supply measurements (learned odometry, depth, contact) to classical estimators, while end-to-end policies fuse sensors implicitly. Classical filters remain essential wherever uncertainty must be explicit and auditable.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Advanced_Sensor_Fusion]])
```
