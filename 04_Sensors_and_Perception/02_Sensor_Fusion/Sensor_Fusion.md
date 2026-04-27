---
title: Sensor Fusion
description: The art of combining measurements from multiple sensors into a single, more accurate, more reliable estimate than any sensor could provide alone. The probabilistic glue between heterogeneous, asynchronous, error-prone sensor streams.
tags:
  - sensors
  - sensor-fusion
  - state-estimation
  - perception
  - kalman-filter
  - probabilistic-robotics
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-29
permalink: /sensor_fusion/
related:
  - "[[Sensors]]"
  - "[[Perception]]"
  - "[[Kalman_Filter]]"
  - "[[Estimation]]"
  - "[[Localization]]"
  - "[[SLAM]]"
  - "[[IMU_Sensors]]"
  - "[[GPS]]"
  - "[[Camera_Systems]]"
  - "[[LIDAR]]"
---

# Sensor Fusion

**Sensor fusion** is the process of combining measurements from multiple sensors into a single estimate that is more accurate, more complete, and more robust than any single sensor's reading. It is the way modern robots reconcile a noisy, slow, but globally accurate GPS with a fast, low-noise, but drift-prone IMU; or a dense LIDAR point cloud with a semantically rich camera image; or a high-rate gyroscope with a slower visual-pose update. The output is one consistent state — pose, velocity, map, object track — that downstream planners and controllers can rely on.

> **Etymology.** *Fusion* — from Latin *fundere*, "to pour, melt, blend together," via *fusio*, "the act of pouring." Same root as *fuse* (electrical), *foundry*, and *infuse*. Originally a metallurgical term — pouring molten metal into a mold. The metaphor in *sensor fusion* is exact: the individual sensor streams are like separate metals, each with its own weakness, that are melted together to produce a single alloy stronger than any of its components.

---

## Why we fuse — the four reasons no single sensor is enough

1. **Complementary strengths.** GPS has accurate global position but slow update and no orientation. IMU has fast orientation but unbounded position drift. Each fixes the other's blind spot.
2. **Redundancy.** A sensor can fail (camera in tunnel, GPS in canyon, LIDAR in fog). Fusion lets the system fall back gracefully when one sensor goes dark.
3. **Noise reduction.** Two independent measurements of the same quantity, weighted optimally by their variances, produce a fused estimate with lower variance than either alone — the **inverse-variance weighting** result, $\sigma_{\text{fused}}^{-2} = \sigma_1^{-2} + \sigma_2^{-2}$.
4. **Asynchronous coverage.** A fast sensor (IMU at 1 kHz) keeps the state up-to-date between slow updates from a heavy sensor (camera at 30 Hz, GPS at 10 Hz).

The two-sensor fusion math is striking enough to remember: if two sensors give estimates $\hat{x}_1, \hat{x}_2$ with variances $\sigma_1^2, \sigma_2^2$, the optimal fused estimate is

$$
\hat{x}_{\text{fused}} = \frac{\sigma_2^2}{\sigma_1^2 + \sigma_2^2} \hat{x}_1 + \frac{\sigma_1^2}{\sigma_1^2 + \sigma_2^2} \hat{x}_2, \qquad \sigma_{\text{fused}}^2 = \frac{\sigma_1^2 \sigma_2^2}{\sigma_1^2 + \sigma_2^2}
$$

Each sensor is weighted *inversely* by its noise — the noisier one gets less weight, but its information is not thrown away. This is the heart of the [[Kalman_Filter]] and every Bayesian fusion algorithm that builds on it.

---

## Three levels of fusion

The literature distinguishes where in the perception pipeline fusion happens:

| Level | Inputs | Outputs | Examples |
|---|---|---|---|
| **Low / data level** | Raw sensor signals | Fused signal | Image pixel + LIDAR depth → RGB-D, ICP point matching |
| **Mid / feature level** | Per-sensor features | Joint feature representation | Visual feature + IMU preintegrated motion → VIO |
| **High / decision level** | Per-sensor estimates | Fused estimate | Multiple object trackers' bounding boxes → fused track |

Low-level fuses *information* (with the most data and the most computational cost); high-level fuses *decisions* (cheaper but discards detail). Modern stacks are typically *hybrid*: deep networks fuse at low/mid level (BEV features from camera + LIDAR), while a Kalman filter fuses at high level (tracker outputs + IMU pose).

---

## The Bayesian framework — the textbook story

Sensor fusion is a special case of **Bayesian inference**. We have:

- A *state* $x$ — what we want to estimate (robot pose, map, object position).
- A *prior* $p(x)$ — what we believed before this measurement.
- A *measurement* $z$ from a sensor with a known model $p(z \mid x)$.
- The *posterior* $p(x \mid z)$ — what we should believe now.

Bayes' rule gives the recipe:

$$
p(x \mid z) = \frac{p(z \mid x) \, p(x)}{p(z)}
$$

Add a second sensor $z_2$ with model $p(z_2 \mid x)$ assumed conditionally independent of $z_1$:

$$
p(x \mid z_1, z_2) \propto p(z_1 \mid x) \, p(z_2 \mid x) \, p(x)
$$

Multiplying likelihoods is the formal expression of "combine the evidence."

In practice, $x$ is high-dimensional and the integrals are intractable. Different algorithms make different approximations:

- **Kalman filter (KF):** assume linear models and Gaussian noise → closed-form. See [[Kalman_Filter]].
- **Extended Kalman filter (EKF):** linearize about the current estimate. The most common engineering choice.
- **Unscented Kalman filter (UKF):** propagate sigma points through the nonlinearity instead of linearizing. Better for stiff nonlinearities.
- **Particle filter:** represent the posterior with samples (particles). Handles non-Gaussian and multi-modal distributions. Foundation of MCL/AMCL ([[Localization]]).
- **Factor graphs / smoothing:** keep all measurements and re-optimize a sliding window of states. The modern SLAM/VIO standard (GTSAM, Ceres, g2o).
- **Information filter / Square-root filter:** numerically more stable variants of the KF, used in tactical-grade INS.

---

## The classic robotics fusion recipes

### IMU + GPS — the baseline navigation stack

GPS gives $(x, y, z)$ in a global frame at 1–10 Hz with meter accuracy. IMU gives $(\boldsymbol{\omega}, \mathbf{a})$ at 100–1000 Hz with subdegree-per-second drift. An EKF with a 15-state vector — pose (6) + velocity (3) + accelerometer bias (3) + gyro bias (3) — fuses them. Output: 100–1000 Hz pose at GPS accuracy. This is what every drone autopilot does, and it's what your phone's "GPS + sensors" mode is doing.

### Visual-Inertial Odometry (VIO) — for GPS-denied environments

Camera + IMU. The camera provides drift-correcting position-and-orientation updates from feature tracking; the IMU provides high-rate motion between frames. Modern systems (VINS-Mono, OKVIS, OpenVINS) use **IMU preintegration** (Forster 2015) to build factor-graph constraints between consecutive camera frames. Drift: 0.1–1% of distance traveled.

### LIDAR-Inertial Odometry (LIO)

LIDAR + IMU. Point cloud registration (ICP, NDT, KISS-ICP) provides relative motion between scans; the IMU motion-compensates the cloud during a sweep and bridges scan-to-scan. Standard examples: LIO-SAM, FAST-LIO, FAST-LIO2.

### LIDAR + camera + IMU + GPS — the kitchen sink

Modern AV stacks (Waymo, Cruise, Apollo) fuse all four. LIDAR for geometry, camera for semantics, IMU for high-rate motion, GPS for absolute pose. The fusion is split across multiple filters/graphs: a state-estimation filter for pose+velocity, a tracking filter for moving objects, a SLAM back-end for map registration.

---

## Implementation gotchas — the painful real-world list

1. **Time synchronization.** A 30 ms timestamp mismatch on a 100°/s motion is 3°. Fix it with hardware sync (PTP, IEEE-1588) or hardware-triggered cameras. *Fixing this is the single highest-leverage thing you can do.*
2. **Coordinate frames.** Every sensor has its own frame, attached to a robot link. Get the static transform tree right (URDF, `tf2`) before anything else. See [[Sensor_Calibration_Techniques]].
3. **Bias estimation.** IMU biases drift over time — they have to be filter states, not constants. The `robot_localization` ROS package and PX4's EKF2 do this correctly out of the box.
4. **Outlier rejection.** A single bad measurement (LIDAR misregistration, GPS multipath) can ruin a Gaussian filter. Use innovation gating, RANSAC, or robust kernels (Huber, Cauchy) in the back-end.
5. **Initialization.** Filters need a reasonable starting estimate. VIO needs at least 1 m of motion to estimate metric scale; GPS needs 5–10 satellites; IMU-only needs a stationary period to estimate gravity direction.
6. **Asynchronous arrival.** Sensors don't agree on rates or timing. Either buffer to the slowest, run a fixed-rate filter and apply measurements as they arrive, or use a smoother that re-optimizes on each measurement.

---

## Worked example — fusing a noisy GPS with a noisy IMU position

Suppose a stationary robot has:

- GPS reading: $\hat{x}_g = 10.5$ m, $\sigma_g = 2.0$ m (2 m one-sigma horizontal accuracy).
- IMU-integrated position: $\hat{x}_i = 9.5$ m, $\sigma_i = 0.5$ m (good for short windows).

Inverse-variance fusion:

$$
\hat{x}_f = \frac{(0.5)^2 \cdot 10.5 + (2.0)^2 \cdot 9.5}{(0.5)^2 + (2.0)^2} = \frac{0.25 \cdot 10.5 + 4 \cdot 9.5}{4.25} = \frac{2.625 + 38}{4.25} = 9.56 \text{ m}
$$

$$
\sigma_f = \sqrt{\frac{(0.5)^2 \cdot (2.0)^2}{(0.5)^2 + (2.0)^2}} = \sqrt{\frac{1.0}{4.25}} = 0.485 \text{ m}
$$

The IMU dominates because it's currently more confident. The fused estimate has tighter uncertainty than either alone. As the IMU drifts (its $\sigma_i$ grows over time), the GPS gradually takes over the weighting. This is exactly what a Kalman filter does, generalized to vector states and arbitrary linear models.

---

## Tooling

| Tool | Use |
|---|---|
| **`robot_localization` (ROS 2)** | Production-quality EKF/UKF for pose+velocity fusion |
| **PX4 / ArduPilot EKF2** | Drone-grade IMU + GPS + magnetometer fusion |
| **GTSAM** | Factor-graph back-end for SLAM and VIO |
| **OpenVINS, VINS-Fusion** | Open-source VIO implementations |
| **FAST-LIO2, LIO-SAM** | Open-source LIDAR-inertial odometry |
| **MSCKF** | Multi-state-constraint Kalman filter (Mourikis & Roumeliotis 2007) |
| **Drake `MultibodyPlant` + perception** | Drake's integrated state estimation |

---

## Recommended reading

- Thrun, Burgard, Fox, *Probabilistic Robotics* (2005), Ch. 3-4 — the canonical Bayes-filter framework
- Barfoot, *State Estimation for Robotics* (2017) — modern Lie-theoretic treatment, free PDF
- Mourikis & Roumeliotis (2007), *A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation* — the MSCKF paper
- Forster, Carlone, Dellaert, Scaramuzza (2015), *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry* — modern VIO foundations
- Hall & Llinas, *Handbook of Multisensor Data Fusion* (2nd ed., 2008) — older but broad survey

---

## Dataview

```dataview
LIST FROM #sensor-fusion OR #robotics WHERE contains(file.outlinks, [[Sensor_Fusion]])
```
