---
title: Kalman Filter
description: A recursive Bayesian filter that maintains a Gaussian estimate of a system's state through a predict–update cycle. Optimal under linear-Gaussian assumptions; the foundation of every modern estimation pipeline (EKF, UKF, MSCKF, factor-graph smoothers).
tags:
  - sensors
  - kalman-filter
  - state-estimation
  - sensor-fusion
  - probabilistic-robotics
  - control
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-29
permalink: /kalman_filter/
related:
  - "[[Sensor_Fusion]]"
  - "[[Estimation]]"
  - "[[Sensors]]"
  - "[[IMU_Sensors]]"
  - "[[Localization]]"
  - "[[SLAM]]"
  - "[[GPS]]"
  - "[[Particle_Filter]]"
---

# Kalman Filter

The **Kalman filter** is a recursive Bayesian filter that maintains a Gaussian probability distribution over the state of a dynamic system, updating it as control inputs and noisy measurements arrive. Predict the next state from a motion model; update the prediction with each measurement; repeat. Under the assumptions of linear dynamics, linear measurement, and Gaussian noise, the filter is the **optimal** (minimum mean-squared error, maximum likelihood, minimum variance — they all coincide) estimator. Under almost any other assumption, it remains a remarkably good *practical* estimator and is the core of nearly every robot's state estimator in 2025.

> **Etymology.** Named after **Rudolf Emil Kálmán** (Hungarian-American, 1930–2016), whose 1960 paper *A New Approach to Linear Filtering and Prediction Problems* (J. Basic Engineering) introduced the recursive form. The Hungarian surname *Kálmán* derives from the Turkic *qalman*, "the one who stays/remains" — fitting for a filter that summarizes the past in a single Gaussian state. The 1960 paper was nearly rejected as too theoretical until Stanley Schmidt at NASA Ames realized it solved the Apollo navigation problem; the resulting flight code (the **Schmidt EKF**) put humans on the Moon. Kálmán received the Kyoto Prize (1985), the IEEE Medal of Honor (1974), and the U.S. National Medal of Science (2009).

---

## The model the filter assumes

A discrete-time linear system with Gaussian noise:

**State equation (motion model):**

$$
\mathbf{x}_k = F_k \mathbf{x}_{k-1} + B_k \mathbf{u}_k + \mathbf{w}_k, \qquad \mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, Q_k)
$$

**Measurement equation:**

$$
\mathbf{z}_k = H_k \mathbf{x}_k + \mathbf{v}_k, \qquad \mathbf{v}_k \sim \mathcal{N}(\mathbf{0}, R_k)
$$

where:

- $\mathbf{x}_k \in \mathbb{R}^n$ — the state at time $k$ (e.g., position, velocity, orientation).
- $\mathbf{u}_k \in \mathbb{R}^m$ — the control input (e.g., wheel commands, IMU readings).
- $\mathbf{z}_k \in \mathbb{R}^p$ — the measurement (e.g., GPS reading, LIDAR distance).
- $F_k$ — state-transition matrix.
- $B_k$ — control-input matrix.
- $H_k$ — measurement matrix.
- $Q_k$ — process noise covariance (uncertainty in the motion model).
- $R_k$ — measurement noise covariance (sensor noise).

The filter tracks the **belief** as a Gaussian: $p(\mathbf{x}_k \mid \mathbf{z}_{1:k}) = \mathcal{N}(\hat{\mathbf{x}}_k, P_k)$, with mean $\hat{\mathbf{x}}_k$ and covariance $P_k$.

---

## The five equations to memorize

The Kalman filter is one of the few algorithms that fits on a single index card. Every iteration has two phases.

### Predict

$$
\hat{\mathbf{x}}_k^- = F_k \hat{\mathbf{x}}_{k-1} + B_k \mathbf{u}_k
$$

$$
P_k^- = F_k P_{k-1} F_k^T + Q_k
$$

The state moves forward in time according to the model; the covariance grows by the process noise.

### Update

$$
\mathbf{y}_k = \mathbf{z}_k - H_k \hat{\mathbf{x}}_k^- \quad (\text{innovation: measurement } - \text{prediction})
$$

$$
S_k = H_k P_k^- H_k^T + R_k \quad (\text{innovation covariance})
$$

$$
K_k = P_k^- H_k^T S_k^{-1} \quad (\text{Kalman gain})
$$

$$
\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + K_k \mathbf{y}_k
$$

$$
P_k = (I - K_k H_k) P_k^-
$$

The Kalman gain $K_k$ is the heart of the algorithm — it is the optimal weight to put on the new measurement given the current uncertainty in the prediction and the measurement noise. When $R$ is large (noisy sensor), $K \to 0$ and the filter ignores the measurement. When $P^-$ is large (uncertain prediction), $K \to H^{-1}$ and the filter trusts the measurement entirely.

That single quantity — *how much should I trust this new evidence?* — is what the filter is computing.

---

## Why this is "optimal"

For linear-Gaussian systems, the Kalman filter is **simultaneously**:

- The **maximum-likelihood estimator** of the state given all measurements so far.
- The **minimum mean-squared error** estimator.
- The **conditional mean** $\mathbb{E}[\mathbf{x}_k \mid \mathbf{z}_{1:k}]$.
- The **best linear unbiased estimator (BLUE)** even without the Gaussian assumption.

These are normally four separate concepts that disagree; the linear-Gaussian special case happens to make them all give the same answer. That coincidence is what made the 1960 paper foundational.

The two-sensor inverse-variance-weighting result — see [[Sensor_Fusion]] — is the scalar special case of the Kalman update step.

---

## When the assumptions break — and what people use instead

Robot dynamics are rarely linear (rotations, ride dynamics, contact). Sensor models are rarely linear (camera projection, GPS lever-arm). Noise is rarely Gaussian (outliers, multipath, occlusion). So the textbook Kalman filter is almost never used directly. The variants are:

| Variant | Trick | Best for |
|---|---|---|
| **Extended Kalman Filter (EKF)** | Linearize $f$ and $h$ at current estimate via Jacobians | Mild nonlinearity, the most-deployed variant |
| **Unscented Kalman Filter (UKF)** | Propagate sigma points through nonlinearity (no Jacobians) | Stiff nonlinearity, easier to code |
| **Error-State / Indirect KF** | Filter on the *error* in state, not state itself; reset between updates | IMU integration (orientation manifold), strapdown INS |
| **Cubature Kalman Filter (CKF)** | Sigma-point variant with cubature integration rule | Higher-dimensional UKF replacement |
| **Information Filter** | Track $P^{-1}$ instead of $P$; sums of information across sensors | Sparse SLAM, distributed estimation |
| **Square-Root Filter** | Track $\sqrt{P}$ via Cholesky factor for numerical stability | Tactical-grade INS, long-running filters |
| **Multi-State Constraint KF (MSCKF)** | Marginalize feature positions, keep only camera poses | Visual-inertial odometry on a budget |
| **Particle Filter** | Represent the posterior with weighted samples | Strongly non-Gaussian, multi-modal (kidnapped robot) |

For modern robotics work the most-used three are **EKF** (most things), **UKF** (when EKF Jacobians are painful), and **MSCKF / factor-graph smoothers** (when full bundle adjustment is wanted).

---

## A worked example — tracking a 1D position

Suppose a robot moves in 1D with position $p$, velocity $v$, and constant control $u$ (commanded acceleration). State $\mathbf{x} = [p, v]^T$. Time step $\Delta t$:

**Motion model:**

$$
F = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}, \qquad B = \begin{bmatrix} \Delta t^2 / 2 \\ \Delta t \end{bmatrix}
$$

**Measurement model:** GPS-like noisy position only.

$$
H = \begin{bmatrix} 1 & 0 \end{bmatrix}, \qquad R = \sigma_z^2
$$

Each step: predict using the constant-acceleration model, update with the GPS reading. After many iterations the filter naturally converges to a steady-state Kalman gain — the position estimate is a low-pass-filtered GPS, and the velocity estimate is a filtered finite-difference of GPS, both with provably-optimal smoothing.

This is the same filter that was running on the Apollo Lunar Module, scaled up to 6 DoF and run in onboard real time.

---

## Practical engineering — what makes a Kalman filter actually work

Implementing the equations is easy. Making the filter work on a real robot is harder. The recurring pitfalls:

1. **Tuning $Q$ and $R$.** The two covariance matrices encode "trust the model" vs "trust the sensor" and they're rarely well-known a priori. Best practice: estimate $R$ from sensor datasheet + Allan variance; tune $Q$ to make the **innovation $\mathbf{y}_k$ behave like white noise** (innovation analysis).
2. **Numerical stability.** $P$ should stay symmetric and positive-definite. If you ever see $P$ go non-PSD, switch to the **Joseph form** $P_k = (I - K_k H_k) P_k^- (I - K_k H_k)^T + K_k R_k K_k^T$ which guarantees symmetry and positive-definiteness.
3. **Outliers.** A single bad measurement (GPS multipath, LIDAR misregistration) can ruin a Gaussian filter. Gate measurements by **Mahalanobis distance** $\mathbf{y}_k^T S_k^{-1} \mathbf{y}_k > \chi^2_{0.99}$ → reject.
4. **Filter divergence.** If $P$ gets too small, the filter ignores measurements (an "overconfident" filter). Add **process-noise inflation** when innovations are consistently large.
5. **Time-aligned measurements.** The textbook update step assumes $\mathbf{z}_k$ corresponds to the state $\mathbf{x}_k$ at the same instant. Out-of-order or delayed measurements need either a **smoother** (re-process the past) or **rollback-and-replay** logic.
6. **State-space choice.** For orientation, do *not* use Euler angles in the state — use a quaternion or rotation matrix, with the **error-state** formulation. Otherwise the gimbal-lock singularity creeps in. See [[Lie_Groups]].

---

## When *not* to use a Kalman filter

The KF is wrong when:

- **The posterior is multi-modal.** A robot that could be in either of two rooms after kidnapping cannot be summarized by a single Gaussian. Use a particle filter ([[Localization]]: MCL/AMCL).
- **Measurements are heavy-tailed.** Huber/Cauchy robust kernels in a factor-graph smoother handle this better.
- **Constraints are equalities.** Constrained estimation (objects on a known plane, vehicles on a road) is awkward in KF form; constraint-projection or factor-graph constraints work better.
- **You want to keep all the information.** A KF marginalizes the past into a single Gaussian, losing detail. A factor graph (GTSAM, Ceres) keeps all measurements and re-optimizes — slower but recovers the loss.

---

## Tooling

| Library | Notes |
|---|---|
| **`robot_localization` (ROS 2)** | Production EKF/UKF for IMU + GPS + odom + others |
| **PX4 / ArduPilot EKF2** | Drone-grade error-state EKF |
| **`filterpy`** | Pure-Python KF/EKF/UKF/PF for prototyping (Roger Labbe's *Kalman and Bayesian Filters in Python*) |
| **GTSAM** | Factor-graph SLAM with iSAM2 incremental smoother |
| **Ceres Solver** | General-purpose nonlinear least-squares (Google) |
| **`numpy` + `scipy`** | Quick custom filters |
| **MATLAB Sensor Fusion Toolbox** | Industry-standard for aerospace |

---

## Recommended reading

- Kalman, R. E. (1960), *A New Approach to Linear Filtering and Prediction Problems*, J. Basic Engineering — the original
- Thrun, Burgard, Fox, *Probabilistic Robotics* (2005), Ch. 3 — robotics-flavoured derivation
- Barfoot, *State Estimation for Robotics* (2017) — modern Lie-theoretic treatment, free PDF
- Labbe, *Kalman and Bayesian Filters in Python* — interactive Jupyter tutorial (free)
- Welch & Bishop (2006), *An Introduction to the Kalman Filter* (UNC tech report) — short, clear, classic
- Solà (2017), *Quaternion kinematics for the error-state Kalman filter* (arXiv 1711.02508) — the modern IMU-friendly reference

---

## Dataview

```dataview
LIST FROM #kalman-filter OR #state-estimation WHERE contains(file.outlinks, [[Kalman_Filter]])
```
