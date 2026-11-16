---
title: Probabilistic Robotics
description: "Probabilistic robotics represents what a robot knows as probability distributions and updates them with Bayes' rule as it moves and senses. Covers the Bayes filter with a computed door-sensing example, the main filters (Kalman, extended and unscented Kalman, particle), factor graphs for SLAM, planning under uncertainty, and how probabilistic estimation and learned models work together in 2026."
aliases:
  - Probabilistic Robots
  - Bayes Filter
  - Bayesian Filtering
tags:
  - probabilistic-robotics
  - bayesian-inference
  - kalman-filter
  - uncertainty
  - state-estimation
type: concept
level: intermediate
subfield:
  - estimation-mapping
status: keystone
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Probability_and_Statistics_for_Robotics]]"
  - "[[Bayesian_Inference]]"
related:
  - "[[Localization]]"
  - "[[SLAM]]"
  - "[[Sensor_Fusion]]"
  - "[[SLAM_with_ROS]]"
  - "[[Knowledge_Representation_and_Reasoning]]"
key_takeaways:
  - "Robots never know their state exactly; probabilistic robotics keeps a belief (a probability distribution) and updates it with a motion step (prediction) and a measurement step (Bayes' rule)."
  - "The Kalman filter does this exactly for linear-Gaussian systems; particle filters handle multimodal beliefs such as 'I might be in either corridor'; factor graphs solve whole trajectories at once and power modern SLAM."
  - "In 2026 learned networks often provide measurements (detections, learned odometry), while probabilistic estimators fuse them and keep track of how uncertain the robot should be."
sources:
  - "Thrun, S., Burgard, W. and Fox, D. (2005). Probabilistic Robotics. https://mitpress.mit.edu/9780262201629/probabilistic-robotics/"
  - "Dellaert, F. and Kaess, M. (2017). Factor Graphs for Robot Perception. https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf"
  - "Dellaert, F. et al. (2026). GTSAM: Georgia Tech Smoothing and Mapping library. https://gtsam.org/"
  - "Moore, T. and Stouch, D. (2026). robot_localization: nonlinear state estimation for ROS. https://github.com/cra-ros-pkg/robot_localization"
---

# Probabilistic Robotics

**Probabilistic robotics** is the practice of representing everything a robot believes, such as where it is, what the map looks like, or where an object is, as a **probability distribution** rather than a single answer, and updating those beliefs as new evidence arrives. Sensors are noisy, wheels slip and the world changes; probability is the mathematics of reasoning under exactly those conditions.

> **Etymology.** Latin *probabilis*, "provable, credible". The term became the name of a field with Thrun, Burgard and Fox's 2005 textbook *Probabilistic Robotics*, which collected the methods that made robust localization and SLAM practical.

## The Bayes filter

A belief $bel(x_t)$ over the state is updated in two steps every cycle:

1. **Predict** with the motion model, $\overline{bel}(x_t) = \int p(x_t \mid u_t, x_{t-1})\, bel(x_{t-1})\, dx_{t-1}$. Uncertainty grows.
2. **Correct** with the measurement, $bel(x_t) = \eta\, p(z_t \mid x_t)\, \overline{bel}(x_t)$. Uncertainty shrinks.

Here $u_t$ is the control (for example odometry), $z_t$ the measurement, and $\eta$ normalises the result.

### Worked example: is the door open?

A robot's prior belief is 50% that a door is open. Its camera says "open" correctly 60% of the time when the door is open, and wrongly says "open" 30% of the time when it is closed. It sees "open":

$$P(\text{open}\mid z) = \frac{0.6 \times 0.5}{0.6 \times 0.5 + 0.3 \times 0.5} = \frac{0.30}{0.45} = 0.667$$

One noisy reading moves the belief from 50% to 67%, not to certainty. A second independent "open" reading moves it to $0.6 \times 0.667 / (0.6 \times 0.667 + 0.3 \times 0.333) = 0.80$. Evidence accumulates, and the robot can act when its belief is strong enough for the cost of being wrong.

## Main filters

| Filter | Belief represented as | Good for |
|---|---|---|
| **Kalman filter** | Gaussian (mean + covariance); exact for linear systems | IMU and odometry fusion, tracking |
| **Extended / unscented Kalman filter** | Gaussian through nonlinear models | Most robot state estimation (e.g. ROS `robot_localization`) |
| **Particle filter** | A set of weighted samples | Multimodal beliefs; global localization (AMCL in Nav2) |
| **Histogram / grid filters** | Probabilities over discrete cells | Occupancy grid mapping |
| **Factor graphs (smoothing)** | Joint distribution over a whole trajectory, solved by optimisation | SLAM and visual-inertial odometry (GTSAM, iSAM2, g2o) |

## Planning under uncertainty

When the robot is unsure, the best action may be to gather information first: look in the cupboard before trying to grasp. **POMDPs** (partially observable Markov decision processes) formalise this. Exact solutions are intractable for real robots, but approximations, belief-space planning and learned policies with memory (see [[RNN_and_LSTM]]) put the idea to work.

## Probabilistic estimation and learning (2026)

Learned networks and probabilistic estimators are complementary:
- networks supply measurements and their uncertainty (object detections, learned depth and odometry, contact estimates);
- filters and factor graphs fuse them with physics-based motion models, catch inconsistent readings, and say how confident the robot should be.

Learned models trained to output calibrated uncertainty, and differentiable filters trained end to end, blur the boundary further. The principle stays the same: a robot that knows how uncertain it is can act safely.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Probabilistic_Robotics]])
```
