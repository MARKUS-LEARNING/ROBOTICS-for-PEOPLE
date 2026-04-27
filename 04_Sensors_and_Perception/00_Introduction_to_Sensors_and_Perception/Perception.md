---
title: Perception
description: The pipeline that converts raw sensor measurements into a useful internal representation of the robot's body and environment. The bridge between physics (what the sensor measured) and decision-making (what the robot should do).
tags:
  - robotics
  - perception
  - sensors
  - state-estimation
  - computer-vision
  - sensor-fusion
layout: default
category: robotics
author: Jordan_Smith
date: 2025-05-02
permalink: /perception/
related:
  - "[[Sensors]]"
  - "[[Sensor_Fusion]]"
  - "[[Computer_Vision]]"
  - "[[SLAM]]"
  - "[[Localization]]"
  - "[[Mapping]]"
  - "[[Kalman_Filter]]"
  - "[[State_Estimation]]"
  - "[[Object_Recognition]]"
  - "[[Deep_Learning]]"
---

# Perception

**Perception** is the pipeline that turns raw sensor measurements into an internal representation the robot can act on. A camera produces millions of pixels per second and a LIDAR produces hundreds of thousands of 3D points per second, but a planner does not consume pixels or points — it consumes statements like "there is a chair at $(x, y)$, the robot is at $(x', y')$ facing north, the floor is clear from here to there." Perception is the machinery that produces those statements.

> **Etymology.** From Latin *percipere* = *per-* "thoroughly" + *capere* "to seize, grasp." Literally, "to grasp fully." The English word arrives via Old French *perception*, used in the 14th century for the act of grasping with the senses. Note the symmetry with *concept* (Latin *concipere*, "to take in together") — a *concept* is internal, a *percept* is the immediate result of taking in the world. Robotics inherits both meanings: the perception system *grasps* the world, and the result is a set of *concepts* the planner can reason over.

---

## Why perception is hard

A naive view of perception says "just read the sensor." The actual problems are these:

1. **Sensors are noisy.** Every reading is the truth plus error (see [[Sensors]]). A single measurement is rarely trustworthy.
2. **Sensors are partial.** A camera sees only what is in front of it; a LIDAR cannot see through walls; a GPS doesn't work indoors. No single sensor gives a complete picture.
3. **The world is high-dimensional.** A 1080p image is a 2,073,600-dimensional vector. Reasoning over raw pixels is intractable; you need a *representation* of much lower dimension.
4. **The world changes.** People move, lighting changes, doors open. A perception system that worked at 9 AM may fail at noon.
5. **Time matters.** Sensor measurements arrive asynchronously, with latency. The robot needs an estimate of the world *now*, not 50 ms ago.

The job of the perception stack is to address these five problems simultaneously: smooth out noise, fuse partial views, compress to a useful representation, adapt to change, and project to current time.

---

## The perception stack

A canonical robotics perception pipeline runs roughly bottom-up:

```
Raw signals    →  Preprocessing  →  Features  →  Objects/Scene  →  State Estimate
(pixels,          (denoise,          (edges,        (chair,         (robot pose,
 LIDAR points,     calibrate,         keypoints,     person,         map,
 IMU samples)      timestamp)         clusters)      lane)           velocities)
```

Each layer takes the layer below as input and produces a more abstract, lower-dimensional representation. The arrows are not strictly one-way — modern systems feed object hypotheses back to the feature layer to bias detection (top-down attention), and they pass uncertainty estimates up alongside point estimates.

The key insight: **each layer should output not just a value but a *belief* (a point estimate + an uncertainty)**. Bayesian-filtering frameworks ([[Kalman_Filter]], particle filters, factor graphs) are the formal expression of this idea — perception isn't "read the sensor and report a number," it's "maintain a probability distribution over the world state and update it as evidence arrives."

---

## The four canonical perception subproblems

| Subproblem | Question | Typical answer | Tools |
|---|---|---|---|
| **State estimation** | "Where am I and how am I moving?" | Robot pose + velocity | IMU + encoders + GPS, EKF/UKF |
| **Mapping** | "What does the world look like?" | Occupancy grid / point cloud / mesh | LIDAR / RGB-D, [[Mapping]] |
| **Localization** | "Where am I, given a map?" | Pose in map frame | LIDAR matching, AMCL, [[Localization]] |
| **Object understanding** | "What's in front of me?" | Bounding boxes, classes, segmentation | Cameras + neural nets, [[Computer_Vision]] |

[[SLAM]] is the joint solution of mapping + localization when neither is given a priori — the robot builds the map and locates itself in it simultaneously.

---

## Active vs passive perception

A passive perception system processes whatever sensor data arrives. An *active* perception system *chooses* what to measure next. Examples:

- A robot tilts its head to disambiguate two object hypotheses
- A drone moves to a viewpoint that improves coverage of an unexplored region
- A manipulator probes a deformable object with its fingers to figure out its shape

Active perception is the bridge between perception and planning — the perception system decides which measurements it needs to reduce its uncertainty, and the planner executes the motions to acquire them. Modern formulations cast this as **information-theoretic exploration**: pick the action that maximizes expected information gain, $\mathbb{E}[I(X; Z_{a})]$, where $Z_a$ is the measurement obtained by action $a$ and $X$ is the world state.

---

## Classical vs learned perception

Perception research has two big eras:

| | **Classical (1970s–2010)** | **Learned (2012–present)** |
|---|---|---|
| Approach | Hand-engineered features (SIFT, ORB, edge detectors) + geometric models | Deep neural networks trained end-to-end |
| Feature extraction | Designed by humans | Learned from data |
| Strengths | Interpretable, well-understood failure modes, low data requirements | Far higher accuracy on hard real-world data |
| Weaknesses | Fragile to lighting, viewpoint, clutter | Opaque failures, large datasets, distribution shift |
| Where each still wins | Geometric problems (SLAM, calibration), low-data domains | Object detection, semantic segmentation, depth from monocular |

The honest answer in 2025 is **hybrid**: classical geometry for the metric layer (poses, depths, optimization) plus learned components for the semantic layer (what *is* this object?). [[SLAM]] systems like ORB-SLAM3 and OKVIS keep classical bundle adjustment at their core; perception modules in autonomous-driving stacks (Waymo, Tesla, Wayve) are learned but their outputs are fused into a classical Kalman/factor-graph state estimator.

See [[Computer_Vision]] for the visual side and [[State_Estimation]] for the geometric side.

---

## Worked example — perceiving a chair to grasp it

A pick-and-place task that humans treat as one motion ("grab the chair") decomposes into a perception waterfall:

1. **Raw RGB-D frame** arrives from the wrist camera (640 × 480 × 4 bytes, 30 Hz).
2. **Preprocessing** — depth-denoise, fill missing pixels, register depth to color.
3. **Object detection** — a YOLO-style network outputs `chair, bbox=(x, y, w, h), conf=0.91`.
4. **Segmentation** — Mask R-CNN refines the bounding box to a per-pixel mask.
5. **3D pointing** — masked depth pixels become a partial point cloud of the chair.
6. **Pose estimation** — fit a CAD model or use a learned 6-DoF pose net to recover the chair's $SE(3)$ pose.
7. **Grasp synthesis** — given the pose and an antipodal-grasp library, select a graspable point on the back of the chair.
8. **Output to planner** — `target = SE(3) pose, gripper width = 30 mm`.

Steps 1-2 are signal processing; 3-4 are learned vision; 5-6 are geometry; 7-8 are task-specific reasoning. Each step has its own failure mode and its own confidence value — the planner that consumes the result uses that confidence to decide whether to grasp now, gather more views, or fail safely.

---

## Tooling

| Tool | Used for |
|---|---|
| **OpenCV** | Classical image processing, calibration, feature detection |
| **PCL (Point Cloud Library)** | LIDAR / RGB-D point-cloud processing |
| **Open3D** | Modern Python-first point-cloud + mesh library |
| **PyTorch / JAX** | Neural-network perception (detection, segmentation, depth) |
| **GTSAM, Ceres, g2o** | Factor-graph and bundle-adjustment back-ends |
| **ROS 2 perception stack** | `sensor_msgs`, `vision_msgs`, `nav_msgs`, `tf2` |
| **Drake `PerceptionSystems`** | Integrated perception inside Drake's diagram framework |
| **Isaac ROS** | NVIDIA's GPU-accelerated perception nodes (deep-learning models, VSLAM) |

---

## Recommended reading

- Thrun, Burgard, Fox, *Probabilistic Robotics* (2005) — the canonical textbook for perception-as-state-estimation
- Hartley & Zisserman, *Multiple View Geometry in Computer Vision* (2nd ed., 2003) — the geometric foundations
- Szeliski, *Computer Vision: Algorithms and Applications* (2nd ed., 2022) — broad classical + modern coverage (free PDF)
- Marr, *Vision* (1982) — the philosophical foundation; introduced the "primal sketch → 2½D sketch → 3D model" hierarchy
- Bajcsy, *Active Perception* (1988) — the original case for choosing what to measure

---

## Dataview

```dataview
LIST FROM #perception OR #robotics WHERE contains(file.outlinks, [[Perception]])
```
