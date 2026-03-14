---
title: Euclidean Space
description: Euclidean Space is a mathematical space characterized by the familiar properties of Euclidean geometry, providing the framework for understanding spatial relationships and motion in robotics.
tags:
  - robotics
  - mathematics
  - geometry
  - kinematics
  - euclidean-space
  - engineering
  - glossary-term
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /euclidean_space/
related:
  - "[[Kinematics]]"
  - "[[Vector_Mathematics]]"
  - "[[Coordinate_Systems]]"
  - "[[Distance_Metrics]]"
  - "[[Transformations]]"
  - "[[Robot_Design]]"
---

# Euclidean Space

**Euclidean Space** is a mathematical space characterized by the familiar properties of Euclidean geometry. It provides the framework for understanding spatial relationships and motion, making it foundational in robotics for tasks involving positioning, navigation, and manipulation. Euclidean space is defined by a set of axioms that describe the behavior of points, lines, and planes in a flat, continuous space.

---
![image](https://github.com/user-attachments/assets/015cdb7d-b0d4-42d6-85a2-4028716519a9)

<font size=1>*source: https://sciencephotogallery.com/featured/three-dimensional-euclidean-space-science-photo-library.html*</font>
---

## Key Concepts

### Dimensions

Euclidean space can have any number of dimensions, but the most commonly used in robotics are two-dimensional (2D) and three-dimensional (3D) spaces. These dimensions allow for the representation of points and vectors in a Cartesian coordinate system.

### Distance Metric

The distance between two points in Euclidean space is given by the Euclidean distance formula, which is derived from the Pythagorean theorem. This metric is crucial for calculating distances and positions in robotic applications.

### Vector Mathematics

Vectors in Euclidean space are used to represent quantities with both magnitude and direction. Vector operations, such as addition, subtraction, and dot products, are essential for describing motion and forces in robotics.

### Transformations

Transformations in Euclidean space, such as translations and rotations, are used to describe the movement and orientation of robotic systems. These transformations are often represented using matrices and are fundamental in kinematics and dynamics.

---

## Mathematical Formulation

### Distance Metric

The Euclidean distance between two points $\mathbf{p}_1$ and $\mathbf{p}_2$ in $n$ -dimensional space is given by:

$$
d(\mathbf{p}_1, \mathbf{p}_2) = \sqrt{\sum_{i=1}^{n} (p_{1i} - p_{2i})^2}
$$

where $p_{1i}$ and $p_{2i}$ are the coordinates of the points in the $i$-th dimension.

### Vector Representation

A vector $\mathbf{v}$ in $n$ -dimensional Euclidean space can be represented as:

$$
\mathbf{v} = \begin{bmatrix} v_1 \\ v_2 \\ \vdots \\ v_n \end{bmatrix}
$$

### Transformation Matrix

A transformation matrix $T$ for translating and rotating a vector $\mathbf{v}$ in 3D Euclidean space can be represented as:

$$
T = \begin{bmatrix}
R & \mathbf{t} \\
\mathbf{0} & 1
\end{bmatrix}
$$

where $R$ is the rotation matrix, and $\mathbf{t}$ is the translation vector.

### Example: Robotic Arm

Consider a robotic arm operating in 3D Euclidean space. The position of the end-effector can be represented as a vector $\mathbf{p}$:

$$
\mathbf{p} = \begin{bmatrix} x \\ y \\ z \end{bmatrix}
$$

The distance between the current position $\mathbf{p}_c$ and the target position $\mathbf{p}_t$ is calculated using the Euclidean distance formula:

$$
d = \sqrt{(x_t - x_c)^2 + (y_t - y_c)^2 + (z_t - z_c)^2}
$$

This distance metric is used to plan the arm's motion and ensure it reaches the target position efficiently.

---

## Distance Metrics Comparison for Robotics

While the Euclidean distance is the most natural metric in physical space, several distance metrics are useful in different robotics contexts:

| Metric | Formula ($\mathbb{R}^n$) | Robotics Application |
|---|---|---|
| **Euclidean** ($L^2$) | $d = \sqrt{\sum (x_i - y_i)^2}$ | Path length in workspace, nearest-neighbor queries in point clouds |
| **Manhattan** ($L^1$) | $d = \sum \lvert x_i - y_i \rvert$ | Grid-based path planning (4-connected grids), taxicab-style mobile robot navigation |
| **Chebyshev** ($L^\infty$) | $d = \max_i \lvert x_i - y_i \rvert$ | Grid-based planning (8-connected grids), bounding-box collision checks, joint-space distance when each joint moves independently at max speed |

**Practical notes:**
- In **joint space**, the Chebyshev ($L^\infty$) metric often better predicts motion time than Euclidean distance, because the total move time is determined by the slowest joint (the one with the largest angular displacement).
- In **configuration space** planning, the metric choice affects the behavior of sampling-based planners like RRT. The Euclidean metric in joint space is the default in most implementations (e.g., OMPL).
- For **point cloud registration** (e.g., ICP algorithm), the Euclidean metric is used for nearest-neighbor search, typically accelerated with KD-trees.

---

## Coordinate Frame Transformations in Euclidean Space

Robotics fundamentally involves relating quantities expressed in different coordinate frames. A rigid body transformation in 3D Euclidean space consists of a rotation $R \in SO(3)$ and a translation $\mathbf{t} \in \mathbb{R}^3$.

### Transforming a Point Between Frames

Given a point $\mathbf{p}^B$ expressed in frame $\{B\}$, its coordinates in frame $\{A\}$ are:

$$
\mathbf{p}^A = R^A_B \, \mathbf{p}^B + \mathbf{t}^A_B
$$

Or equivalently using the homogeneous transformation:

$$
\begin{bmatrix} \mathbf{p}^A \\ 1 \end{bmatrix} = T^A_B \begin{bmatrix} \mathbf{p}^B \\ 1 \end{bmatrix}
$$

### Chaining Transforms

Transforms compose by multiplication. For frames $\{A\}$, $\{B\}$, $\{C\}$:

$$
T^A_C = T^A_B \cdot T^B_C
$$

This is the foundation of forward kinematics: the end-effector pose relative to the base is the product of all link transforms.

### Velocity Transformation

Velocities transform differently from points. The velocity of a point expressed in frame $\{A\}$ given its velocity in frame $\{B\}$ is:

$$
\mathbf{v}^A = R^A_B \, \mathbf{v}^B + \boldsymbol{\omega}^A_{AB} \times R^A_B \, \mathbf{p}^B
$$

where $\boldsymbol{\omega}^A_{AB}$ is the angular velocity of frame $\{B\}$ relative to frame $\{A\}$, expressed in $\{A\}$.

**Practical example:** A camera mounted on a robot arm sees an object at position $\mathbf{p}^{\text{cam}}$ in the camera frame. To command the arm to grasp it, we transform through the chain: $T^{\text{base}}_{\text{cam}} = T^{\text{base}}_{\text{ee}} \cdot T^{\text{ee}}_{\text{cam}}$, where $T^{\text{ee}}_{\text{cam}}$ is the hand-eye calibration result.

---

## Applications in Robotics

- **Navigation**: Euclidean space is used to calculate distances and plan paths for mobile robots navigating through their environment.
- **Manipulation**: Vectors and transformations in Euclidean space are essential for controlling robotic manipulators to reach and manipulate objects.
- **Computer Vision**: Euclidean geometry is used to analyze and interpret visual data, enabling robots to understand their surroundings.
- **Simultaneous Localization and Mapping (SLAM)**: Euclidean space provides the framework for mapping environments and localizing robots within those maps.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #mathematics WHERE contains(file.outlinks, [[Euclidean_Space]])
