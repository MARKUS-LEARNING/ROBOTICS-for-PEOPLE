---
title: Pose Representation
description: Pose Representation is the mathematical description of the position and orientation of a robotic system in space, essential for navigation, manipulation, and interaction with the environment.
tags:
  - robotics
  - kinematics
  - pose-representation
  - spatial-representation
  - robot-design
  - engineering
  - glossary-term
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /pose_representation/
related:
  - "[[Kinematics]]"
  - "[[Homogeneous_Transformation]]"
  - "[[Euler_Angles]]"
  - "[[Quaternions]]"
  - "[[Transformation_Matrices]]"
  - "[[Robot_Design]]"
---

# Pose Representation

**Pose Representation** is the mathematical description of the position and orientation of a robotic system in space. It is essential for navigation, manipulation, and interaction with the environment. Pose representation allows robots to understand their spatial relationship with objects and other entities in their surroundings, enabling precise control and task execution.

---
<img src="https://i0.wp.com/circuitcellar.com/wp-content/uploads/2020/02/355_Torrico_Figure_6.png?w=604&ssl=1"></img>
<font size=1>*source: https://circuitcellar.com/research-design-hub/build-a-4-dof-robotic-arm-part-2/*</font>
---

## Key Concepts

### Position

Position refers to the location of a point in space, typically described using [[Cartesian_Space|Cartesian coordinates]] in a three-dimensional space. It represents the translational component of the pose.

### Orientation

Orientation describes the rotational aspect of the pose, specifying how an object or robot is rotated relative to a reference frame. It is often represented using Euler angles, quaternions, or rotation matrices.

### Homogeneous Transformation

Homogeneous transformation matrices are used to represent both the position and orientation of a robot in a single mathematical framework. They allow for efficient computation of transformations between different coordinate frames.

### Euler Angles

Euler angles represent orientation using three angles: roll, pitch, and yaw. While intuitive, they can suffer from issues like gimbal lock, which limits their use in certain applications.

### Quaternions

Quaternions provide a stable and efficient way to represent orientation in three-dimensional space. They avoid the issues associated with Euler angles and are widely used in robotics for their computational advantages.

---

## Mathematical Formulation

### Homogeneous Transformation Matrix

A homogeneous transformation matrix $T$ combines both position and orientation into a single matrix representation:

$$
T = \begin{bmatrix}
R & \mathbf{p} \\
\mathbf{0} & 1
\end{bmatrix}
$$

where:
- $R$ is the rotation matrix (3x3).
- $\mathbf{p}$ is the position vector (3x1).
- $\mathbf{0}$ is a zero vector (1x3).

### Euler Angles

Euler angles represent orientation as a sequence of rotations around specified axes. The rotation matrix $R$ for Euler angles $(\phi, \theta, \psi)$ is given by:

$$
R = R_z(\psi) R_y(\theta) R_x(\phi)
$$

where $R_x(\phi)$, $R_y(\theta)$, and $R_z(\psi)$ are the rotation matrices for roll, pitch, and yaw, respectively.

### Quaternions

A quaternion $q$ is represented as:

$$
q = w + xi + yj + zk
$$

where $w$ is the scalar part, and $x$, $y$, and $z$ are the vector parts. The corresponding rotation matrix $R$ for a quaternion is:

$$
R = \begin{bmatrix}
1 - 2(y^2 + z^2) & 2(xy - wz) & 2(xz + wy) \\
2(xy + wz) & 1 - 2(x^2 + z^2) & 2(yz - wx) \\
2(xz - wy) & 2(yz + wx) & 1 - 2(x^2 + y^2)
\end{bmatrix}
$$

### Pose Composition and Inverse

**Pose Composition**: When two poses are chained (e.g., frame $A$ relative to world, frame $B$ relative to $A$), the resulting pose is computed by matrix multiplication:

$$
T_{WB} = T_{WA} \cdot T_{AB}
$$

**Pose Inverse**: The inverse of a homogeneous transformation recovers the reverse transformation:

$$
T^{-1} = \begin{bmatrix}
R^T & -R^T \mathbf{p} \\
\mathbf{0} & 1
\end{bmatrix}
$$

This avoids a full $4 \times 4$ matrix inversion by exploiting the orthogonality of rotation matrices ($R^{-1} = R^T$).

### SLERP: Quaternion Interpolation

**Spherical Linear Interpolation (SLERP)** produces smooth, constant-angular-velocity interpolation between two orientations represented as unit quaternions $q_0$ and $q_1$:

$$
\text{SLERP}(q_0, q_1, t) = \frac{\sin((1-t)\Omega)}{\sin \Omega} q_0 + \frac{\sin(t\Omega)}{\sin \Omega} q_1
$$

where $\Omega = \cos^{-1}(q_0 \cdot q_1)$ is the angle between the two quaternions and $t \in [0, 1]$ is the interpolation parameter.

**Practical notes for SLERP implementation:**
- Always check the sign of $q_0 \cdot q_1$. If negative, negate one quaternion before interpolating to ensure the shortest-path rotation.
- When $\Omega \approx 0$ (nearly identical orientations), fall back to normalized linear interpolation (NLERP) to avoid division by zero.
- SLERP is standard in robot trajectory generation. For a pick-and-place motion, you typically SLERP the orientation while linearly interpolating position, then combine into the full pose.

---

## Representation Comparison

Choosing the right orientation representation is critical in practice. The table below compares the three main approaches:

| Property | Euler Angles | Rotation Matrix | Quaternion |
|---|---|---|---|
| **Parameters** | 3 (e.g., roll, pitch, yaw) | 9 (with 6 constraints) | 4 (with 1 constraint) |
| **Minimal representation** | Yes | No | No (but compact) |
| **Singularities** | Yes (gimbal lock) | No | No |
| **Composition** | Sequential rotations (order-dependent) | Matrix multiply $R_1 R_2$ | Quaternion multiply $q_1 \otimes q_2$ |
| **Interpolation** | Poor (discontinuous) | Non-trivial | Excellent (SLERP) |
| **Computational cost** | Low storage, high composition cost | High storage, moderate composition | Low storage, low composition |
| **Human readability** | Excellent | Poor | Poor |
| **Typical use** | HMI displays, joystick input | Homogeneous transforms, DH convention | Motion planning, AHRS/IMU output, game engines |

**Practitioner guidance:**
- Use **Euler angles** only for user interfaces and debugging displays. Never use them internally for computation in 3D applications.
- Use **rotation matrices** when composing transforms in kinematic chains (DH parameters naturally produce these).
- Use **quaternions** for trajectory interpolation, state estimation (EKF/UKF), and any application where orientation must be integrated over time.
- The ROS `tf2` library internally uses quaternions. The URDF format specifies orientation as quaternion $(x, y, z, w)$.

---

### Example: Mobile Robot

Consider a mobile robot navigating in a 2D plane. Its pose can be represented using a homogeneous transformation matrix:

$$
T = \begin{bmatrix}
\cos(\theta) & -\sin(\theta) & x \\
\sin(\theta) & \cos(\theta) & y \\
0 & 0 & 1
\end{bmatrix}
$$

where $(x, y)$ is the position, and $\theta$ is the orientation (yaw angle). This representation allows the robot to calculate its position and orientation relative to a global reference frame, enabling precise navigation and interaction with the environment.

---

## Applications in Robotics

- **Navigation**: Pose representation is crucial for mobile robots to navigate through their environment, avoiding obstacles and reaching target locations.
- **Manipulation**: Enables robotic manipulators to precisely position and orient their end-effectors for tasks such as grasping and assembly.
- **Localization**: Allows robots to determine their position and orientation within a map, essential for autonomous operation.
- **Simultaneous Localization and Mapping (SLAM)**: Pose representation is fundamental for building and updating maps while simultaneously localizing the robot within the environment.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Pose_Representation]])
