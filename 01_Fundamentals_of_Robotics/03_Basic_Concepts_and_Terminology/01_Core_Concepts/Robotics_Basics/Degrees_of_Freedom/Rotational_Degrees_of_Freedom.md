---
title: Rotational Degrees of Freedom
description: Rotational Degrees of Freedom refer to the number of independent rotational motions a mechanical system can undergo, crucial for understanding and designing robotic systems with specific motion capabilities.
tags:
  - robotics
  - kinematics
  - degrees-of-freedom
  - rotational-motion
  - mechanism-design
  - engineering
  - glossary-term
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /rotational_degrees_of_freedom/
related:
  - "[[Kinematics]]"
  - "[[Degrees_of_Freedom]]"
  - "[[Mechanism_Design]]"
  - "[[Euler_Angles]]"
  - "[[Quaternions]]"
  - "[[Robot_Design]]"
  - "[[Gimbal_Lock]]"
---

# Rotational Degrees of Freedom

**Rotational Degrees of Freedom** refer to the number of independent rotational motions a mechanical system can undergo. Understanding rotational degrees of freedom is crucial for designing robotic systems with specific motion capabilities, such as manipulators and mobile robots. These degrees of freedom determine how a system can rotate around different axes, affecting its ability to perform tasks that require precise orientation and movement.

---

## Key Concepts

### Rotational Motion

Rotational motion involves movement around an axis, described by angles and angular velocities. In robotics, rotational motion is essential for tasks such as grasping, manipulating objects, and navigating environments.

### Euler Angles

Euler angles are a way to represent the orientation of a rigid body using three angles, typically denoted as roll, pitch, and yaw. They are commonly used to describe rotational degrees of freedom in robotics.

### Quaternions

Quaternions provide an alternative to Euler angles for representing rotations in three-dimensional space. They avoid issues like gimbal lock and are often used in robotics for their computational efficiency and stability.

### Gimbal Lock

Gimbal lock is a phenomenon where two rotational degrees of freedom align, causing a loss of one degree of freedom. This issue is particularly relevant in systems using Euler angles and can be avoided by using quaternions.

---

## Mathematical Formulation

### Euler Angles Representation

Euler angles represent rotations as a sequence of three rotations around specified axes. The rotation matrix for Euler angles $\phi$, $\theta$, $\psi$ is given by:

$$
R = R_z(\psi) R_y(\theta) R_x(\phi)
$$

where:
- $R_x(\phi)$ is the rotation matrix for roll.
- $R_y(\theta)$ is the rotation matrix for pitch.
- $R_z(\psi)$ is the rotation matrix for yaw.

### Quaternion Representation

A quaternion $q$ is represented as:

$$
q = w + xi + yj + zk
$$

where $w$ is the scalar part, and $x$, $y$, and $z$ are the vector parts. The rotation matrix corresponding to a quaternion $q$ is given by:

$$
R = \begin{bmatrix}
1 - 2(y^2 + z^2) & 2(xy - wz) & 2(xz + wy) \\
2(xy + wz) & 1 - 2(x^2 + z^2) & 2(yz - wx) \\
2(xz - wy) & 2(yz + wx) & 1 - 2(x^2 + y^2)
\end{bmatrix}
$$

### Example: Robotic Arm

Consider a robotic arm with three rotational joints. The rotational degrees of freedom allow the arm to position its end-effector in various orientations. Using Euler angles, the orientation of the end-effector can be described by:

$$
R = R_z(\psi) R_y(\theta) R_x(\phi)
$$

This allows the arm to perform tasks such as grasping objects at different angles and orientations.

---

---

## Gimbal Lock: Numerical Example

Gimbal lock occurs when two rotation axes align, collapsing three rotational DoF into two. This is a singularity of the Euler angle representation, not a physical limitation of the mechanism.

### Setup

Consider the ZYX Euler angle convention (yaw-pitch-roll). The individual rotation matrices are:

$$
R_z(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}, \quad
R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}, \quad
R_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}
$$

### The Singularity at $\theta = 90°$

When $\theta = \pi/2$ (pitch = 90 degrees), $\cos\theta = 0$ and $\sin\theta = 1$:

$$
R = R_z(\psi) R_y(\pi/2) R_x(\phi) = \begin{bmatrix} 0 & \sin(\phi - \psi) & \cos(\phi - \psi) \\ 0 & -\cos(\phi - \psi) & \sin(\phi - \psi) \\ -1 & 0 & 0 \end{bmatrix}
$$

Notice that $\psi$ and $\phi$ appear only as the difference $(\phi - \psi)$. This means yaw and roll have become coupled -- changing $\psi$ by $+5°$ and $\phi$ by $+5°$ produces no net rotation. The system has lost one independent DoF.

### Angular Velocity at the Singularity

The angular velocity in body frame is:

$$
\boldsymbol{\omega} = \begin{bmatrix} \dot{\phi} - \dot{\psi}\sin\theta \\ \dot{\theta}\cos\phi + \dot{\psi}\cos\theta\sin\phi \\ -\dot{\theta}\sin\phi + \dot{\psi}\cos\theta\cos\phi \end{bmatrix}
$$

At $\theta = \pi/2$, this becomes:

$$
\boldsymbol{\omega} = \begin{bmatrix} \dot{\phi} - \dot{\psi} \\ 0 \\ 0 \end{bmatrix}
$$

regardless of $\dot\theta$. Only the combination $(\dot\phi - \dot\psi)$ affects the angular velocity about the x-axis; the y and z components are locked to zero. The Jacobian mapping $(\dot\phi, \dot\theta, \dot\psi) \to \boldsymbol{\omega}$ drops rank from 3 to 1.

> **Practitioner's note:** Gimbal lock is the reason roboticists switched to quaternions for orientation control in spacecraft (Apollo program), game engines, and modern robot controllers. If your application uses Euler angles, you must add singularity detection and switching logic, which is error-prone.

---

## Axis-Angle Representation and Rodrigues' Formula

Any rotation in 3D can be described by a unit axis $\hat{\mathbf{k}} = (k_x, k_y, k_z)^T$ and an angle $\theta$:

$$
R(\hat{\mathbf{k}}, \theta) = I + \sin\theta \, [\hat{\mathbf{k}}]_\times + (1 - \cos\theta) \, [\hat{\mathbf{k}}]_\times^2
$$

This is **Rodrigues' rotation formula**, where $[\hat{\mathbf{k}}]_\times$ is the skew-symmetric matrix:

$$
[\hat{\mathbf{k}}]_\times = \begin{bmatrix} 0 & -k_z & k_y \\ k_z & 0 & -k_x \\ -k_y & k_x & 0 \end{bmatrix}
$$

### Extracting Axis-Angle from a Rotation Matrix

Given a rotation matrix $R$:

$$
\theta = \arccos\left(\frac{\text{tr}(R) - 1}{2}\right)
$$

$$
\hat{\mathbf{k}} = \frac{1}{2\sin\theta} \begin{bmatrix} R_{32} - R_{23} \\ R_{13} - R_{31} \\ R_{21} - R_{12} \end{bmatrix}
$$

This is undefined when $\theta = 0$ (identity -- any axis works) or $\theta = \pi$ (requires special handling via the symmetric part of $R$).

### The Matrix Exponential Connection

Rodrigues' formula is equivalent to the matrix exponential:

$$
R = e^{[\hat{\mathbf{k}}]_\times \theta} = e^{[\boldsymbol{\omega}]_\times}
$$

where $\boldsymbol{\omega} = \theta \hat{\mathbf{k}}$ is the rotation vector. This is the foundation of the **exponential coordinates** used in modern robotics (Lie group / Lie algebra formulation).

---

## Rotation Matrix to Quaternion Conversion

Given a rotation matrix $R$, the corresponding unit quaternion $q = (w, x, y, z)$ can be computed as follows. First, compute $w$ from the trace:

$$
w = \frac{1}{2}\sqrt{1 + R_{11} + R_{22} + R_{33}}
$$

Then, if $w \neq 0$:

$$
x = \frac{R_{32} - R_{23}}{4w}, \quad y = \frac{R_{13} - R_{31}}{4w}, \quad z = \frac{R_{21} - R_{12}}{4w}
$$

When $w \approx 0$ (rotation near 180 degrees), use the Shepperd method to avoid numerical instability: find the largest diagonal element of $R$ and compute the corresponding quaternion component first.

### Quaternion to Rotation Matrix

The inverse conversion (quaternion to rotation matrix) is given in the Mathematical Formulation section above.

### Quaternion Multiplication (Composition of Rotations)

The composition of two rotations $q_1$ followed by $q_2$ is:

$$
q_1 \otimes q_2 = \begin{bmatrix} w_1 w_2 - x_1 x_2 - y_1 y_2 - z_1 z_2 \\ w_1 x_2 + x_1 w_2 + y_1 z_2 - z_1 y_2 \\ w_1 y_2 - x_1 z_2 + y_1 w_2 + z_1 x_2 \\ w_1 z_2 + x_1 y_2 - y_1 x_2 + z_1 w_2 \end{bmatrix}
$$

---

## SLERP: Spherical Linear Interpolation

**SLERP** (Spherical Linear Interpolation) provides the smoothest possible interpolation between two orientations represented as unit quaternions $q_0$ and $q_1$:

$$
\text{SLERP}(q_0, q_1, t) = \frac{\sin((1-t)\Omega)}{\sin\Omega} q_0 + \frac{\sin(t\Omega)}{\sin\Omega} q_1
$$

where $t \in [0, 1]$ is the interpolation parameter and $\Omega = \arccos(q_0 \cdot q_1)$ is the angle between the two quaternions.

### Properties of SLERP

- **Constant angular velocity**: The interpolated rotation proceeds at a constant rate, unlike linear interpolation of Euler angles.
- **Shortest path**: SLERP always follows the shortest great-arc path on the unit quaternion sphere $S^3$. To ensure this, check that $q_0 \cdot q_1 \geq 0$; if not, negate one quaternion (since $q$ and $-q$ represent the same rotation).
- **Singularity-free**: Unlike Euler angle interpolation, SLERP never encounters gimbal lock.

### When $\Omega \approx 0$ (Nearly Identical Orientations)

When $q_0 \approx q_1$, $\sin\Omega \approx 0$ and the formula becomes numerically unstable. Fall back to normalized linear interpolation (NLERP):

$$
\text{NLERP}(q_0, q_1, t) = \frac{(1-t) q_0 + t \, q_1}{\| (1-t) q_0 + t \, q_1 \|}
$$

NLERP does not have constant angular velocity but is a good approximation for small $\Omega$.

> **Practitioner's tip:** Most modern robot controllers (ROS 2, KUKA Sunrise, ABB RobotStudio) use quaternion SLERP internally for orientation interpolation. When programming robot motion in task space, always interpolate orientations with SLERP rather than interpolating Euler angles independently -- the latter produces erratic motion and can pass through gimbal lock configurations.

---

## Applications in Robotics

- **Robotic Manipulators**: Rotational degrees of freedom enable manipulators to orient and position their end-effectors precisely for tasks such as assembly and welding.
- **Mobile Robots**: Allow mobile robots to navigate and orient themselves in their environment, essential for tasks like obstacle avoidance and path planning.
- **Aerospace**: Used in the design of aircraft and spacecraft control systems to manage orientation and stability during flight.
- **Virtual Reality**: Rotational degrees of freedom are crucial for tracking head and body movements in virtual reality systems, providing an immersive experience.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Rotational_Degrees_of_Freedom]])
