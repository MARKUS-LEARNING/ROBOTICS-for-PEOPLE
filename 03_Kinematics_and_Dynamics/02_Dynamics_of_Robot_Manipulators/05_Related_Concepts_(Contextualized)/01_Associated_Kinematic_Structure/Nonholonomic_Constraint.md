---
title: Nonholonomic Constraint
description: A nonholonomic constraint is a restriction on the motion of a system that cannot be expressed as a function of the system's coordinates alone, often arising in wheeled mobile robots.
tags:
  - kinematics
  - mobile-robot
  - wheeled-robot
  - locomotion
  - nonholonomic
  - glossary-term
  - mechanism
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /nonholonomic_constraint/
related:
  - "[[Differential_Drive]]"
  - "[[Wheeled_Mobile_Robots]]"
  - "[[Mobile_Robots]]"
  - "[[Kinematics]]"
  - "[[Locomotion]]"
  - "[[Instantaneous_Center_of_Rotation_(ICR)]]"
---

# Nonholonomic Constraint

A **nonholonomic constraint** is a restriction on the motion of a system that cannot be expressed as a function of the system's coordinates alone. These constraints often arise in wheeled mobile robots, such as those using a differential drive system, where the constraint limits the robot's ability to move sideways instantaneously. Understanding nonholonomic constraints is crucial for designing and controlling robotic systems that must navigate within these limitations.

---

## Types of Constraints

1. **Holonomic Constraints**: These constraints can be expressed solely in terms of the system's coordinates and can be integrated to obtain relationships between the coordinates. Examples include a pendulum's length or a rigid body's fixed distance between two points.

2. **Nonholonomic Constraints**: These constraints involve velocity-dependent relationships that cannot be integrated to yield a simple coordinate relationship. They often arise from rolling or sliding contact, as seen in wheeled robots.

---

## Mathematical Representation

### Nonholonomic Constraint in Wheeled Robots

For a differential drive robot, the nonholonomic constraint can be expressed as:

$$
v_y = 0
$$

where $v_y$ is the velocity component along the axis perpendicular to the direction of the drive wheels (typically the $Y_R$ axis in the robot's local frame). This constraint implies that the robot cannot move sideways instantaneously.

### Pfaffian Form

Nonholonomic constraints are often represented in Pfaffian form:

$$
A(q) \cdot \dot{q} = 0
$$

where $A(q)$ is a matrix that depends on the generalized coordinates $q$, and $\dot{q}$ is the vector of generalized velocities. For a differential drive robot, this form can be written as:

$$
\begin{bmatrix}
0 & 1 & 0
\end{bmatrix}
\cdot
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}
\end{bmatrix}
= 0
$$

where $\dot{x}$, $\dot{y}$, and $\dot{\theta}$ are the linear and angular velocities of the robot in its local frame.

---

## Impact on Robotics

- **Motion Planning**: Nonholonomic constraints must be considered when planning the motion of mobile robots, as they limit the possible trajectories the robot can take.
- **Control Systems**: Designing control systems for robots with nonholonomic constraints requires advanced techniques, such as feedback linearization or model predictive control, to manage the constraints effectively.
- **Kinematic Analysis**: Understanding nonholonomic constraints is essential for the kinematic analysis of robotic systems, helping to describe and predict the robot's motion accurately.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #kinematics OR #mobile-robot WHERE contains(file.outlinks, [[Nonholonomic_Constraint]])
