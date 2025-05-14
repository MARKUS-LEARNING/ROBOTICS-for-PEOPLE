---
title: DH Parameters
description: DH Parameters (Denavit-Hartenberg Parameters) are a standardized method for representing the kinematic structure of robotic manipulators, essential for deriving forward and inverse kinematics.
tags:
  - robotics
  - kinematics
  - dynamics
  - dh-parameters
  - robot-design
  - engineering
  - glossary-term
  - mechanism
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /dh_parameters/
related:
  - "[[Kinematics]]"
  - "[[Dynamics]]"
  - "[[Forward_Kinematics]]"
  - "[[Inverse_Kinematics]]"
  - "[[Robot_Design]]"
  - "[[Homogeneous_Transformation]]"
  - "[[Jacobian_Matrix]]"
---

# DH Parameters

**DH Parameters** (Denavit-Hartenberg Parameters) are a standardized method for representing the kinematic structure of robotic manipulators, essential for deriving forward and inverse kinematics. They provide a systematic way to describe the relative positions and orientations of adjacent links in a robotic arm, facilitating the analysis and control of robotic systems.

---
![image](https://github.com/user-attachments/assets/3c2965c0-1030-4560-b69e-f293015a9ac6)



<font size=1>*source: https://link.springer.com/chapter/10.1007/978-3-031-04870-8_1</font>
---

## Definition

DH parameters define the relationship between two consecutive joint frames in a robotic manipulator using four parameters:

1. **$a_i$ (Link Length)**: The distance between the $z_i$ and $z_{i-1}$ axes measured along the $x_i$ axis.
2. **$\alpha_i$ (Link Twist)**: The angle between the $z_i$ and $z_{i-1}$ axes measured about the $x_i$ axis.
3. **$d_i$ (Link Offset)**: The distance between the $x_i$ and $x_{i-1}$ axes measured along the $z_{i-1}$ axis.
4. **$\theta_i$ (Joint Angle)**: The angle between the $x_i$ and $x_{i-1}$ axes measured about the $z_{i-1}$ axis.

---

## Homogeneous Transformation Matrix

The DH parameters are used to construct the homogeneous transformation matrix ${}^{i-1}T_i$ that relates the coordinate frame of link $i$ to the coordinate frame of link $i-1$:

$$
^{i-1}T_i =
\begin{bmatrix}
\cos(\theta_i) & -\sin(\theta_i) \cos(\alpha_i) & \sin(\theta_i) \sin(\alpha_i) & a_i \cos(\theta_i) \\
\sin(\theta_i) & \cos(\theta_i) \cos(\alpha_i) & -\cos(\theta_i) \sin(\alpha_i) & a_i \sin(\theta_i) \\
0 & \sin(\alpha_i) & \cos(\alpha_i) & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

This matrix encapsulates the rotation and translation between two consecutive joint frames.

---

## Applications in Robotics

- **Forward Kinematics**: DH parameters are used to compute the forward kinematics of a robotic manipulator, determining the position and orientation of the end-effector given the joint angles.
- **Inverse Kinematics**: They are essential for solving inverse kinematics problems, where the goal is to find the joint angles that achieve a desired end-effector position and orientation.
- **Robot Design**: DH parameters are used in the design and simulation of robotic manipulators, helping engineers to model and analyze the kinematic behavior of robotic systems.
- **Control Systems**: Understanding DH parameters is crucial for designing control algorithms that can accurately manipulate the robot's motion.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[DH_Parameters]])
