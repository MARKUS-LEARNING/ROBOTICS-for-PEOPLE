---
title: Links and Joints Definitions
description: Links and Joints Definitions are fundamental concepts in robotics, describing the structural components and connections that enable motion in mechanical systems.
tags:
  - robotics
  - kinematics
  - links
  - joints
  - mechanism-design
  - engineering
  - glossary-term
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /links_and_joints_definitions/
related:
  - "[[Kinematics]]"
  - "[[Mechanism_Design]]"
  - "[[Degrees_of_Freedom]]"
  - "[[Revolute_Joint]]"
  - "[[Prismatic_Joint]]"
  - "[[Robot_Design]]"
  - "[[Denavit-Hartenberg_Parameters]]"
---

# Links and Joints Definitions

**Links and Joints Definitions** are fundamental concepts in robotics, describing the structural components and connections that enable motion in mechanical systems. Links are the rigid bodies that make up the structure of a robotic system, while joints are the connections between links that allow relative motion. Understanding these components is crucial for designing and analyzing robotic mechanisms.

---
![image](https://github.com/user-attachments/assets/ffe3f005-3e6e-4cfc-b12e-bd6e0ebafa9c)


<font size=1>*source: https://www.mdpi.com/1996-1073/14/20/6690*</font>
---

## Key Concepts

### Links

Links are the rigid components that form the structure of a robotic system. They connect to other links through joints, creating a kinematic chain. Links can vary in shape and size, depending on the design requirements of the robotic system.

### Joints

Joints are the connections between links that allow relative motion. They define the types of movements a robotic system can perform and are classified based on the number of degrees of freedom they provide.

### Degrees of Freedom

Degrees of freedom (DoF) refer to the number of independent parameters required to define the configuration of a mechanical system. Joints contribute to the degrees of freedom by allowing motion in specific directions.

### Types of Joints

- **Revolute Joint**: Allows rotation around a single axis, providing one degree of freedom. Commonly used in robotic arms for rotational motion.
- **Prismatic Joint**: Allows linear motion along a single axis, providing one degree of freedom. Used for extending or retracting components.
- **Universal Joint**: Allows rotation around two axes, providing two degrees of freedom. Used in applications requiring flexible motion.
- **Spherical Joint**: Allows rotation around three axes, providing three degrees of freedom. Used in applications requiring omnidirectional rotation.

---

## Mathematical Formulation

### Denavit-Hartenberg Parameters

The Denavit-Hartenberg (DH) parameters are used to describe the kinematic relationships between links and joints in a robotic system. The transformation matrix for a joint can be represented as:

$$
T_i = \begin{bmatrix}
\cos(\theta_i) & -\sin(\theta_i)\cos(\alpha_i) & \sin(\theta_i)\sin(\alpha_i) & a_i\cos(\theta_i) \\
\sin(\theta_i) & \cos(\theta_i)\cos(\alpha_i) & -\cos(\theta_i)\sin(\alpha_i) & a_i\sin(\theta_i) \\
0 & \sin(\alpha_i) & \cos(\alpha_i) & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

where:
- $\theta_i$ is the joint angle.
- $d_i$ is the joint offset.
- $a_i$ is the link length.
- $\alpha_i$ is the link twist.

### Example: Robotic Arm

Consider a robotic arm with three revolute joints. The kinematic chain can be described using DH parameters for each joint. The forward kinematics equation for the arm is:

$$
T = T_1 \cdot T_2 \cdot T_3
$$

where $T_i$ is the transformation matrix for the $i$-th joint. This equation allows for the calculation of the end-effector position and orientation based on the joint angles.

---

## Applications in Robotics

- **Robotic Manipulators**: Understanding links and joints is essential for designing manipulators with specific motion capabilities, enabling tasks such as grasping and assembly.
- **Mobile Robots**: Links and joints define the structure and mobility of mobile robots, allowing them to navigate and interact with their environment.
- **Industrial Automation**: Mechanisms in automated machinery rely on links and joints to perform repetitive tasks with precision and efficiency.
- **Prosthetics**: Designing prosthetic devices involves creating links and joints that mimic human motion, providing functionality and comfort.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #kinematics WHERE contains(file.outlinks, [[Links_and_Joints_Definitions]])
