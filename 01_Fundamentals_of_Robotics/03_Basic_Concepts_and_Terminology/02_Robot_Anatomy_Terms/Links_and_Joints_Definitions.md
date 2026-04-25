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
  - manipulator-arm
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith
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

## Complete DH Parameter Table: PUMA 560

The PUMA 560 is the canonical 6R industrial robot used in textbooks. Its **standard DH parameters** are:

| Joint $i$ | $\theta_i$ (variable) | $d_i$ (mm) | $a_i$ (mm) | $\alpha_i$ (deg) |
|---|---|---|---|---|
| 1 | $\theta_1$ | 660 | 0 | -90 |
| 2 | $\theta_2$ | 0 | 432 | 0 |
| 3 | $\theta_3$ | 149 | -20 | 90 |
| 4 | $\theta_4$ | 433 | 0 | -90 |
| 5 | $\theta_5$ | 0 | 0 | 90 |
| 6 | $\theta_6$ | 56 | 0 | 0 |

The forward kinematics is computed as:

$$
T_0^6 = T_1(\theta_1) \cdot T_2(\theta_2) \cdot T_3(\theta_3) \cdot T_4(\theta_4) \cdot T_5(\theta_5) \cdot T_6(\theta_6)
$$

**Note:** Joints 4, 5, 6 form a **spherical wrist** -- their axes intersect at a single point (the wrist center), which enables kinematic decoupling for closed-form inverse kinematics.

---

## Modified DH Convention Comparison

Two DH conventions are widely used, and confusing them is a common source of bugs:

### Standard (Classic) DH Convention

- Frame $\{i\}$ is attached to the **distal** end of link $i$ (at joint $i+1$)
- Transform order: $T_i = \text{Rot}_z(\theta_i) \cdot \text{Trans}_z(d_i) \cdot \text{Trans}_x(a_i) \cdot \text{Rot}_x(\alpha_i)$
- Used by: Denavit & Hartenberg (1955), Craig's textbook, many European references

### Modified (Proximal) DH Convention

- Frame $\{i\}$ is attached to the **proximal** end of link $i$ (at joint $i$)
- Transform order: $T_i = \text{Rot}_x(\alpha_{i-1}) \cdot \text{Trans}_x(a_{i-1}) \cdot \text{Rot}_z(\theta_i) \cdot \text{Trans}_z(d_i)$
- Used by: Craig's "Introduction to Robotics" (later editions), Corke's Robotics Toolbox, many ROS implementations

| Aspect | Standard DH | Modified DH |
|---|---|---|
| Frame attachment | Distal end of link | Proximal end of link |
| Transform order | $R_z T_z T_x R_x$ | $R_x T_x R_z T_z$ |
| Base frame | Coincides with joint 1 | Separate base frame |
| Common in | Textbooks (Spong, Siciliano) | Software (Peter Corke toolbox, ROS) |

**Critical warning:** The two conventions produce **different** DH parameter values for the same robot. Always verify which convention a reference uses before applying its parameters. Mixing conventions is a frequent source of kinematic errors.

---

## Practical Joint Specification Parameters

When selecting or specifying joints for a robot design, the following parameters are critical:

### Key Joint Specifications

| Parameter | Definition | Typical Range (Industrial 6R) | Why It Matters |
|---|---|---|---|
| **Range of motion** | Angular travel from stop to stop | $\pm 170°$ to $\pm 360°$ | Determines workspace boundary |
| **Max velocity** | Peak angular velocity | 60 -- 400 deg/s | Limits cycle time |
| **Max torque (peak)** | Short-duration torque limit | 50 -- 1,000 Nm | Determines max payload |
| **Continuous torque** | Sustained torque (thermal limit) | 30 -- 500 Nm | Limits steady-state payload |
| **Repeatability** | Variation in returning to same position | $\pm 0.01$ -- $\pm 0.1$ mm | Directly affects application accuracy |
| **Backlash** | Angular dead zone in gear train | 0.3 -- 15 arcmin | Limits precision in reversal |
| **Torsional stiffness** | Resistance to angular deflection | 10 -- 500 Nm/arcmin | Affects positional accuracy under load |
| **Weight** | Mass of the joint assembly | 0.5 -- 30 kg | Affects upstream joint sizing |
| **Moment of inertia** | Rotational inertia of joint | 0.01 -- 5 kg-m$^2$ | Affects dynamic performance |

### Joint Specifications for Common Robot Joints

| Robot | Joint 2 (Shoulder) Range | Joint 2 Max Speed | Repeatability | Payload |
|---|---|---|---|---|
| UR5e | $\pm 360°$ | 180 deg/s | $\pm 0.03$ mm | 5 kg |
| KUKA iiwa 14 | $\pm 120°$ (J2) | 85 deg/s | $\pm 0.15$ mm | 14 kg |
| FANUC M-20iA | $+76° / -60°$ (J2) | 160 deg/s | $\pm 0.08$ mm | 20 kg |
| ABB IRB 6700 | $+85° / -65°$ (J2) | 150 deg/s | $\pm 0.05$ mm | 150 kg |
| Franka Emika Panda | $\pm 101°$ (J2) | 120 deg/s | $\pm 0.1$ mm | 3 kg |

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
```
