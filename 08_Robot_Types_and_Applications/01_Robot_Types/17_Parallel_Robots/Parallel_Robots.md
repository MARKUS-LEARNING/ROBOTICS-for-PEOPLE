---
title: Parallel Robots
description: Parallel robots are robotic systems that utilize parallel kinematic chains to connect a base platform to a moving platform, providing high stiffness, precision, and load-bearing capabilities.
tags:
  - robotics
  - manipulator
  - parallel-robot
  - kinematics
  - dynamics
  - mechanism
  - glossary-term
  - actuator
  - motion
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-05-02
permalink: /parallel_robots/
related:
  - "[[Stewart_Platform]]"
  - "[[Kinematics]]"
  - "[[Dynamics]]"
  - "[[Actuator]]"
  - "[[Mechanisms_and_Actuation]]"
  - "[[Control_Systems]]"
  - "[[Robot_Design]]"
  - "[[Manipulator_Arm]]"
---

# Parallel Robots

**Parallel robots** are robotic systems that utilize parallel kinematic chains to connect a base platform to a moving platform, providing high stiffness, precision, and load-bearing capabilities. Unlike serial robots, where each joint supports all subsequent joints, parallel robots distribute loads across multiple kinematic chains, resulting in enhanced structural rigidity and accuracy. These robots are widely used in applications requiring precise positioning and high dynamic performance.

---

## Types of Parallel Robots

1. **Stewart Platform**: A common type of parallel robot featuring six prismatic joints connecting a base platform to a moving platform, providing six degrees of freedom. It is used in applications such as flight simulators and robotic manufacturing.
   <br>

2. **Delta Robot**: Utilizes three parallelogram-based kinematic chains to connect the base to the moving platform. Known for its high-speed and high-precision capabilities, it is often used in pick-and-place operations and assembly tasks.
   <br>

3. **Hexapod**: Features six legs connecting the base to the moving platform, often used in large-scale applications like telescopes and antenna positioning systems.
   <br>

4. **Tripod**: A simpler version of parallel robots with three kinematic chains, often used in educational and research settings for studying parallel robot kinematics.
   <br>

---

## Kinematic Model

The kinematics of parallel robots involve solving for the positions and orientations of the moving platform based on the lengths of the actuators in the kinematic chains.

### Forward Kinematics

Forward kinematics involves calculating the position and orientation of the moving platform given the actuator lengths. This is typically more complex than for serial robots and often requires numerical methods due to the nonlinear relationships involved.

For a general parallel robot, the forward kinematics can be expressed as:

$$
\mathbf{T} = f(\mathbf{l})
$$

where $\mathbf{T}$ is the transformation matrix representing the position and orientation of the moving platform, and $\mathbf{l}$ is the vector of actuator lengths.

### Inverse Kinematics

Inverse kinematics involves determining the required actuator lengths to achieve a desired position and orientation of the moving platform. This can often be solved analytically for simpler parallel robots or using numerical methods for more complex configurations.

The inverse kinematics equation can be written as:

$$
\mathbf{l} = g(\mathbf{T})
$$

where $\mathbf{l}$ is the vector of actuator lengths, and $\mathbf{T}$ is the desired transformation matrix.

---

## Dynamics

The dynamic behavior of parallel robots is influenced by the masses of the platforms, the inertia of the moving platform, and the forces exerted by the actuators. The dynamics are described using equations of motion that account for these factors.

### Equations of Motion

The equations of motion for parallel robots can be derived using Newton-Euler formulations or Lagrange's equations. These equations are used to analyze the platform's response to external forces and to design control systems for precise motion control.

The dynamic equation using Lagrange's formulation is:

$$
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{q}} \right) - \frac{\partial L}{\partial q} = Q
$$

where $L = T - V$ is the Lagrangian, $T$ is the kinetic energy, $V$ is the potential energy, $q$ represents the generalized coordinates (often the actuator lengths), and $Q$ represents the generalized forces.

---

## Applications

- **Manufacturing**: Parallel robots are used in manufacturing for precise positioning and orientation of tools and components, especially in high-speed operations.
  <br>

- **Medical Devices**: Employed in medical devices for precise control of surgical instruments and patient positioning.
  <br>

- **Research and Development**: Utilized in research for studying parallel robot kinematics and dynamics, as well as developing new control algorithms.
  <br>

- **Simulation**: Used in flight simulators and other training devices to provide realistic motion cues.
  <br>

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #manipulator WHERE contains(file.outlinks, [[Parallel_Robots]])
