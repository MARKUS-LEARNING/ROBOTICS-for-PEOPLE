---
title: Stewart Platform
description: The Stewart Platform is a type of parallel manipulator consisting of six prismatic joints connecting a base platform to a moving platform, providing six degrees of freedom.
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
permalink: /stewart_platform/
related:
  - "[[Parallel_Robots]]"
  - "[[Kinematics]]"
  - "[[Dynamics]]"
  - "[[Actuator]]"
  - "[[Mechanisms_and_Actuation]]"
  - "[[Control_Systems]]"
  - "[[Robot_Design]]"
  - "[[Manipulator_Arm]]"
---

# Stewart Platform

The **Stewart Platform** is a type of parallel manipulator consisting of six prismatic joints connecting a base platform to a moving platform, providing six degrees of freedom. It is widely used in applications requiring precise positioning and orientation, such as flight simulators, robotic manufacturing, and medical devices. The platform's design allows for high stiffness, accuracy, and load-bearing capacity.

---
![stewart_platform](https://github.com/user-attachments/assets/a6ff99b8-074e-4f38-9792-35201832cd3c)
<font size=1>*source: https://free3d.com/3d-model/hexapod-platform-9826.html*</font>
---

## Components of the Stewart Platform

1. **Base Platform**: The fixed base to which the six actuators are attached. It provides the structural support for the entire system.
   <br>

2. **Moving Platform**: The platform that is manipulated by the actuators. It can move in six degrees of freedom relative to the base platform.
   <br>

3. **Actuators**: Six linear actuators (prismatic joints) that connect the base platform to the moving platform. These actuators can extend or retract to adjust the position and orientation of the moving platform.
   <br>

4. **Joints**: Each actuator is connected to the platforms via spherical joints, allowing for rotational movement in multiple axes.
   <br>

---

## Kinematic Model

The kinematics of the Stewart Platform can be described using forward and inverse kinematic equations. These equations relate the lengths of the actuators to the position and orientation of the moving platform.

### Forward Kinematics

The forward kinematics problem involves calculating the position and orientation of the moving platform given the lengths of the six actuators. This is typically solved using iterative numerical methods due to the complexity of the equations.

### Inverse Kinematics

The inverse kinematics problem involves determining the required actuator lengths to achieve a desired position and orientation of the moving platform. This can be solved analytically or using numerical methods.

---

## Dynamics

The dynamic behavior of the Stewart Platform is influenced by the masses of the platforms, the inertia of the moving platform, and the forces exerted by the actuators. The dynamics are described using equations of motion that account for these factors.

### Equations of Motion

The equations of motion for the Stewart Platform can be derived using Newton-Euler formulations or Lagrange's equations. These equations are used to analyze the platform's response to external forces and to design control systems for precise motion control.

---

## Applications

- **Flight Simulators**: The Stewart Platform is commonly used in flight simulators to provide realistic motion cues to pilots.
  <br>

- **Robotic Manufacturing**: Used in manufacturing for precise positioning and orientation of tools and components.
  <br>

- **Medical Devices**: Employed in medical devices for precise control of surgical instruments and patient positioning.
  <br>

- **Research and Development**: Utilized in research for studying parallel robot kinematics and dynamics.
  <br>

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #manipulator WHERE contains(file.outlinks, [[Stewart_Platform]])
