---
title: Degrees of Freedom (DoF)
description: Defines Degrees of Freedom (DoF) as the number of independent parameters required to specify the configuration of a mechanism or body.
tags:
  - glossary-term
  - kinematics
  - configuration-space
  - mobility
  - robot-design
  - manipulator-arm
  - mobile-robot
  - legged-robot
  - parallel-robot
layout: default
category: robotics
author: Jordan_Smith_and_le_Chat
date: 2025-04-27
permalink: /degrees_of_freedom/
related:
  - "[[Kinematic_Chains]]"
  - "[[Actuator]]"
  - "[[Joint_Kinematics]]"
  - "[[Workspace]]"
  - "[[Singularities]]"
  - "[[Manipulator_Arm_Types]]"
  - "[[Mobile_Robots]]"
  - "[[Legged_Robots]]"
  - "[[Parallel_Mechanisms_and_Robots]]"
  - "[[Grübler's_Formula]]"
---

# Degrees of Freedom (DoF)

**Degrees of Freedom (DoF)** refers to the minimum number of independent parameters required to completely specify the configuration (position and orientation) of a mechanism or a body in space. It quantifies the independent ways a system can move, providing insight into its mobility and control complexity. Understanding DoF is crucial for designing robots that can effectively interact with their environment.

---
![image](https://github.com/user-attachments/assets/ceac21fd-7e77-487c-bf76-0c78e2410209)

<font size=1>*source: https://medium.com/@tomtenner/the-kutzbach-criterion-e1592ef88bd1*</font>
---

## Degrees of Freedom (DoF) Calculation

The Degrees of Freedom (DoF) of a mechanical or robotic system can be calculated using the following formulas:

```
$dof = Σ (freedoms of points) - # of independent constraints$

$dof = Σ (freedoms of bodies) - # of independent constraints$
```



---
## DoF of Rigid Bodies

- **In 3D Space**: A free rigid body has 6 DoF: 3 translational (e.g., $x$, $y$, $z$) and 3 rotational (e.g., roll $\phi$, pitch $\theta$, yaw $\psi$). These degrees allow the body to move freely in space and orient itself in any direction.
  <br>

- **In 2D Plane**: A free rigid body moving on a plane has 3 DoF: 2 translational ($x$, $y$ position) and 1 rotational (orientation $\theta$). This configuration is common in planar robots and mechanisms constrained to two-dimensional motion.
  <br>

---

## DoF of Mechanisms

The DoF of a mechanism is the number of independent inputs needed to determine the configuration of all its parts. This concept is essential for analyzing and designing robotic systems:

- **Joint Contribution**: Joints connect links and constrain their relative motion. Each joint allows a certain number of DoF:
  - **Revolute/Prismatic Joints**: 1 DoF
  - **Cylindrical/Universal Joints**: 2 DoF
  - **Spherical/Planar Joints**: 3 DoF
  - A connection with no constraints can be modeled as a 6-DoF joint.
  <br>

- **Serial Manipulators**: For typical open [[Kinematic_Chains|kinematic chain]] manipulators, the total DoF is usually the sum of the DoF of its joints:
  - A general-purpose spatial manipulator typically requires 6 DoF to arbitrarily position and orient its end-effector in 3D space.
  - Manipulators with more than 6 DoF are called **kinematically redundant**. Redundancy increases dexterity, potentially allowing for [[Singularities]] and obstacle avoidance.
  - Manipulators with fewer than 6 DoF have restricted motion capabilities and operate within a lower-dimensional subspace of the full 6D space.
  - Tasks requiring less than 6 DoF (e.g., using symmetric tools) make a 6-DoF robot task-redundant.
  - Kinematic singularities are configurations where the manipulator locally loses one or more DoF in terms of end-effector motion.
  <br>

- **Mobile Robots (Planar)**:
  - The chassis of a mobile robot moving on a plane typically has 3 DoF ($x$, $y$, $\theta$). This refers to the chassis pose.
  - The wheels impose kinematic constraints (holonomic or nonholonomic) that affect the **differential degrees of freedom (DDOF)**, which is the number of independent velocity inputs the robot can control instantaneously.
  - An **omnidirectional** robot has 3 DDOF (its velocity in $x$, $y$, and $\theta$ can be controlled independently). A differential drive robot has 2 DDOF.
  - The **maneuverability** ($\delta M$) considers both mobility ($\delta m = \text{DDOF}$) and steerability ($\delta s$).
  <br>

- **Legged Robots**: DoF is often described by the number of actuated joints per leg (typically 2-3), plus the DoF of the main body (often 6 if considered a floating base).
  <br>

- **Parallel Robots**: These have closed kinematic loops. Their DoF is typically calculated using formulas like the Grübler-Kutzbach criterion, which considers the number of links, joints, and joint types. The DoF can potentially change depending on the configuration (posture).
  <br>

---

## Relevance in Robotics

- **Task Requirements**: The necessary DoF for a robot depends on the task. Simple tasks might require fewer than 6 DoF (e.g., vertical assembly). Using symmetric tools can also reduce the required DoF, making a 6-DoF robot task-redundant.
  <br>

- **Workspace**: The DoF defines the dimensionality of the robot's configuration space. The [[Workspace]] (reachable and dexterous) is the set of poses the robot can achieve within this space, limited by link geometry and joint limits.
  <br>

- **Singularities**: At kinematic [[Singularities]], a manipulator loses one or more DoF in its ability to move the end-effector in Cartesian space, regardless of joint velocities. Redundancy can help mitigate this.
  <br>

- **Control**: The DoF determines the number of independent variables that need to be controlled. [[Motion_Control|Control strategies]] often differ based on whether the system is fully actuated (number of actuators = DoF) or underactuated.
  <br>

The concept of DoF is crucial for understanding a robot's mobility, dexterity, and control complexity. It plays a fundamental role in the design and analysis of robotic systems, influencing their ability to perform tasks and interact with their environment.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

- **List all related kinematics and robotics concepts**:
  ```dataview
  LIST FROM #kinematics OR #robot-design WHERE contains(file.outlinks, [[Degrees_of_Freedom]])
